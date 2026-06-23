#include "ctrl_link.h"
#include "radio.h"
#include "ctrl_bonds.h"
#include "report45.h"
#include "deck_input.h"
#include "triton.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

int g_linkSlot = -1;
unsigned long g_linkAliveMs = 0;

// Link is considered up while we've heard a poll within this window (the puck polls ~250 Hz).
#define LINK_ALIVE_MS 1000u
// RX dwell per loop. One transaction (catch a poll, reply) per loop -> ~200 Hz, plenty for input, and
// short enough that USB/CDC servicing isn't starved. Tunable on hardware.
#define RX_WIN_US 4500u
// Time the puck's RX window stays open for our reply; keep the turnaround well under this.
static uint8_t g_seq = 0;
// Last time we caught a frame on the SESSION address (NOT the ch2 discovery beacon). This is the
// truthful "we're actually exchanging with the puck" signal that drives ctrlLinkUp() / the UI
// "available" tile -- hearing the discovery beacon alone does NOT mean the session works.
static unsigned long g_sessRxMs = 0;

// SESSION-CHANNEL HUNT. The E1 host frame advertises a "primary" channel, but that channel mostly
// carries the puck's E1/E2 keepalive -- the ACTIVE session (E3 input poll + our F1 reply) rides a small
// channel set, NOT necessarily the advertised primary (proven by the real-puck capture sniff1.json:
// E1/E2 on ch68, the E3/F1 session on ch48; and by puck_sniffer.ino's session-hunt). So once we adopt a
// session we DWELL across {primary, then these candidates} on the session address until a poll (E2/E3)
// appears, then PARK on that channel. If the parked channel goes dry (QoS hop / puck moved), un-park and
// hunt again. Mirrors puck_sniffer.ino HUNT_CH.
static const uint8_t HUNT_CH[] = { 2, 80, 18, 46, 76, 22, 68, 52, 55, 56, 57 };
static uint8_t g_huntIdx = 0;
static uint8_t g_parkCh = 0; // 0 = hunting; non-zero = parked on this channel
static unsigned long g_huntStepMs = 0;
#define HUNT_DWELL_MS 40u // per-channel dwell while hunting
#define RECONNECT_MS \
	4000u // dry this long -> abandon the session, re-catch E1 on ch2

// ---- build replies into rftx (ESB DPL RAM layout: [LENGTH][S1][payload...]) ----
static void buildF1()
{
	// payload = [0xF1][tlen][ttype=6][report45(46)]  -> length = 3 + REPORT45_LEN
	rftx[0] = (uint8_t)(3 + REPORT45_LEN); // LENGTH (payload byte count)
	rftx[1] = (uint8_t)((((g_pid++) & 3) << 1) | 1); // S1: PID<<1 | NO_ACK
	rftx[2] = 0xF1;
	rftx[3] = REPORT45_LEN; // TLV len
	rftx[4] = 6; // TLV type 6 (HID report)
	// neutral report (idle controller) when not forwarding or input has gone stale
	bool fresh = deckForwarding() && (millis() - deckLastInputMs() < 200u);
	if (fresh) {
		buildReport45(rftx + 5, g_seq++);
	} else {
		memset(rftx + 5, 0, REPORT45_LEN);
		rftx[5] = 0x45;
		rftx[6] = g_seq++;
	}
}

static void buildF3()
{
	// reply to E7 awake/version probe: rf_link reads the version at rfrx[6] (payload byte 4).
	rftx[0] = 5;
	rftx[1] = (uint8_t)((((g_pid++) & 3) << 1) | 1);
	rftx[2] = 0xF3;
	rftx[3] = 0;
	rftx[4] = 0;
	rftx[5] = 0;
	rftx[6] = 1; // protocol/version byte
}

// Transmit whatever is staged in rftx on the currently-configured channel/address, then return to RX
// config. Mirrors rf_link's TX path; radio is left disabled at exit.
static void txStaged()
{
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
}

// One RX->maybe-reply transaction on (base,prefix,ch). Returns the received type byte (rfrx[2]) or 0.
// `reply` selects whether we answer polls (true once connected) or just listen (discovery).
static uint8_t rxOnce(const uint8_t *base, uint8_t prefix, uint8_t ch,
		      uint16_t winUs, bool reply)
{
	rfConfig(ch);
	rfSetAddr(base, prefix);
	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	rfrx[0] = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	uint32_t t0 = micros();
	while (!NRF_RADIO->EVENTS_END && (uint32_t)(micros() - t0) < winUs) {
	}
	uint8_t type = 0;
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		g_rfRxCount++;
		bool crcok = NRF_RADIO->CRCSTATUS & 1;
		uint8_t rxlen = rfrx[0];
		if (crcok && rxlen && rxlen <= 96) {
			type = rfrx[2];
			if (reply) {
				// E1 = host-frame keepalive (no reply expected); follow channel hops.
				if (type == 0xE1) {
					uint8_t newCh = rfrx[11];
					if (newCh && newCh != g_sessCh)
						g_sessCh = newCh;
				} else if (type == 0xE7) {
					buildF3();
					txStaged();
				} else if (type >= 0xE0 && type <= 0xEF) {
					// E3 GET (and any other E-type poll): answer with input.
					buildF1();
					txStaged();
				}
			}
			return type;
		}
	}
	NRF_RADIO->TASKS_DISABLE = 1;
	RWAIT_DISABLED();
	NRF_RADIO->EVENTS_DISABLED = 0;
	return type;
}

// Listen on the shared "ibex"/ch2 rendezvous for a discovery E1 that matches a stored bond. On a match,
// adopt the advertised session base/prefix/channel and switch to connected.
static void discoveryScan()
{
	uint8_t t = rxOnce(g_rfBase, g_rfPrefix, 2, RX_WIN_US, false);
	if (t != 0xE1)
		return;
	const uint8_t *proteus =
		rfrx + 3; // payload [proteus_uuid 4][ibex_uuid 4]
	const uint8_t *ibex = rfrx + 7;
	int slot = ctrlBondMatch(proteus, ibex);
	if (slot < 0) {
		// We hear the puck's beacon but its UUIDs don't match any stored bond -- the controller
		// side of the pairing didn't land, or we're paired to a different puck. Very useful signal.
		static unsigned long lastMiss = 0;
		if (Serial.availableForWrite() > 40 &&
		    millis() - lastMiss > 1000) {
			lastMiss = millis();
			Serial.printf(
				"# E1 heard, no bond match: puuid=%02X%02X%02X%02X iuuid=%02X%02X%02X%02X\n",
				proteus[0], proteus[1], proteus[2], proteus[3],
				ibex[0], ibex[1], ibex[2], ibex[3]);
		}
		return;
	}
	// adopt the advertised session address (rfHostFrameOnce layout: ch@[11], base@[15..19], prefix@[19])
	uint8_t advCh = rfrx[11];
	memcpy(g_sessBase, rfrx + 15, 4);
	g_sessPrefix = rfrx[19];
	if (advCh)
		g_sessCh = advCh;
	g_linkSlot = slot;
	g_linkAliveMs = millis();
	g_parkCh = 0; // start hunting the session channel set from scratch
	g_huntIdx = 0;
	g_huntStepMs = millis();
	deckText("link: adopted session");
	if (Serial.availableForWrite() > 40)
		Serial.printf(
			"# LINK adopted slot=%d base=%02X%02X%02X%02X/%02X ch=%u\n",
			slot, g_sessBase[0], g_sessBase[1], g_sessBase[2],
			g_sessBase[3], g_sessPrefix, g_sessCh);
}

bool ctrlLinkUp()
{
	// "up" = we caught a real SESSION-address frame recently (a poll/keepalive on the adopted
	// base+channel), not merely the ch2 discovery beacon -- so the UI tile only goes tappable when
	// the puck is genuinely polling us.
	return g_linkSlot >= 0 && (millis() - g_sessRxMs) < LINK_ALIVE_MS;
}

void ctrlLinkTask()
{
	// nothing to do until at least one puck is bonded
	bool anyBond = false;
	for (int i = 0; i < NBOND; i++)
		if (g_bond[i].used)
			anyBond = true;
	if (!anyBond) {
		static unsigned long lastNb = 0;
		if (Serial.availableForWrite() > 40 &&
		    millis() - lastNb > 2000) {
			lastNb = millis();
			Serial.println(
				"# no bonds stored -- pair first (controller never got 0xEE)");
		}
		g_linkSlot = -1;
		return;
	}

	// 1 Hz status line for a serial monitor: searching vs connected, channel, RX count.
	{
		static unsigned long stMs = 0;
		static uint32_t lastRx = 0;
		if (Serial.availableForWrite() > 50 &&
		    millis() - stMs >= 1000) {
			stMs = millis();
			Serial.printf(
				"# link slot=%d %s ch=%u park=%u rx=%lu/s\n",
				g_linkSlot,
				ctrlLinkUp() ? "CONNECTED" : "searching",
				g_sessCh, g_parkCh,
				(unsigned long)(g_rfRxCount - lastRx));
			lastRx = g_rfRxCount;
		}
	}

	// No session adopted yet -> hunt the E1 host frame on the shared ch2/"ibex" rendezvous.
	if (g_linkSlot < 0) {
		discoveryScan();
		return;
	}

	// Adopted a session, but the puck has gone fully dry for a long time -> abandon it and re-catch the
	// E1 (its primary channel may have changed). Only after RECONNECT_MS so a hunt sweep has a chance.
	if ((millis() - g_linkAliveMs) >= RECONNECT_MS) {
		g_linkSlot = -1;
		g_parkCh = 0;
		g_huntIdx = 0;
		deckText("link: lost, rescanning");
		if (Serial.availableForWrite() > 40)
			Serial.println("# LINK lost, rescanning ch2");
		return;
	}

	// Pick the channel to listen on: parked channel if we have one, else sweep {primary, HUNT_CH...}.
	uint8_t tryCh = g_parkCh ? g_parkCh :
				   (g_huntIdx == 0 ?
					    g_sessCh :
					    HUNT_CH[(g_huntIdx - 1) %
						    (uint8_t)sizeof HUNT_CH]);

	uint8_t t = rxOnce(g_sessBase, g_sessPrefix, tryCh, RX_WIN_US, true);

	if (t == 0xE1) {
		// keepalive beacon on the session base: refreshes the advertised primary channel (rxOnce already
		// copied it into g_sessCh) + proves the puck is present, but it is NOT a poll -> keep hunting.
		g_linkAliveMs = millis();
	} else if (t >= 0xE0 && t <= 0xEF) {
		// a real poll (E2/E3/E7) -> the session lives on THIS channel; PARK and answer it from here.
		bool wasUp = (millis() - g_sessRxMs) < LINK_ALIVE_MS;
		g_parkCh = tryCh;
		g_sessCh = tryCh;
		g_linkAliveMs = g_sessRxMs = millis();
		if (!wasUp) {
			deckText("link: session live");
			if (Serial.availableForWrite() > 40)
				Serial.printf(
					"# SESSION live: type=%02X ch=%u base=%02X%02X%02X%02X/%02X\n",
					t, tryCh, g_sessBase[0], g_sessBase[1],
					g_sessBase[2], g_sessBase[3],
					g_sessPrefix);
		}
	} else {
		// nothing this dwell. parked channel gone dry -> resume hunting (QoS hop / puck moved)
		if (g_parkCh && (millis() - g_sessRxMs) >= LINK_ALIVE_MS)
			g_parkCh = 0;
		if (!g_parkCh && (millis() - g_huntStepMs) >= HUNT_DWELL_MS) {
			g_huntStepMs = millis();
			g_huntIdx++;
		}
	}
}
