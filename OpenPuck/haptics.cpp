#include "haptics.h"
#include "bonds.h"
#include "config.h"
#include "rf_link.h"

// USBDevice.suspended() -> autonomous controller power-off on host sleep
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

uint8_t g_relayOp = 0xE3; // E3 poll
// relay sub-TLV TYPE byte. Vestigial: rfConnFlushRelay derives the on-air type from the report id (0x05 for
// actuators/haptics <0x87, 0x01 for config/settings/LED >=0x87). Still exposed over WebUSB diagnostics.
uint8_t g_relaySub = 0x05;
volatile uint8_t g_testHaptic = 0;
volatile uint8_t g_hapticStop = 0;
// Per-slot reconnect block. 0 = idle; non-zero = drop haptics aimed at this slot until millis() catches up.
unsigned long g_hapticBlockUntil[NSLOT] = { 0 };

// Controller power-off. CONFIRMED from a real Windows USB capture of the Valve puck: Steam's "turn off
// controller" is the single feature-0x01 command 0x9F with payload ASCII "off!" (6F 66 66 21). The dongle
// forwards host feature reports verbatim, so the controller acts on report 0x9F directly -- we relay it the
// same way (E3 SET sub-TLV). The wire relay is NO-ACK, so send a small burst: a single lost frame must not
// leave the controller on.
void hapticSendShutdown()
{
	static const uint8_t OFF[4] = { 0x6f, 0x66, 0x66, 0x21 }; // "off!"
	// broadcast: every connected controller should power off, not just the slot the shutdown was triggered
	// from (the trigger can come from a Steam interface, the test button, or host-suspend -- any of which
	// "logically" means "all controllers off").
	for (uint8_t i = 0; i < HAPTIC_SHUTDOWN_SHOTS; i++)
		relayEnqueue(0x9F, OFF, sizeof OFF, 0xFF);
}

// millis of last 0x82 haptic OUTPUT relayed (Steam mode)
static unsigned long g_haptic82Ms = 0;

// a non-zero 0x82 haptic is currently active (awaiting host stop)
static bool g_haptic82On = false;

// millis of last translated host rumble (0x80), per-slot (4 XInput interfaces each have their own stream)
static unsigned long g_rumble80Ms[NSLOT] = { 0 };

// Steam/Triton rumble is latched on until an explicit zero report; tracked per-slot so each controller's
// stuck-rumble watchdog is independent
static bool g_rumble80On[NSLOT] = { false, false, false, false };

// when to fire the next post-reconnect haptic re-init (0 = none scheduled)
static unsigned long g_reinitAt = 0;

// how many re-init shots remain in this connect window
static uint8_t g_reinitLeft = 0;

// haptic activity happened -> arm a clear once it goes idle (catches a
// latch that engaged during/after use, even seconds after connect)
static bool g_hapClearArmed = false;

// ---- relay ring: multi-producer (USB ISR + loop-context console/xinput), single consumer (poll flush) ----
// Producers serialize through a brief PRIMASK critical section (copy is <=62 bytes); the consumer only ever
// touches the tail entry, which no producer writes while the ring isn't full -- so a flush can't be torn.
struct RelayMsg {
	uint8_t rid, len, slot;
	uint8_t data[RELAY_MAXP];
};
// deep enough to hold a full Steam settings/LED transaction burst without loss
#define RELAY_QLEN 32
static RelayMsg g_rq[RELAY_QLEN];
static volatile uint8_t
	g_rqHead = 0,
	g_rqTail = 0; // head=next write, tail=next read; empty when equal
static inline uint8_t rqNext(uint8_t i)
{
	return (uint8_t)((i + 1) % RELAY_QLEN);
}

bool relayPending()
{
	return g_rqHead != g_rqTail;
}
bool relayEnqueue(uint8_t rid, const uint8_t *payload, uint8_t plen,
		  uint8_t slot)
{
	// 60B is the RF frame ceiling; longer can't be relayed
	if (plen > RELAY_MAXP)
		plen = RELAY_MAXP;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t h = g_rqHead, nx = rqNext(h);
	// Full -> evict the OLDEST entry, never the newest. Steam sends commands as bursts where the meaningful
	// frame comes LAST (a settings transaction ends with its commit; a haptic stream ends with its stop), so
	// dropping the oldest guarantees the most recent command (LED commit, haptic stop) always lands.
	if (nx == g_rqTail)
		g_rqTail = rqNext(g_rqTail);
	g_rq[h].rid = rid;
	g_rq[h].len = plen;
	g_rq[h].slot = slot;
	if (plen)
		memcpy(g_rq[h].data, payload, plen);
	g_rqHead = nx;
	// Any haptic relay (Steam OR Xbox rumble OR test) arms the idle-clear and refreshes its timer -- so the
	// during-use buzz gets cleared in EVERY USB mode. The re-init's own 0x81/0x87 deliberately don't match.
	if (rid == 0x82 || rid == 0x80) {
		g_haptic82Ms = millis();
		g_hapClearArmed = true;
	}
	__set_PRIMASK(pm);
	return true;
}

#if OPK_LOG
// diagnostic capture: a ring of the last OUTPUT reports Steam sends (rid/slot/bytes/ms), dumped with 'H'.
struct HapLog {
	uint32_t ms;
	uint8_t slot, rid, n, b[16];
}; // 16 payload bytes: capture full 0x87 settings frames
// Big always-on ring: log EVERYTHING from boot (Steam writes, our TX-to-controller, link edges) so a rare
// reconnect-buzz can be caught after the fact -- the trigger happens moments after boot, while this RAM is
// fresh, and we dump it once the panel reconnects. 4096 * 20B ~= 80KB.
#define HAPLOG_N 4096
static HapLog g_hapLog[HAPLOG_N];
static uint16_t g_hapHead = 0;
static uint16_t g_hapTail =
	0; // live/dump drain cursor (loop-context reader; chases g_hapHead)

void hapLogAdd(uint8_t slot, uint8_t rid, const uint8_t *b, uint16_t n)
{
	// Written from the USB SET ISR (handleSet) AND loop context (relay flush / link edges) -> guard g_hapHead.
	// Special slot markers for the diagnostic capture: 0xFE = a frame WE transmitted to the controller (TX
	// relay); 0xFD = a link state edge (b[0]=1 up, 0 down). Real Steam writes use the interface index (0..3).
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	HapLog &e = g_hapLog[g_hapHead];
	e.ms = millis();
	e.slot = slot;
	e.rid = rid;
	e.n = (uint8_t)(n > 255 ? 255 : n);
	for (int i = 0; i < 16; i++)
		e.b[i] = (i < (int)n) ? b[i] : 0;
	g_hapHead = (uint8_t)((g_hapHead + 1) %
			      (sizeof g_hapLog / sizeof g_hapLog[0]));
	__set_PRIMASK(pm);
}
void hapticDumpLog()
{
	const uint16_t N = HAPLOG_N;
	uint32_t now = millis();
	Serial.printf("# --- capture history (now=%lu, curSlot=%d) ---\n",
		      (unsigned long)now, g_curSlot);
	for (uint16_t i = 0; i < N; i++) {
		HapLog &e = g_hapLog[(uint16_t)((g_hapHead + i) % N)];
		if (!e.ms && !e.rid)
			continue;
		Serial.printf("# -%lums if%u rid=%02X n=%u:",
			      (unsigned long)(now - e.ms), e.slot, e.rid, e.n);
		for (uint8_t j = 0; j < 16 && j < e.n; j++)
			Serial.printf(" %02X", e.b[j]);
		Serial.println();
	}
	Serial.println("# --- end ---");
}
// ---- drain cursor: stream entries to the WebUSB panel. resetDrain(false)=from "now" (live only);
//      resetDrain(true)=from the OLDEST entry (dump the whole ring from boot). pull skips empty slots. ----
void hapLogResetDrain(bool fromBoot)
{
	// oldest slot (the next-to-overwrite holds it)
	g_hapTail = fromBoot ? (uint16_t)((g_hapHead + 1) % HAPLOG_N) :
			       // "now"
			       g_hapHead;
}
bool hapLogPull(uint32_t *logMs, uint8_t *slot, uint8_t *rid, uint8_t *n,
		uint8_t bytes16[16])
{
	while (g_hapTail != g_hapHead) {
		HapLog &e = g_hapLog[g_hapTail];
		g_hapTail = (uint16_t)((g_hapTail + 1) % HAPLOG_N);
		if (!e.ms && !e.rid)
			continue; // skip empty slot (ring not yet full)
		// Return the ABSOLUTE log time (millis since boot), not "age now". The panel drains in batches 100ms
		// apart, so an age computed here would jump between batches; the panel computes age vs the newest entry.
		*logMs = e.ms;
		*slot = e.slot;
		*rid = e.rid;
		*n = (e.n > 16) ? 16 : e.n;
		memcpy(bytes16, e.b, 16);
		return true;
	}
	return false;
}
#endif // OPK_LOG

// Per-slot helpers. slot==-1 (default) checks the CURRENT poll slot (g_curSlot), used by flush-time code
// paths that don't have a slot in hand. Callers with a real slot pass it in.
bool hapticLinkUp(int slot)
{
	int s = (slot >= 0) ? slot : g_curSlot;
	if (s < 0 || s >= NSLOT)
		return false;
	return g_slot[s].used && (millis() - g_connReplyMs[s]) < 300;
}
bool haptic82Blocked(int slot)
{
	int s = (slot >= 0) ? slot : g_curSlot;
	if (s < 0 || s >= NSLOT)
		return true;
	return !hapticLinkUp(s) ||
	       (g_hapticBlockUntil[s] &&
		(int32_t)(millis() - g_hapticBlockUntil[s]) < 0);
}
// "Is haptics from this USB interface's slot allowed through?" -- the slot must be currently connected.
bool hapticRelaySlotOk(int slot)
{
	return slot >= 0 && slot < NSLOT && hapticLinkUp(slot);
}
static bool haptic82PayloadOn(const uint8_t *p, uint16_t n)
{
	if (n < 3)
		return false;
	for (uint16_t i = 2; i < n; i++)
		if (p[i])
			// observed form is [01 01 gain], but treat any trailing non-zero as active
			return true;
	return false;
}
static void hapticCancelPendingOn()
{
	// void queued ON entries (stale Steam haptics / translated rumble across a reconnect)
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	for (uint8_t i = g_rqTail; i != g_rqHead; i = rqNext(i)) {
		RelayMsg &m = g_rq[i];
		if (m.rid == 0x82) {
			bool on = false;
			for (uint8_t j = 2; j < m.len; j++)
				if (m.data[j]) {
					on = true;
					break;
				}
			if (on)
				m.rid = 0;
		}
		if (m.rid == 0x80) {
			bool on = false;
			for (uint8_t j = 0; j < m.len; j++)
				if (m.data[j]) {
					on = true;
					break;
				}
			if (on)
				m.rid = 0;
		}
	}
	__set_PRIMASK(pm);
}
void haptic82HostReport(const uint8_t *p, uint16_t n, int slot)
{
	// the on/off flag is global -- one relay stream keeps the watchdog armed across slots
	(void)slot;
	if (n < 3)
		return;
	g_haptic82Ms = millis();

	// any haptic activity arms a clear when it next goes idle (kills a latch from this use)
	g_hapClearArmed = true;
	// Track on/off only. Do NOT synthesize a stop burst: Steam's own stop is forwarded verbatim, and each extra
	// 0x82 is a discrete pad click -- the spurious end-of-movement "click"/buzz the real puck never produces.
	g_haptic82On = haptic82PayloadOn(p, n);
}
bool hapticSteamRumble(uint16_t lowFreq, uint16_t highFreq, uint8_t slot)
{
	if (slot >= NSLOT)
		return false;
	// user rumble-strength scale (percent; 200 = double). Clamp to 16-bit.
	if (g_rumbleScale != 100) {
		uint32_t l = (uint32_t)lowFreq * g_rumbleScale / 100,
			 h = (uint32_t)highFreq * g_rumbleScale / 100;
		lowFreq = (l > 0xFFFF) ? 0xFFFF : (uint16_t)l;
		highFreq = (h > 0xFFFF) ? 0xFFFF : (uint16_t)h;
	}
	bool on = lowFreq || highFreq;
	// Per-slot settle gate (the per-slot reconnect block + link-up check). 0x82 haptics in Steam mode use the
	// same gate; for XInput, the host only sends a stream while a controller is connected, so this also doubles
	// as "no controller here, no relay".
	if (on && haptic82Blocked(slot))
		return false;
	if (!on && !hapticLinkUp(slot))
		return false;

	// SDL's current Steam/Triton structs define output report 0x80 as:
	//   type, uint16 intensity, {uint16 speed, int8 gain} left/right.
	// We map conventional gamepad low/high-frequency motors to left/right speeds and use max as intensity.
	uint16_t intensity = lowFreq > highFreq ? lowFreq : highFreq;
	uint8_t p[9];

	// haptic_type_t::HAPTIC_TYPE_RUMBLE; 0 is the off/zero report
	p[0] = on ? 0x04 : 0x00;
	p[1] = (uint8_t)(intensity & 0xFF);
	p[2] = (uint8_t)(intensity >> 8);
	p[3] = (uint8_t)(lowFreq & 0xFF);
	p[4] = (uint8_t)(lowFreq >> 8);
	p[5] = 0;
	p[6] = (uint8_t)(highFreq & 0xFF);
	p[7] = (uint8_t)(highFreq >> 8);
	p[8] = 0;
	if (!relayEnqueue(0x80, p, sizeof p, slot))
		return false;
	g_rumble80Ms[slot] = millis();
	g_rumble80On[slot] = on;
	return true;
}
// Queue a pending test-haptic / stop relay (runs inside the poll cadence -- never at raw loop rate). Test
// haptics broadcast to all connected slots (slot 0xFF); the stop frame is broadcast too (a stuck latch can
// affect any controller, and the haptic-engine clear-re-init is settings-only so it's harmless on healthy
// ones).
void rfConnQueueHapticRelay()
{
	if (relayPending())
		return; // host relays first; injectables wait for an idle cycle
	static const uint8_t HAP_ON[3] = { 0x01, 0x01, 0xF7 };
	static const uint8_t HAP_OFF[3] = { 0x01, 0x01, 0x00 };
	if (g_testHaptic) {
		if (relayEnqueue(0x82, HAP_ON, 3, 0xFF))
			g_testHaptic--;
	} else if (g_hapticStop && !g_xbox) {
		if (relayEnqueue(0x82, HAP_OFF, 3, 0xFF))
			g_hapticStop--;
	}
}
// rfConnFlushRelay(ch, s1): scan the queue for the first entry that targets the CURRENT slot (g_curSlot),
// or the broadcast sentinel 0xFF. Emit it on that slot's session address (so each controller only sees
// its own relay). With 4 connected slots the per-slot relay rate is 1/4 of the per-cycle rate; sustained
// buzz streams are still 1 packet/cycle/slot which the controller's haptic engine handles fine.
void rfConnFlushRelay(uint8_t ch, uint8_t s1)
{
	int cur = (g_curSlot >= 0 && g_curSlot < NSLOT) ? g_curSlot : 0;
	while (g_rqTail != g_rqHead) {
		RelayMsg &m = g_rq[g_rqTail];

		// rid 0 = entry voided by hapticCancelPendingOn -> skip, take the next
		if (m.rid) {
			// Only relay entries for the CURRENT slot (or broadcast 0xFF). If the entry is for
			// another slot, leave it in the queue -- its slot's turn will pick it up.
			if (m.slot != 0xFF && m.slot != cur) {
				// advance past it without consuming; the per-slot flush is the consumer
				g_rqTail = rqNext(g_rqTail);
				continue;
			}
			uint8_t rl = m.len;
			if (rl > RELAY_MAXP)
				rl = RELAY_MAXP;
			// On-air sub-TLV framing. CONFIRMED from real puck<->controller sniffs (puck_sniffer): a command LANDS on
			// the controller only with the type-01 + inner-len form  E3 [2+rl][01][rid][innerlen][data]; with the legacy
			// form  E3 [1+rl][05][rid][data]  (no inner-len) the controller DISCARDS any 0x87+ command (it reads data[0]
			// as the length).
			//
			// WHITELIST the landing form to EXACTLY the two commands that need it; everything else keeps legacy form:
			//   * LED brightness  -- report 0x87 whose first register byte is 0x2D
			//   * controller power-off -- report 0x9F ("off!")
			// Steam ALSO sends other 0x87 passthrough writes during play -- the haptic-config block (reg 0x30 =
			// IMU/subsystem enable, 0x34/0x35 = haptic amplitude). Landing those regresses BOTH: a landed 0x30 FREEZES
			// the IMU (gyro stops) and landed 0x34/0x35 = the connect buzz. So keep this list tight.
			bool land01 =
				(m.rid == 0x9F) ||
				(m.rid == 0x87 && rl >= 1 && m.data[0] == 0x2D);
			uint8_t p[5 + RELAY_MAXP], plen;
			if (land01) {
				p[0] = g_relayOp;
				p[1] = (uint8_t)(2 + rl);
				p[2] = 0x01;
				p[3] = m.rid;
				p[4] = rl;
				memcpy(p + 5, m.data, rl);
				plen = (uint8_t)(5 + rl);
			} else {
				p[0] = g_relayOp;
				p[1] = (uint8_t)(1 + rl);
				p[2] = 0x05;
				p[3] = m.rid;
				memcpy(p + 4, m.data, rl);
				plen = (uint8_t)(4 + rl);
			}
			// log what we actually TX to the controller (slot 0xFE) for the buzz hunt
			hapLogAdd(0xFE, m.rid, m.data, rl);

			// copied out -> release the slot before the TX
			g_rqTail = rqNext(g_rqTail);
			// s1 carries a PID distinct from the GET poll that follows (caller cycles it), so the controller's ESB
			// dedup never mistakes the GET for a retransmit of this relay. 80us RX window: the relay is NO-ACK, so
			// don't burn the full ~1.2ms reply window (that halves the poll rate during haptics).
			rfConnTx(ch, s1, p, plen, 80);

			// ONE relay per poll cycle (matches the real puck's pacing)
			return;
		}
		g_rqTail = rqNext(g_rqTail);
	}
}

// Haptic-subsystem RE-INIT: the exact sequence Steam sends (captured on hardware) when it (re)takes control,
// which clears a stuck haptic script on the controller -- a 0x81 reset action plus 0x87 writes to the haptic
// registers (30/07/08/31/52, 18/2e/34/35). Replayed to recover from the latched-buzz the controller falls
// into across a reconnect. Brightness (0x87 reg 2d) is deliberately OMITTED so we don't stomp the LED.
//
// The 0x87 frames below go out on LEGACY (type-05) framing (the whitelist lands 0x87 only for brightness
// 0x2D), so the controller DISCARDS them -- kept verbatim only to match the captured sequence. The effective
// re-init is the three 0x81 frames. (Landing the 0x30 here would freeze the gyro.)
//
// slot defaults to 0xFF (broadcast to all connected) -- the clear re-init is a settings-only reset and
// is harmless on healthy controllers, so it's worth re-initializing every slot the firmware knows about.
void hapticReinit(uint8_t slot)
{
	static const uint8_t H30[] = { 0x30, 0x00, 0x00, 0x07, 0x07,
				       0x00, 0x08, 0x07, 0x00, 0x31,
				       0x02, 0x00, 0x52, 0x03, 0x00 };
	static const uint8_t H18[] = { 0x18, 0x00, 0x00, 0x2e, 0x00,
				       0x00, 0x34, 0xff, 0xff, 0x35,
				       0xff, 0xff, 0x34, 0xff, 0xff };
	static const uint8_t H35[] = { 0x35, 0xff, 0xff, 0x2e, 0x00, 0x00 };
	static const uint8_t T81A[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	static const uint8_t T81B[] = {
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	// reset action (FUN_0001f554) -- Steam sends this first
	relayEnqueue(0x81, nullptr, 0, slot);
	relayEnqueue(0x87, H30, sizeof H30, slot);

	// haptic config (enabled/amplifier/gain): the part that clears a latch
	relayEnqueue(0x87, H18, sizeof H18, slot);
	relayEnqueue(0x87, H35, sizeof H35, slot);
	relayEnqueue(0x81, T81A, sizeof T81A, slot);
	relayEnqueue(0x81, T81B, sizeof T81B, slot);
}
void hapticInit()
{
	g_rqHead = g_rqTail = 0;
	g_haptic82On = false;
	for (int s = 0; s < NSLOT; s++) {
		g_rumble80On[s] = false;
		g_rumble80Ms[s] = 0;
	}
	// boot: block stale Steam 0x82 until link stable (every slot)
	for (int s = 0; s < NSLOT; s++)
		g_hapticBlockUntil[s] = millis() + HAPTIC_RECONNECT_BLOCK_MS;
	// NO fabricated stop burst. USB capture proves Steam only ever sends 0x82 [01 01 f7] pulses -- never a
	// zero-gain [01 01 00] "stop". An invented stop frame at boot/connect is a likelier cause of the connect-time
	// buzz than a cure. Be a pure pass-through of Steam's haptics, like the real puck.
	g_hapticStop = 0;
}
// Arm the post-(re)connect haptic block + schedule the clearing re-init. Called from rf_link the moment a
// controller reply arrives after a gap (the reliable reconnect signal), and as a backup on hapticTask's
// link-up edge. Idempotent -- safe to call repeatedly. Per-slot: only the slot that just reconnected is
// blocked, the other slots keep relaying. The scheduled re-init is global (broadcast) so any slot's
// latch gets cleared.
void hapticOnReconnect(int slot)
{
	if (slot < 0 || slot >= NSLOT)
		return;
	// no haptics relayed for the next 3s (per-slot)
	g_hapticBlockUntil[slot] = millis() + HAPTIC_RECONNECT_BLOCK_MS;
	g_haptic82On = false;
	g_rumble80On[slot] = false;
	g_rumble80Ms[slot] = 0;

	// drop any haptic ON queued before the link came up
	hapticCancelPendingOn();
	// Re-init the haptic engine repeatedly across the settle window rather than waiting for the block to end: the
	// brief connect buzz engages early (during the block, controller-internal), so a single late shot misses it.
	// The re-init is settings only (no haptic play) and Steam haptics are blocked throughout, so it can't buzz.
	g_reinitAt = millis() + 200u; // first reset ~200ms after (re)connect

	// then every HAPTIC_REINIT_GAP_MS across the window
	g_reinitLeft = HAPTIC_REINIT_SHOTS;
	uint8_t mk = 2;

	// capture marker: RECONNECT detected (block+reinit armed)
	hapLogAdd(0xFD, 0xEE, &mk, 1);
}
void hapticTask()
{
	// Per-slot link-edge detect: any slot going from "down" to "up" gets the haptic block + re-init
	// scheduled for that slot. The rf_link decode already calls hapticOnReconnect on the first F-reply
	// after a gap (reliable); this loop is a backup for slots whose link-up the decode didn't see directly
	// (e.g. a long gap followed by a quiet stretch).
	static bool wasHapticLinkUp[NSLOT] = { 0 };
	for (int s = 0; s < NSLOT; s++) {
		if (!g_slot[s].used)
			continue;
		bool up = hapticLinkUp(s);
		if (up && !wasHapticLinkUp[s]) {
			uint8_t mk = 1;
			hapLogAdd(0xFD, 0xEE, &mk, 1);
			hapticOnReconnect(s);
		}
		if (!up && wasHapticLinkUp[s]) {
			uint8_t mk = 0;
			hapLogAdd(0xFD, 0xEE, &mk, 1);
		}
		wasHapticLinkUp[s] = up;
	}
	// Proactive haptic re-init (scheduled at (re)connect by hapticOnReconnect): the re-init is
	// settings-only so broadcasting to all connected slots is fine, and it's the same per-slot
	// re-init the controller would have received on its own connect.
	if (g_reinitAt && anySlotLinkUp() &&
	    (int32_t)(millis() - g_reinitAt) >=
		    0) { // proactive haptic re-init across the connect window
		hapticReinit();
		g_reinitAt = (g_reinitLeft && --g_reinitLeft) ?
				     (millis() + HAPTIC_REINIT_GAP_MS) :
				     0;
	}
	// Controller power-off on host SLEEP: send the power-off command (0x9F "off!") the instant the USB bus
	// suspends, like the real puck. BUT only when USB power (VBUS) is still present -- i.e. a genuine host sleep,
	// NOT a cable unplug. Pulling the dongle ALSO trips the suspend edge (in the brief window it runs on residual
	// power), and we must NOT kill the controller then; it should only power off on a shutdown command or a real
	// host sleep. VBUSDETECT is 1 while the cable still delivers 5V, 0 once unplugged. wasSusp starts true so a
	// boot-into-suspended state never false-fires.
	static bool wasSusp = true;
	bool susp = USBDevice.suspended();
	bool vbus = (NRF_POWER->USBREGSTATUS &
		     POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
	if (susp && !wasSusp && vbus)
		hapticSendShutdown();
	wasSusp = susp;
	// Steam-mode: host went quiet -> mark the 0x82 stream inactive. Do NOT synthesize a stop: trackpad haptics
	// are one-shot pulses, so firing a 0x82-zero ~HAPTIC_QUIET_MS after a swipe ends is the extra end-of-movement
	// click the real puck doesn't make. Steam forwards its own stop for any sustained haptic.
	if (!g_xbox && g_haptic82On &&
	    millis() - g_haptic82Ms > HAPTIC_QUIET_MS)
		g_haptic82On = false;
	// Per-slot stuck-rumble watchdog: if a controller's rumble stream has been "on" without a refresh for
	// 2.5s, force a zero report to clear the controller's latch. The previous single-stream watchdog
	// couldn't cover a 4-XInput scenario where only one slot is stuck (it would only fire when the
	// merged-flag went stale).
	for (int s = 0; s < NSLOT; s++) {
		if (g_rumble80On[s] && millis() - g_rumble80Ms[s] > 2500u)
			hapticSteamRumble(0, 0, (uint8_t)s);
	}
	// Haptic activity has gone idle for a while -> fire one re-init to clear any latch it left behind (the buzz
	// that engages during/after use and won't self-clear, incl. after a mode switch). Fires only after a quiet
	// gap, so it never interrupts active haptics; the brightness-less re-init is silent (settings, no play, no
	// LED). Runs in ALL modes (the controller-side latch is mode-independent).
	if (g_hapClearArmed &&
	    (millis() - g_haptic82Ms) > HAPTIC_CLEAR_IDLE_MS) {
		g_hapClearArmed = false;
		hapticReinit();
	}
}
