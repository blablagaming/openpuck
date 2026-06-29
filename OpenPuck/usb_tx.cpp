#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

// One outbound ring per active HID destination. Only one USB mode is live at a time, and a mode presents at
// most CFG_TUD_HID HID interfaces plus the wake mouse, so a handful of channels covers every case. Channels are
// allocated lazily on first send to a given Adafruit_USBD_HID* and never freed (the object set is fixed for the
// session). The producer side runs under PRIMASK (loop AND the RF-decode path enqueue); the consumer is the
// single usbd task (tud_sof_cb), so head/tail need no further locking beyond the enqueue critical section.
#define TX_CHAN_MAX 8
#define TX_RING_N \
	4 // depth per destination; 250 Hz produced, ~1 kHz SOF drain -> stays shallow
#define TX_DATA_MAX \
	64 // CFG_TUD_HID_EP_BUFSIZE -- the largest report any mode sends (63 B + id)

struct TxItem {
	uint8_t rid;
	uint8_t len;
	uint8_t data[TX_DATA_MAX];
};
struct TxChan {
	Adafruit_USBD_HID *hid; // nullptr = free slot
	TxItem ring[TX_RING_N];
	volatile uint8_t head,
		tail; // head=next write, tail=next read; empty when equal
};
static TxChan g_chan[TX_CHAN_MAX];

static inline uint8_t txNext(uint8_t i)
{
	return (uint8_t)((i + 1) % TX_RING_N);
}

void usbTxHid(Adafruit_USBD_HID *hid, uint8_t rid, const void *data,
	      uint16_t len)
{
	if (!hid)
		return;
	if (len > TX_DATA_MAX)
		len = TX_DATA_MAX;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	// find this destination's channel, or claim a free one
	TxChan *c = nullptr;
	for (int i = 0; i < TX_CHAN_MAX; i++) {
		if (g_chan[i].hid == hid) {
			c = &g_chan[i];
			break;
		}
	}
	if (!c) {
		for (int i = 0; i < TX_CHAN_MAX; i++) {
			if (!g_chan[i].hid) {
				c = &g_chan[i];
				c->hid = hid;
				c->head = c->tail = 0;
				break;
			}
		}
	}
	if (c) {
		uint8_t h = c->head, nx = txNext(h);
		// full -> drop the oldest (advance tail), never the newest
		if (nx == c->tail)
			c->tail = txNext(c->tail);
		c->ring[h].rid = rid;
		c->ring[h].len = (uint8_t)len;
		if (len)
			memcpy(c->ring[h].data, data, len);
		c->head = nx;
	}
	__set_PRIMASK(pm);
}

// usbd task only (tud_sof_cb). Send at most one ready report per destination per SOF: after a send the
// endpoint stays busy until the host polls it (~1 ms), and SOF fires every ~1 ms, so this naturally paces each
// stream to the host's poll rate without ever blocking. A destination whose endpoint is busy is simply left
// for the next frame; it does not hold up the others.
void usbTxDrain(void)
{
	for (int i = 0; i < TX_CHAN_MAX; i++) {
		TxChan *c = &g_chan[i];
		if (!c->hid)
			continue;
		uint8_t t = c->tail;
		if (t == c->head)
			continue; // empty
		if (!c->hid->ready())
			continue; // endpoint busy / not mounted -> retry next SOF
		TxItem &it = c->ring[t];
		// sendReport copies into the endpoint buffer, so releasing the slot right after is safe.
		if (c->hid->sendReport(it.rid, it.data, it.len))
			c->tail = txNext(t);
	}
}

// Per-frame drain callbacks for non-HID senders (XInput endpoint, WebUSB blob). Registered at setup(),
// invoked from tud_sof_cb on the usbd task. Fixed, tiny capacity; registration is setup-only (single-threaded)
// so no locking is needed and the SOF reader sees a stable list.
#define TX_DRAIN_MAX 4
static usbTxDrainFn g_drainFns[TX_DRAIN_MAX];
static uint8_t g_nDrain;

void usbTxRegisterDrain(usbTxDrainFn fn)
{
	if (fn && g_nDrain < TX_DRAIN_MAX)
		g_drainFns[g_nDrain++] = fn;
}

// Drain trigger: a DEDICATED FreeRTOS task -- NOT tud_sof_cb. SOF proved unusable here: TinyUSB gates the SOF
// callback on `sof_consumer`, which configuration_reset() wipes on every bus reset (tu_varclr(&_usbd_dev)), so
// after the host's first reset tud_sof_cb silently stops firing and NO device->host data flows (0x45 input,
// the 0x79 connect report, streamed reports, the WebUSB blob -- all dead, while RF keeps working). A plain task
// is ungated and always runs. Crucially it is a SEPARATE task from loop(): if a sendReport ever blocks on a
// full usbd event queue, this task just blocks/yields -- loop() keeps running and feeding the ~8 s watchdog,
// which is the whole reason we moved sends off loop() in the first place.
static void usbTxTask(void *arg)
{
	(void)arg;
	for (;;) {
		usbTxDrain();
		for (uint8_t i = 0; i < g_nDrain; i++)
			g_drainFns[i]();
		vTaskDelay(1); // ~1 ms, matching the USB frame cadence
	}
}
void usbTxBegin(void)
{
	// 512 words (2 KB): covers the deepest drain path (webusbSendBlob's 115 B frame + the tud_* call chain).
	// TASK_PRIO_LOW = same as loop(): cooperative, won't preempt the RF poll timing; vTaskDelay yields each ms.
	xTaskCreate(usbTxTask, "usbtx", 512, NULL, TASK_PRIO_LOW, NULL);
}
