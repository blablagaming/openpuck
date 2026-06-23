#include "identity.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

char g_unit[16];
char g_board[16];

// 0x83 attributes for the CONTROLLER, as [tag][u32-LE] records (the format Steam's GET_ATTRIBUTES
// returns): tag 01 = product id (0x1302, vs the puck's 0x1304); tag 02 = capabilities; tag 0A = build
// id; tag 04 = radio/firmware build = 0x6A18D057 (the IBEX image version @0x120, NOT the puck's
// 0x6A18D053 -- reporting the puck build is what made Steam nag "update firmware" and refuse to pair);
// tag 09 = board rev. The puck blob is otherwise carried over.
// LIVE-ITERATION: a real controller's 0x83 may carry EXTRA records (e.g. a separate main-firmware build)
// this puck-derived blob lacks. If Steam still won't pair or still nags, capture the real controller's
// response (`scmd 1302 --up 1 83`) and paste its exact bytes here -- the array length auto-propagates
// via sizeof, so a longer blob needs no other change.
const uint8_t ATTR83[] = { 0x01, 0x02, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
			   0x00, 0x0A, 0xF2, 0xF9, 0xD2, 0x68, 0x04, 0x57, 0xD0,
			   0x18, 0x6A, 0x09, 0x47, 0x00, 0x00, 0x00 };
const uint16_t ATTR83_LEN = sizeof ATTR83;

void genSerial()
{
	uint32_t id = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];
	snprintf(g_unit, sizeof g_unit, "FXA99602%05lX",
		 (unsigned long)(id & 0xFFFFF));
	snprintf(g_board, sizeof g_board, "MXA99602%05lX",
		 (unsigned long)(id & 0xFFFFF));
}
