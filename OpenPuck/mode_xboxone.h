// mode_xboxone.h -- Xbox Series X|S HID gamepad personality (MODE_XBOXONE):
// HID gamepad interface + right-pad mouse. Windows xinputhid.sys binds by
// VID:PID (045E:0B12) + HID gamepad usage, providing full XInput with guide.
#pragma once
#include "controllers.h"
#include <stdint.h>

class XboxOneController : public IController {
public:
  void begin() override;
  void onReport45(const uint8_t* rep, bool fresh, uint8_t bodyTlen) override;
  void task() override;
};
extern XboxOneController g_xboxOneCtl;
