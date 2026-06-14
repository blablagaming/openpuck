#include "mode_xboxone.h"
#include "triton.h"
#include "config.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

XboxOneController g_xboxOneCtl;

// Xbox Series X|S HID gamepad descriptor: 16 buttons, 1 hat, 6 x 16-bit axes.
// Windows xinputhid.sys matches on HID gamepad (page 1, usage 5) + Xbox VID:PID
// and provides full XInput API including the guide button.
static const uint8_t GAMEPAD_HID_DESC[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};

// Right-pad mouse (same glide model as Xbox 360 / lizard mode)
static const uint8_t MOUSE_HID_DESC[] = { TUD_HID_REPORT_DESC_MOUSE() };
static Adafruit_USBD_HID g_gamepad;
static Adafruit_USBD_HID g_mouse;

// Xbox One HID gamepad button bits (16-bit field, ordered as Windows expects)
enum {
  XB1_A      = 0x0001,
  XB1_B      = 0x0002,
  XB1_X      = 0x0004,
  XB1_Y      = 0x0008,
  XB1_LB     = 0x0010,
  XB1_RB     = 0x0020,
  XB1_VIEW   = 0x0040,   // Back
  XB1_MENU   = 0x0080,   // Start
  XB1_GUIDE  = 0x0100,   // Home button
  XB1_L3     = 0x0200,
  XB1_R3     = 0x0400,
  XB1_DUP    = 0x0800,
  XB1_DDOWN  = 0x1000,
  XB1_DLEFT  = 0x2000,
  XB1_DRIGHT = 0x4000,
};

static uint16_t codeToXB1(uint8_t c){
  switch(c){
    case 1: return XB1_A; case 2: return XB1_B; case 3: return XB1_X; case 4: return XB1_Y;
    case 5: return XB1_LB; case 6: return XB1_RB; case 7: return XB1_L3; case 8: return XB1_R3;
    case 9: return XB1_VIEW; case 10: return XB1_MENU; case 11: return XB1_GUIDE;
    case 12: return XB1_DUP; case 13: return XB1_DDOWN; case 14: return XB1_DLEFT; case 15: return XB1_DRIGHT;
    default: return 0;
  }
}

static void rfXboxOneGamepad(const uint8_t* r){
  uint32_t b=btnsOf(r);
  uint16_t btn=0;
  if(b&TB_DUP)btn|=XB1_DUP;   if(b&TB_DDN)btn|=XB1_DDOWN; if(b&TB_DLF)btn|=XB1_DLEFT; if(b&TB_DRT)btn|=XB1_DRIGHT;
  if(b&TB_VIEW)btn|=XB1_MENU; if(b&TB_MENU)btn|=XB1_VIEW; if(b&TB_STEAM)btn|=XB1_GUIDE;
  if(b&TB_LB)btn|=XB1_LB;  if(b&TB_RB)btn|=XB1_RB;
  if(b&TB_L3)btn|=XB1_L3;  if(b&TB_R3)btn|=XB1_R3;
  uint16_t fA=g_abSwap?XB1_B:XB1_A, fB=g_abSwap?XB1_A:XB1_B, fX=g_abSwap?XB1_Y:XB1_X, fY=g_abSwap?XB1_X:XB1_Y;
  if(b&TB_A)btn|=fA; if(b&TB_B)btn|=fB; if(b&TB_X)btn|=fX; if(b&TB_Y)btn|=fY;
  if(b&TB_L4)btn|=codeToXB1(g_back[0]); if(b&TB_R4)btn|=codeToXB1(g_back[1]);
  if(b&TB_L5)btn|=codeToXB1(g_back[2]); if(b&TB_R5)btn|=codeToXB1(g_back[3]);
  uint8_t lt=trigU8(u16off(r,4)), rt=trigU8(u16off(r,6));
  int16_t lx=(int16_t)s16off(r,8), ly=(int16_t)s16off(r,10);
  int16_t rx=(int16_t)s16off(r,12), ry=(int16_t)s16off(r,14);
  // HID gamepad report: buttons[2] hat[1] lx[2] ly[2] rx[2] ry[2] lt[2] rt[2] = 15 B
  // Hat is always centered (8 = null) since dpad is button-based
  uint8_t rep[15];
  rep[0]=btn&0xFF; rep[1]=btn>>8; rep[2]=0x08;
  rep[3]=lx&0xFF; rep[4]=lx>>8; rep[5]=ly&0xFF; rep[6]=ly>>8;
  rep[7]=rx&0xFF; rep[8]=rx>>8; rep[9]=ry&0xFF; rep[10]=ry>>8;
  rep[11]=lt;     rep[12]=0;    rep[13]=rt;     rep[14]=0;
  if(g_gamepad.ready()) g_gamepad.sendReport(0, rep, sizeof rep);
}

// Right-pad -> mouse (same glide model as Xbox 360 / lizard)
static void rfXboxOneMouse(const uint8_t* r){
  uint32_t b=btnsOf(r);
  static int prx=0,pry=0; static bool prt=false; static float vx=0,vy=0,rmx=0,rmy=0; static uint8_t pmb=0;
  bool rtouch=b&TB_RPADT; int rx=s16off(r,22), ry=s16off(r,24);
  if(rtouch){ if(prt){ vx+=(rx-prx); vy+=(ry-pry); } prx=rx; pry=ry; } prt=rtouch;
  float mxf=vx/(float)(g_mDiv*10)+rmx, myf=-(vy/(float)(g_mDiv*10))+rmy;
  int dx=(int)mxf, dy=(int)myf; rmx=mxf-dx; rmy=myf-dy;
  if(dx>127)dx=127; if(dx<-127)dx=-127; if(dy>127)dy=127; if(dy<-127)dy=-127;
  float f=g_mFric/100.0f; vx*=f; vy*=f; if(vx>-1&&vx<1)vx=0; if(vy>-1&&vy<1)vy=0;
  uint8_t mb=((b&TB_RPADC)?1:0)|((b&TB_LPADC)?2:0);
  if(dx||dy||mb!=pmb){ pmb=mb;
    hid_mouse_report_t m; m.buttons=mb; m.x=(int8_t)dx; m.y=(int8_t)dy; m.wheel=0; m.pan=0;
    if(g_mouse.ready()) g_mouse.sendReport(0,&m,sizeof m);
  }
}

void XboxOneController::begin(){
  USBDevice.setID(0x045E, 0x0B12);   // Xbox Series X|S
  USBDevice.setDeviceVersion(0x0115);
  USBDevice.setManufacturerDescriptor("Microsoft");
  USBDevice.setProductDescriptor("Xbox Series X|S Controller");
  g_gamepad.setReportDescriptor(GAMEPAD_HID_DESC, sizeof GAMEPAD_HID_DESC);
  g_gamepad.setPollInterval(1); g_gamepad.begin();
  g_mouse.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
  g_mouse.setReportDescriptor(MOUSE_HID_DESC, sizeof MOUSE_HID_DESC); g_mouse.setPollInterval(1); g_mouse.begin();
}

void XboxOneController::onReport45(const uint8_t* rep, bool fresh, uint8_t bodyTlen){
  (void)fresh; (void)bodyTlen;
  rfXboxOneGamepad(rep); rfXboxOneMouse(rep);
}

void XboxOneController::task(){}
