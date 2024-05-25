#include <stdint.h>
#include "esp_log.h"
#include "NimBLEDevice.h"

class Gamepad {

public:
  void init();

  bool isConnected();

  void setConnectCB(void(* callback)());
  void setDisconnectCB(void(* callback)());
  static void setGamepadData(uint8_t* dataptr);
  
  static void (* connectCB)();
  static void (* disconnectCB)();

  // sticks
  int16_t left_stick_y();
  int16_t left_stick_x();
  int16_t right_stick_y();
  int16_t right_stick_x();
  bool left_stick_button();
  bool right_stick_button();

  // triggers
  uint16_t left_trigger();
  uint16_t right_trigger();

  // bumpers
  bool left_bumper();
  bool right_bumper();

  // d-pad
  bool up();
  bool down();
  bool left();
  bool right();

  // buttons
  bool a();
  bool b();
  bool x();
  bool y();

  // menu buttons
  bool share();
  bool menu();
  bool xbox();

private:

  static uint8_t gamepad_data[];

};
