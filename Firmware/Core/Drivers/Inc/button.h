#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
#include "core.h"
extern "C" {
#endif

void button_Configuration(void);

class Switches;
extern Switches switches;

class Switches {
public:
  explicit Switches(){};

  bool key_pressed() {
    return HAL_GPIO_ReadPin(B_KEY_GPIO_Port, B_KEY_Pin) == GPIO_PIN_SET;
  }

  bool boot_pressed() {
    return HAL_GPIO_ReadPin(B_BOOT_GPIO_Port, B_BOOT_Pin) == GPIO_PIN_SET;
  }

  void wait_for_key_press() {
    while (not(key_pressed())) {
      delay_ms(10);
    };
  }

  void wait_for_key_release() {
    while (key_pressed()) {
      delay_ms(10);
    };
  }

  void wait_for_key_click() {
    wait_for_key_press();
    wait_for_key_release();
    delay_ms(250);
  }

private:
};

#ifdef __cplusplus
}
#endif

#endif
