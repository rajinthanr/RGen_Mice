#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C"
{
#endif

    void button_Configuration(void);


    class Switches;
// so that we can declare the instance
extern Switches switches;
class Switches {
 public:
  explicit Switches(){};

  inline bool button_pressed() {
    return false;
    return HAL_GPIO_ReadPin(B_KEY_GPIO_Port, B_KEY_Pin) == GPIO_PIN_RESET;
  }

  void wait_for_button_press() {
    while (not(button_pressed())) {
      delay_ms(10);
    };
  }

  void wait_for_button_release() {
    while (button_pressed()) {
      delay_ms(10);
    };
  }

  void wait_for_button_click() {
    wait_for_button_press();
    wait_for_button_release();
    delay_ms(250);
  }

 private:
};

#ifdef __cplusplus
}
#endif

#endif
