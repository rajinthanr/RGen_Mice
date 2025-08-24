#ifndef SWITCHES_H
#define SWITCHES_H

#include "config.h"
#include "core.h"
#include "button.h"

/***
 * The Switches class looks after the multifunction analogue input on UKMARSBOT.
 *
 * A single analogue channel lets you examine four dip switches and a pushbutton
 * using a single ADC line.
 *
 * The dip switches short out combinations of resistors in a potential divider chain
 * and thus cause a different voltage to be presented to the ADC input pin. The
 * maximum voltage is applied when all switches are open. That voltage will be
 * about 66% of the full range of the analogue channel with the resistors chosen.
 *
 * The top resistor in this chain has a pushbutton in parallel so that pressing
 * the button pulls the input all the way up to the positive supply giving a
 * full scale adc reading.
 *
 * There is no debounce on this circuit or in the software. None has yet proven
 * necessary. The simplest option would be to place a small capacitor between
 * the ADC input and ground.
 *
 * NOTE: The switches class relies upon the ADC being updated regularly in the
 *       systick event.
 */

// we need a forward declaration...
class Switches;
// so that we can declare the instance
extern Switches switches;
class Switches
{
public:
  explicit Switches(uint8_t channel) : m_channel(channel) {};

  void update()
  {
  }

  int read()
  {
    update();

    return -1;
  }

  inline bool button_pressed()
  {
    if (is_key_pressed)
    {
      is_key_pressed = 0;
      return true;
    }
    else
      return false;
  }

  void wait_for_button_press()
  {
    while (not(button_pressed()))
    {
      delay_ms(10);
    };
  }

  void wait_for_button_release()
  {
    while (button_pressed())
    {
      delay_ms(10);
    };
  }

  void wait_for_button_click()
  {
    wait_for_button_press();
    wait_for_button_release();
    delay_ms(250);
  }

  // for testing
  int adc_reading()
  {
    update();
    return m_switches_adc;
  }

private:
  uint8_t m_channel = 255;
  int m_switches_adc = 0;
};

#endif