#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include "core.h"
#include "wall_handle.h"

/**
 *
 * The Sensors class uses the data gathered by the AnalogueConverter to
 * provide basic sensor functions for UKMARSBOT. The sensors class tells
 * the robot about its environment both external and internal. In the
 * case of a micromouse the wall sensors are the primary means of
 * perception. For a line follower, there will be line and marker sensors.
 *
 * Internally, the sensors monitor battery voltage
 *
 * You must call begin() before using the sensors.
 * You must call update() in every control cycle. From systick() normally.
 *
 * Normally, each of the four sensors (two side and two front) would get
 * values for the corresponding channel by calling the AnalogueConverter's
 * get_raw() method. That will return the difference between the lit and
 * dark readings for the channel. Thus a close wall will give a larger
 * value and a distant wall will give a small value. The get_raw()
 * method will always return a value that is at least 1.
 *
 * The sensor are used in many tasks so look in the code for the various
 * convenience methods provided.
 *
 * TODO: it is not clear to me that all these methods should be in this class
 *
 * The normal response of the sensor is very non-linear and the gain - that is
 * the change in value for a small shange in distance - will vary greatly
 * as the distance changes.
 *
 * A linearisation function is provided in the get_distance() method. This
 * uses an approximation that is based on the inverse square root of the
 * sensor reading. You will need to supply a scaling constant that is
 * typically in the range 500-1500 and will be dependent on the
 * characteristics of the sensors you use. Do your own calibration and
 * define a constant in the robot config file. This method is not tied
 * to any sensor in particular. Just give it the calibration constant and
 * the sensor value. You can, for example, use it to linearise the fron
 * sum.
 *
 * The linearised value is limited to 200mm because the signal to noise of
 * small sensor readings is poor and you are unlikely to be able to reliably
 * measure distance out that far with the standard sensors.
 *
 * The linearity is pretty good for most sensors until you get to within
 * about 20mm. It is, however, almost always monotonic within 5mm or a target
 * so it is always possible to use the value to adjust, for example, forward
 * distance from a wall ahead.
 *
 */

enum
{
  STEER_NORMAL,
  STEER_LEFT_WALL,
  STEER_RIGHT_WALL,
  STEERING_OFF,
};

// used in the wait_for_user_start_function to indicate whih sensor was occluded
const uint8_t NO_START = 0;
const uint8_t LEFT_START = 1;
const uint8_t RIGHT_START = 2;

//***************************************************************************//
struct SensorChannel
{
  int raw;   // whatever the ADC gives us
  int value; // normalised to 100 at reference position
};

class Sensors;
extern Sensors sensors;

class Sensors
{
public:
  /*** wall sensor variables ***/

  volatile SensorChannel lfs;
  volatile SensorChannel lss;
  volatile SensorChannel rss;
  volatile SensorChannel rfs;

  volatile bool see_front_wall;
  volatile bool see_left_wall;
  volatile bool see_right_wall;

public:
  /*** steering variables ***/
  uint8_t g_steering_mode = STEER_NORMAL;
  // these are functions in case you want to modify them to
  // perform corrections or other scaling
  int get_front_sum()
  {
    return int(m_front_sum);
  };
  int get_front_diff()
  {
    return int(m_front_diff);
  };
  float get_steering_feedback()
  {
    return m_steering_adjustment;
  }
  float get_cross_track_error()
  {
    return m_cross_track_error;
  };

  void enable()
  {
    m_active = true;
  }

  void disable()
  {
    m_active = false;
  }

  //***************************************************************************//

  /**
   * The steering adjustment is an angular error that is added to the
   * current encoder angle so that the robot can be kept central in
   * a maze cell.
   *
   * A PD controller is used to generate the adjustment and the two constants
   * will need to be adjusted for the best response. You may find that only
   * the P term is needed
   *
   * The steering adjustment is limited to prevent over-correction. You should
   * experiment with that as well.
   *
   * @brief Calculate the steering adjustment from the cross-track error.
   * @param error calculated from wall sensors, Negative if too far right
   * @return steering adjustment in degrees
   *
   * TODO: It is not clear that this belongs here rather tham for example,
   *       in a Robot class.
   */
  float calculate_steering_adjustment()
  {
    // always calculate the adjustment for testing. It may not get used.
    float pTerm = STEERING_KP * m_cross_track_error;
    float dTerm = STEERING_KD * (m_cross_track_error - m_last_steering_error);
    float adjustment = pTerm + dTerm * LOOP_FREQUENCY;
    adjustment = clamp(adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);
    m_last_steering_error = m_cross_track_error;
    m_steering_adjustment = adjustment;
    return adjustment;
  }

  void set_steering_mode(uint8_t mode)
  {
    m_last_steering_error = m_cross_track_error;
    m_steering_adjustment = 0;
    g_steering_mode = mode;
  }

  /*********************************** Wall tracking **************************/
  // calculate the alignment errors - too far left is negative

  /***
   * Note: Runs from the systick interrupt. DO NOT call this directly.
   * @brief update the global wall sensor values.
   * @return robot cross-track-error. Too far left is negative.
   */
  void update()
  {
    readSensor();
    see_left_wall = is_wall(L);
    see_right_wall = is_wall(R);
    see_front_wall = is_wall(FL);

    // calculate the alignment errors - too far left is negative
    int error = 0;
    int right_error = SIDE_NOMINAL - rss.value;
    int left_error = SIDE_NOMINAL - lss.value;
    if (g_steering_mode == STEER_NORMAL)
    {
      if (is_wall(L) && is_wall(R))
        error = (dis_reading[L] - dis_reading[R]) / 2;

      else if (!is_wall(L) && is_wall(R))
        error = 5 - dis_reading[R];

      else if (is_wall(L) && !is_wall(R))
        error = dis_reading[L] - 5;
    }
    else if (g_steering_mode == STEER_LEFT_WALL)
    {
      error = 2 * (dis_reading[L] - 5);
    }
    else if (g_steering_mode == STEER_RIGHT_WALL)
    {
      error = -2 * (dis_reading[R] - 5);
    }

    if (dis_reading[FL] < FRONT_WALL_RELIABILITY_LIMIT / 10)
    {
      error = 0;
    }
    m_cross_track_error = error;
    calculate_steering_adjustment();
  }

  //***************************************************************************//

  // Convenience functions for the sensors when used as UI components
  // such as when starting the robot by putting your hand in front

  bool occluded_left()
  {
    return reading[L] > 1500 && reading[R] < 1000;
  }

  bool occluded_right()
  {
    return reading[L] < 1000 && reading[R] > 1500;
  }

  /**
   * The sensors will be enabled and this function will return
   * values indicating which of the two front sensors was occluded by the
   * user. While waiting, the user LED is flashing rapidly to show that it
   * expects some action. The LED will go out when the user is detected.
   *
   * There is no timeout - the method will wait forever if necessary.
   *
   * There is a 250ms delay before returning to let the user get clear.
   *
   * Leaves the sensors disabled.
   */
  uint8_t wait_for_user_start()
  {
    int state = 0;
    LED1_ON;
    uint8_t choice = NO_START;
    while (choice == NO_START)
    {
      int count = 0;
      while (occluded_left())
      {
        count++;
        delay_ms(20);
      }
      if (count > 5)
      {
        choice = LEFT_START;
        break;
      }
      count = 0;
      while (occluded_right())
      {
        count++;
        delay_ms(20);
      }
      if (count > 5)
      {
        choice = RIGHT_START;
        break;
      }
      LED2_ON;
      state = 1 - state;
      delay_ms(25);
    }

    LED1_OFF;
    delay_ms(250);
    return choice;
  }

private:
  float m_last_steering_error = 0;
  volatile bool m_active = false;
  volatile float m_cross_track_error = 0;
  volatile float m_steering_adjustment = 0;
  volatile int m_front_sum = 0;
  volatile int m_front_diff = 0;
};

#endif