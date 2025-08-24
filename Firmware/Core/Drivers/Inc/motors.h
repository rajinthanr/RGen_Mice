#ifndef MOTORS_H
#define MOTORS_H

#include "core.h"
#include "math.h"
#include "config.h"
#include "encoder.h"
#include "drive.h"
#include "main.h"
#include "delay.h"
#include "adc.h"
#include "led.h"
#include "sensor_Function.h"
#include "encoder.h"
#include "icm.h"
#include "drive.h"
#include "usart.h"
#include "flash.h"
#include "wall_handle.h"

#define RADIANS_PER_DEGREE 0.017453292519943295 // PI / 180

/***
 * The Motors class is provided with two main control signals - the forward
 * and rotary speeds. A third input come from the steering correction mechanism
 * used normally when the robot is tracking a wall or line. That input could also
 * come from a trajectory tracker or a target seeker.
 *
 * UKMARSBOT uses DC motors and, to get best performance with least user effort,
 * a combination of feedforward and feedbackcontrollers is used. While this can
 * seem complex, the user need not be aware of all the details so long as the
 * appropriate system characterisation is done to provide suitable values for the
 * various system constants.
 *
 * Under the hood, there are a pair of position controllers which have their set
 * points continuously updated by the desired speeds. Odometry provides feedback
 * for these controllers. greater reliability and precision would be possible if
 * the odometry had better resolution and if an IMU were available. But, you can
 * get remarkably good results with the limited resources available.
 */

class Motors;

extern Motors motors;

class Motors
{
public:
  /***
   * TODO: the constructor should really get at least the hardware pins
   * to do a safe setup.
   */
  void enable_controllers()
  {
    m_controller_output_enabled = true;
  }

  void disable_controllers()
  {
    m_controller_output_enabled = false;
  }

  void reset_controllers()
  {
    m_fwd_error = 0;
    m_rot_error = 0;
    m_previous_fwd_error = 0;
    m_previous_rot_error = 0;
  }

  void stop()
  {
    set_left_motor_volts(0);
    set_right_motor_volts(0);
  }

  void begin()
  {
    stop();
  }

  /**
   * At each iteration of the main control loop we need to calculate
   * now outputs form the two position controllers - one for forward
   * motion, one for rotation.
   *
   * The current error increases by an amount determined by the speed
   * and the control loop interval.
   *
   * It is then decreased by the amount the robot has actually moved
   * in the previous loop interval.
   *
   * These changes are both done in the first line.
   *
   * After that we have a simple PD contoller.
   *
   * NOTE: the D-term constant is premultiplied in the config by the
   * loop frequency to save a little time.
   */
  float position_controller()
  {
    float increment = m_velocity * LOOP_INTERVAL;
    m_fwd_error += increment - robot_fwd_change();
    float diff = m_fwd_error - m_previous_fwd_error;
    m_previous_fwd_error = m_fwd_error;
    float output = FWD_KP * m_fwd_error + FWD_KD * diff;
    return output;
  }

  /**
   * The rotation controller is exactly like the forward controller
   * except that there is an additional error term from the steering.
   * All steering correction is done by treating the error term as an
   * angular velocity. Depending on your method of analysis, you might
   * also try feeding the steering corection back as an angular
   * acceleration.
   *
   * If you have a gyro, you can use the output from that instead of
   * the encoders.
   *
   * A separate controller calculates the steering adjustment term.
   */
  float angle_controller(float steering_adjustment)
  {
    float increment = m_omega * LOOP_INTERVAL;
    m_rot_error += increment - robot_rot_change();
    m_rot_error += steering_adjustment;
    float diff = m_rot_error - m_previous_rot_error;
    m_previous_rot_error = m_rot_error;
    float output = ROT_KP * m_rot_error + ROT_KD * diff;
    return output;
  }

  /**
   * Feed forward attempts to work out what voltage the motors would need
   * to run at the current speed and acceleration.
   *
   * Without this, the controller has a lot of work to do and will be
   * much harder to tune for good performance.
   *
   * The drive train is not symmetric and there is significant stiction.
   * If used with PID, a simpler, single value will be sufficient.
   *
   */

  float leftFeedForward(float speed)
  {
    static float oldSpeed = speed;
    float leftFF = speed * SPEED_FF;
    if (speed > 0)
    {
      leftFF += BIAS_FF;
    }
    else if (speed < 0)
    {
      leftFF -= BIAS_FF;
    }
    else
    {
      // No bias when the speed is 0
    }
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = ACC_FF * acc;
    leftFF += accFF;
    return leftFF;
  }

  float rightFeedForward(float speed)
  {
    static float oldSpeed = speed;
    float rightFF = speed * SPEED_FF;
    if (speed > 0)
    {
      rightFF += BIAS_FF;
    }
    else if (speed < 0)
    {
      rightFF -= BIAS_FF;
    }
    else
    {
      // No bias when the speed is 0
    }
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = ACC_FF * acc;
    rightFF += accFF;
    return rightFF;
  }

  /**
   * Calculate the outputs of the feedback and feedforward controllers
   * for both forward and rotation, and combine them to obtain drive
   * voltages for the left and right motors.
   */
  void update_controllers(float velocity, float omega, float steering_adjustment)
  {
    m_velocity = velocity;
    m_omega = omega;
    float pos_output = position_controller();
    float rot_output = angle_controller(steering_adjustment);
    float left_output = 0;
    float right_output = 0;
    left_output = pos_output - rot_output;
    right_output = pos_output + rot_output;

    float tangent_speed = m_omega * MOUSE_RADIUS * RADIANS_PER_DEGREE;
    float left_speed = m_velocity - tangent_speed;
    float right_speed = m_velocity + tangent_speed;
    float left_ff = leftFeedForward(left_speed);
    float right_ff = rightFeedForward(right_speed);
    if (m_feedforward_enabled)
    {
      left_output += left_ff;
      right_output += right_ff;
    }
    if (m_controller_output_enabled)
    {
      set_right_motor_volts(right_output);
      set_left_motor_volts(left_output);
    }
  }

  /**
   * Once the motor voltages have been calculated, they need to be converted
   * into suitable PWM values for the motor drivers.
   *
   * In this section, the calculations for that are done, taking into account
   * the available battery voltage and the limits of the PWM hardware.
   *
   * If there is not enough voltage available from the battery, the output
   * will just saturate and the motor will not get up to speed.
   *
   * Some people add code to light up an LED whenever the drive output is
   * saturated.
   */
  int pwm_compensated(float desired_voltage, float battery_voltage)
  {
    int pwm = MOTOR_MAX_PWM * desired_voltage / battery_voltage;
    return pwm;
  }

  void set_left_motor_volts(float volts)
  {
    volts = clamp(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    m_left_motor_volts = volts;
    int motorPWM = pwm_compensated(volts, battery_voltage);
    left_motor_pwm(motorPWM);
  }

  void set_right_motor_volts(float volts)
  {
    volts = clamp(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    m_right_motor_volts = volts;
    int motorPWM = pwm_compensated(volts, battery_voltage);
    right_motor_pwm(motorPWM);
  }

  void set_speeds(float velocity, float omega)
  {
    m_velocity = velocity;
    m_omega = omega;
  }

private:
  bool m_controller_output_enabled = true;
  bool m_feedforward_enabled = true;
  float m_previous_fwd_error = 0;
  float m_previous_rot_error = 0;
  float m_fwd_error = 0;
  float m_rot_error = 0;
  float m_velocity = 0;
  float m_omega = 0;
  // these are maintained only for logging
  float m_left_motor_volts = 0;
  float m_right_motor_volts = 0;
};

#endif
