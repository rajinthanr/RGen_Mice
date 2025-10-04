#ifndef MOUSE_H
#define MOUSE_H

#include "button.h"
#include "config.h"
#include "core.h"
#include "delay.h"
#include "led.h"
#include "maze.h"
#include "motion.h"
#include "sensor.h"
#include "usart.h"
#include "wall_handle.h"

/***
 * The Mouse class is really a subclass of a more generic robot. It should
 * add functionality to a robot class by providing specific mapping and
 * planning features that are more closely aligned with its purpose.
 */
class Mouse;
extern Mouse mouse;

void wall_adjustment();

class Mouse {
private:
  Heading m_heading = NORTH;
  Location m_location = START;
  bool m_handStart = false;
  float m_angle = 0;

public:
  uint8_t trust_gyro = 0;
  float speed_adj;
  float steering_adj;
  float target_dis;
  float target_angle;
  uint8_t is_smooth_turn = 0;

  float linear_speed = 0.0f;
  float angular_speed = 0.0f;
  float target_linear_speed = 0.0f;
  float target_angular_speed = 0.0f;
  float max_linear_speed = 100.0f;  // Example: 800 mm/s
  float max_angular_speed = 360.0f; // Example: 360 deg/s
  float max_linear_accel = 600.0f;  // Example: 2000 mm/s^2
  float max_angular_accel = 360.0f; // Example: 1800 deg/s^2
  uint8_t steering_mode = STEERING_OFF;
  uint8_t is_front_adjust = 0;
  float front_adjustment = 0;
  float wall_error = 0;
  float steering_adjustment = 0;
  float last_steering_error = 0;

  Mouse() { init(); }

  enum State { FRESH_START, SEARCHING, INPLACE_RUN, SMOOTH_RUN, FINISHED };

  enum TurnType {
    SS90EL = 0,
    SS90ER = 1,
    SS90L = 2,
    SS90R = 3,
  };

  void init() {
    m_handStart = false;
    set_steering_mode(STEERING_OFF);
    m_location = Location(0, 0);
    m_heading = NORTH;
  }

  //***************************************************************************//
  /**
   * change the mouse heading but do not physically turn
   */

  void set_heading(Heading new_heading) { m_heading = new_heading; }

  static void stopAndAdjust() {
    float remaining = (FULL_CELL + HALF_CELL) - motion.position();
    set_steering_mode(STEERING_OFF);
    // Keep moving with the intent of stopping at the cell centre
    motion.start_move(remaining, motion.velocity(), 0, motion.acceleration());
    // While waiting, check to see if a front wall becomes visible and break
    // out early if it does
    while (not motion.move_finished()) {
      if (is_wall(FL)) {
        break;
      }
      delay_ms(2);
    }
    // If the wait finished early because of a wall ahead then use that
    // wall to creep up on the cell centre
    if (is_wall(FL)) {
      while (is_wall(FL)) {
        motion.start_move(10, 50, 0, 1000);
        delay_ms(2);
      }
    }
  }

  //***************************************************************************//

  void turn_IP180() {
    static int direction = 1;
    direction *= -1; // alternate direction each time it is called
    if (trust_gyro)
      motion.spin_turn(direction * 180 - (angle - m_angle), OMEGA_SPIN_TURN,
                       ALPHA_SPIN_TURN);
    else
      motion.spin_turn(direction * 180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    m_angle += 180 * direction;
  }

  void turn_IP90R() {
    if (trust_gyro)
      motion.spin_turn(-90 - (angle - m_angle), OMEGA_SPIN_TURN,
                       ALPHA_SPIN_TURN);
    else
      motion.spin_turn(-90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    m_angle -= 90;
  }

  void turn_IP90L() {
    if (trust_gyro)
      motion.spin_turn(90 - (angle - m_angle), OMEGA_SPIN_TURN,
                       ALPHA_SPIN_TURN);
    else
      motion.spin_turn(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    m_angle += 90;
  }

  void turn_smooth_left() {
    set_steering_mode(STEERING_OFF);
    if (!maze.is_exit(m_location, m_heading)) {

    } else {
      motion.move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                  SEARCH_SPEED, SEARCH_ACCELERATION);
    }
    LED4_OFF;

    set_steering_mode(STEERING_OFF);

    float perimeter = 0.25 * 2.0f * 3.14159f * (SENSING_POSITION - HALF_CELL);
    float omega = 90 / (perimeter / SEARCH_SPEED); // deg/s
    if (trust_gyro)
      motion.start_turn(90.0f - (angle - m_angle), omega, 0, 10000);
    else
      motion.start_turn(90.0f, omega, 0, 10000);
    motion.move(perimeter, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_omega(0);

    motion.start_move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                      SEARCH_SPEED, SEARCH_ACCELERATION);
    // turn_smooth(SS90EL);
    m_heading = left_from(m_heading);
    set_steering_mode(STEER_NORMAL);
    m_angle += 90;
  }

  void turn_smooth_right() {
    set_steering_mode(STEERING_OFF);
    if (!maze.is_exit(m_location, m_heading)) {

    } else {
      motion.move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                  SEARCH_SPEED, SEARCH_ACCELERATION);
    }
    LED4_OFF;

    set_steering_mode(STEERING_OFF);

    float perimeter = 0.25 * 2.0f * 3.14159f * (SENSING_POSITION - HALF_CELL);
    float omega = 90 / (perimeter / SEARCH_SPEED); // deg/s

    if (trust_gyro)
      motion.start_turn(-90.0f - (angle - m_angle), omega, 0, 10000);
    else
      motion.start_turn(-90.0f, omega, 0, 10000);
    motion.move(perimeter, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_omega(0);

    motion.start_move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                      SEARCH_SPEED, SEARCH_ACCELERATION);
    // turn_smooth(SS90EL);
    m_heading = right_from(m_heading);
    set_steering_mode(STEER_NORMAL);
    m_angle -= 90;
  }

  void stop_at_center() {
    bool has_wall = is_wall(FL);
    set_steering_mode(STEERING_OFF);
    if (has_wall) {
      float remaining = (get_front_dis() - HALF_CELL) - POLE_WIDTH / 2;
      motion.move(remaining, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    }
    wall_adjustment();
    motion.stop();
  }

  void wall_adjustment() {
    if (trust_gyro) {
      // motion.spin_turn(-(angle - m_angle), OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
      return;
    }
    if (maze.is_exit(m_location, m_heading))
      return;
    mouse.is_front_adjust = 1;
    mouse.wall_error = wallFront();
    uint32_t start = millis();
    while (mouse.wall_error > 1) {
      if (millis() - start > 1000)
        break;
      delay_ms(1);
    }
    mouse.is_front_adjust = 0;
  }

  //******************************************************************************//

  void move_ahead() {
    is_left_wall = !maze.is_exit(m_location, left_from(m_heading));
    is_right_wall = !maze.is_exit(m_location, right_from(m_heading));
    set_steering_mode(STEER_NORMAL);
    motion.set_position(SENSING_POSITION - FULL_CELL);
    motion.wait_until_position(HALF_CELL);

    // LED4_OFF;
    set_steering_mode(STEERING_OFF);
    motion.wait_until_position(SENSING_POSITION);
  }

  //***************************************************************************//
  void turn_left() {
    set_steering_mode(STEER_NORMAL);
    if (!maze.is_exit(m_location, m_heading)) {
      motion.move(FULL_CELL + HALF_CELL / 2 - SENSING_POSITION - 20,
                  SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
      float remaining = ((dis_reading[FL] + dis_reading[FR]) / 2 - HALF_CELL) -
                        POLE_WIDTH / 2;
      motion.move(remaining, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    } else {
      motion.move(FULL_CELL * 1.5 - SENSING_POSITION, SEARCH_SPEED, 0,
                  SEARCH_ACCELERATION);
    }
    // LED4_OFF;

    set_steering_mode(STEERING_OFF);
    wall_adjustment();
    turn_IP90L();
    motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED,
                      SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    motion.wait_until_position(SENSING_POSITION);
    // turn_smooth(SS90EL);
    m_heading = left_from(m_heading);
  }

  //***************************************************************************//
  void turn_right() {
    set_steering_mode(STEER_NORMAL);
    if (!maze.is_exit(m_location, m_heading)) {
      motion.move(FULL_CELL + HALF_CELL / 2 - SENSING_POSITION - 20,
                  SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
      float remaining = ((dis_reading[FL] + dis_reading[FR]) / 2 - HALF_CELL) -
                        POLE_WIDTH / 2;
      motion.move(remaining, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    } else {
      motion.move(FULL_CELL * 1.5 - SENSING_POSITION, SEARCH_SPEED, 0,
                  SEARCH_ACCELERATION);
    }
    // LED4_OFF;
    set_steering_mode(STEERING_OFF);
    wall_adjustment();
    turn_IP90R();
    motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED,
                      SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    motion.wait_until_position(SENSING_POSITION);
    // turn_smooth(SS90ER);
    m_heading = right_from(m_heading);
  }

  //***************************************************************************//

  void turn_back() {
    set_steering_mode(STEER_NORMAL);
    if (!maze.is_exit(m_location, m_heading)) {
      motion.move(FULL_CELL + HALF_CELL / 2 - SENSING_POSITION - 20,
                  SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
      float remaining = ((dis_reading[FL] + dis_reading[FR]) / 2 - HALF_CELL) -
                        POLE_WIDTH / 2;
      motion.move(remaining, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    } else {
      motion.move(FULL_CELL * 1.5 - SENSING_POSITION, SEARCH_SPEED, 0,
                  SEARCH_ACCELERATION);
    }
    // LED4_OFF;
    set_steering_mode(STEERING_OFF);
    wall_adjustment();
    turn_IP180();
    motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED,
                      SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    motion.wait_until_position(SENSING_POSITION);
    // turn_smooth(SS90ER);
    m_heading = behind_from(m_heading);
  }

  void run(int mm) {
    print("Follow TO");
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    maze.initialise();
    wait_for_user_start();
    motion.reset_drive_system();
    set_steering_mode(STEER_NORMAL);
    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED,
                SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    motion.move(mm, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    print("\n");
    print("Arrived!  ");
    delay_ms(250);

    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
  }

  void search_to(Location target) {
    maze.flood(target);
    delay_ms(200);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF); // never steer from zero speed
    if (not m_handStart) {
      // back up to the wall behind
      // TODO: what if there is not a wall?
      // perhaps the caller should decide so this ALWAYS starts at the cell
      // centre?
      set_steering_mode(GYRO_OFF);
      motion.move(-BACK_WALL_TO_CENTER, SEARCH_SPEED / 4, 0,
                  SEARCH_ACCELERATION / 2);
      set_steering_mode(STEERING_OFF);
    }
    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    LED1_TOGGLE;
    delay_ms(1000);
    LED1_TOGGLE;
    motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED,
                      SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    print("Off we go...");
    print("[");
    print("%d", target.x);
    print(",");
    print("%d", target.y);
    print("]");
    print("\n");

    motion.wait_until_position(SENSING_POSITION);
    // Each iteration of this loop starts at the sensing point
    while (m_location != target) {
      if (switches.key_pressed()) { // allow user to abort gracefully
        break;
      }
      // print("e\n");
      log_action_status('-', ' ', m_location, m_heading);
      set_steering_mode(STEER_NORMAL);
      m_location =
          m_location.neighbour(m_heading); // the cell we are about to enter
      update_map();
      maze.flood(target);
      unsigned char newHeading =
          maze.heading_to_smallest(m_location, m_heading);
      if (newHeading == BLOCKED) {
        // we are stuck - no way out
        print("Stuck!\n");
        print("e %d\n", maze.heading_to_smallest(m_location, m_heading));
        motion.move(FULL_CELL * 1.5 - SENSING_POSITION, SEARCH_SPEED, 0,
                    SEARCH_ACCELERATION);
        break;
      }
      unsigned char hdgChange = (newHeading - m_heading) & 0x3;
      if (m_location != target) {
        print("%d \n", hdgChange);
        LED4_ON;
        delay_us(50);
        LED4_OFF;
        switch (hdgChange) {
        // each of the following actions will finish with the
        // robot moving and at the sensing point ready for the
        // next loop iteration
        case AHEAD:
          move_ahead();
          break;
        case RIGHT:
          turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:
          turn_left();
          break;
        }
      }
    }
    // we are entering the target cell so come to an orderly
    // halt in the middle of that cell
    stop_at_center();
    ;
    print("\n");
    print("Arrived!  ");
    delay_ms(250);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
  }

  void run_to(Location target) {
    (void)target;
    //// Not implemented
  }

  /****************************************************************************/
  void turn_to_face(Heading newHeading) {
    unsigned char hdgChange =
        (newHeading + HEADING_COUNT - m_heading) % HEADING_COUNT;
    switch (hdgChange) {
    case AHEAD:
      break;
    case RIGHT:
      turn_IP90R();
      break;
    case BACK:
      turn_IP180();
      break;
    case LEFT:
      turn_IP90L();
      break;
    }
    m_heading = newHeading;
  }

  /****************************************************************************/
  void update_map() {
    bool leftWall = is_wall(L);
    bool frontWall = is_wall(FL);
    bool rightWall = is_wall(R);
    char w[] = "--- ";
    if (leftWall) {
      w[0] = 'L';
    }
    if (frontWall) {
      w[1] = 'F';
    }
    if (rightWall) {
      w[2] = 'R';
    }
    print("%s  ", w);

    switch (m_heading) {
    case NORTH:
      maze.update_wall_state(m_location, NORTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, leftWall ? WALL : EXIT);
      break;
    case EAST:
      maze.update_wall_state(m_location, EAST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, leftWall ? WALL : EXIT);
      break;
    case SOUTH:
      maze.update_wall_state(m_location, SOUTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, leftWall ? WALL : EXIT);
      break;
    case WEST:
      maze.update_wall_state(m_location, WEST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, leftWall ? WALL : EXIT);
      break;
    default:
      // This is an error. We should handle it.
      break;
    }
  }

  int search_maze() {
    // wait_for_user_start();
    print("Search TO\n");
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    search_to(maze.goal());
    maze.flood(START);

    Heading best_direction = maze.heading_to_smallest(m_location, m_heading);
    turn_to_face(best_direction);
    m_handStart = false;
    search_to(START);
    turn_to_face(NORTH);
    motion.stop();
    motion.disable_drive();
    return 0;
  }

  //***************************************************************************//

  uint8_t search(Location target) {
    maze.flood(target);
    Heading newHeading = maze.heading_to_smallest(m_location, m_heading);
    if (newHeading == BLOCKED) {
      print("Stuck!\n");
      print("e %d\n", maze.heading_to_smallest(m_location, m_heading));
      return 0;
    }
    turn_to_face(newHeading);
    delay_ms(200);

    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF); // never steer from zero speed

    motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED,
                      SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    print("Off we go...");
    print("[");
    print("%d", target.x);
    print(",");
    print("%d", target.y);
    print("]");
    print("\n");

    motion.wait_until_position(SENSING_POSITION);
    // Each iteration of this loop starts at the sensing point
    while (m_location != target) {
      if (switches.key_pressed()) { // allow user to abort gracefully
        break;
      }
      // print("e\n");
      log_action_status('-', ' ', m_location, m_heading);
      set_steering_mode(STEER_NORMAL);
      m_location =
          m_location.neighbour(m_heading); // the cell we are about to enter

      if (m_location == target)
        break;
      update_map();
      maze.flood(target);
      unsigned char newHeading =
          maze.heading_to_smallest(m_location, m_heading);
      if (newHeading == BLOCKED) {
        // we are stuck - no way out
        print("Stuck!\n");
        print("e %d\n", maze.heading_to_smallest(m_location, m_heading));
        motion.move(FULL_CELL * 1.5 - SENSING_POSITION, SEARCH_SPEED, 0,
                    SEARCH_ACCELERATION);
        break;
      }
      unsigned char hdgChange = (newHeading - m_heading) & 0x3;
      if (m_location != target) {
        print("%d \n", hdgChange);
        // LED4_ON;
        switch (hdgChange) {
        // each of the following actions will finish with the
        // robot moving and at the sensing point ready for the
        // next loop iteration
        case AHEAD:
          move_ahead();
          break;
        case RIGHT:
          if (is_smooth_turn)
            turn_smooth_right();
          else
            turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:
          if (is_smooth_turn)
            turn_smooth_left();
          else
            turn_left();
          break;
        }
      }
    }
    // we are entering the target cell so come to an orderly
    // halt in the middle of that cell
    print("Stopping in center\n");
    stop_at_center();
    ;
    print("\n");
    print("Arrived!  \n");
    delay_ms(250);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
    return m_location == target;
  }
  /****************************************************************************/

  uint8_t fast_run(Location target) {
    maze.flood(target);
    Heading newHeading = maze.heading_to_smallest(m_location, m_heading);
    if (newHeading == BLOCKED) {
      print("Stuck!\n");
      print("e %d\n", maze.heading_to_smallest(m_location, m_heading));
      return 0;
    }
    turn_to_face(newHeading);
    delay_ms(200);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF); // never steer from zero speed

    motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED,
                      SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    print("Off we go...");
    print("[");
    print("%d", target.x);
    print(",");
    print("%d", target.y);
    print("]");
    print("\n");

    motion.wait_until_position(SENSING_POSITION);

    // Each iteration of this loop starts at the sensing point
    while (m_location != target) {
      log_action_status('-', ' ', m_location, m_heading);

      set_steering_mode(STEER_NORMAL);
      m_location =
          m_location.neighbour(m_heading); // the cell we are about to enter

      if (m_location == target)
        break;

      unsigned char newHeading =
          maze.heading_to_smallest(m_location, m_heading);

      if (newHeading == BLOCKED) {
        print("Stuck!\n");
        print("e %d\n", maze.heading_to_smallest(m_location, m_heading));
        motion.move(FULL_CELL * 1.5 - SENSING_POSITION, SEARCH_SPEED, 0,
                    SEARCH_ACCELERATION);
        break;
      }

      unsigned char hdgChange = (newHeading - m_heading) & 0x3;
      if (m_location != target) {
        print("%d \n", hdgChange);

        switch (hdgChange) {
        case AHEAD:
          move_ahead();
          break;
        case RIGHT:
          if (is_smooth_turn)
            turn_smooth_right();
          else
            turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:
          if (is_smooth_turn)
            turn_smooth_left();
          else
            turn_left();
          break;
        }
      }
    }

    // we are entering the target cell so come to an orderly halt in the middle
    // of that cell
    print("Stopping in center\n");
    stop_at_center();
    ;
    print("\n");
    print("Arrived!  \n");
    delay_ms(250);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
    return m_location == target;
  }
  /****************************************************************************/
};

#endif // MOUSE_H
