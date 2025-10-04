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
struct movement {
  Direction direction = AHEAD;
  uint8_t cell_count = 0;
  float speed = 0;
};

class Mouse {
private:
  Heading m_heading = NORTH;
  Location m_location = START;
  bool m_handStart = false;
  float m_angle = 0;
  uint8_t num_smooths = 0;
  uint8_t num_straights = 0;
  movement mpath[MAZE_CELL_COUNT];
  uint16_t num_moves = 0;

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
    num_smooths++;
    num_straights = 0;
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
    num_smooths++;
    num_straights = 0;
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
    if (maze.is_exit(m_location, m_heading))
      return;
    num_smooths = 0;

    if (trust_gyro) {
      // motion.spin_turn(-(angle - m_angle), OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
      return;
    }
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
    num_straights++;
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
          if (is_smooth_turn && (num_straights > 2 || num_smooths < 2))
            turn_smooth_right();
          else
            turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:
          if (is_smooth_turn && (num_straights > 2 || num_smooths < 2))
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
          if (is_smooth_turn && (num_straights > 2 || num_smooths < 2))
            turn_smooth_right();
          else
            turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:
          if (is_smooth_turn && (num_straights > 2 || num_smooths < 2))
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
    print("\n");
    print("Arrived!  \n");
    delay_ms(250);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
    return m_location == target;
  }
  /****************************************************************************/

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // Optimized fast run

  void plan(Location target) {
    clear();
    maze.load_from_flash();
    maze.flood(target);

    Location planLocation = m_location;
    Heading planHeading = maze.heading_to_smallest(planLocation, m_heading);

    uint16_t i = 0;

    while (planLocation != target) {
      Heading newHeading = maze.heading_to_smallest(planLocation, planHeading);
      if (newHeading == BLOCKED) {
        // we are stuck!
        break;
      }

      Direction movedir;
      unsigned char hdgChange =
          (newHeading + HEADING_COUNT - planHeading) % HEADING_COUNT;
      planHeading = newHeading;
      movedir = static_cast<Direction>(hdgChange);

      if (mpath[i].direction == movedir && movedir == AHEAD) {
        mpath[i].direction = AHEAD;
        mpath[i].cell_count++;
      } else {
        i += 1;
        mpath[i].direction = movedir;
        mpath[i].cell_count = 1;
      }

      planLocation = planLocation.neighbour(newHeading);
    }

    if (planLocation == target) {
      num_moves = i + 1;
    }
  }

  //------------------------------------------------------//
  // Clear previous plan
  //------------------------------------------------------//
  void clear() {
    num_moves = 0;
    for (int j = 0; j < MAZE_CELL_COUNT; j++) {
      mpath[j].direction = AHEAD;
      mpath[j].cell_count = 0;
    }
    // memset(mpath, 0, sizeof(mpath)); // optional
  }

  //------------------------------------------------------//
  // Print current planned path
  //------------------------------------------------------//
  void print_plan() {
    for (uint16_t i = 0; i < num_moves; i++) {
      switch (mpath[i].direction) {
      case AHEAD:
        print("S");
        break;
      case RIGHT:
        print("R");
        break;
      case LEFT:
        print("L");
        break;
      case BACK:
        print("B");
        break;
      default:
        print("?");
      }
      print("%d ", mpath[i].cell_count);
    }
    print("\n");
  }

  //------------------------------------------------------//
  // Execute fast run based on mpath[]
  //------------------------------------------------------//
  uint8_t start() {
    Heading newHeading = maze.heading_to_smallest(m_location, m_heading);
    if (newHeading == BLOCKED) {
      print("Stuck!\n");
      return 0;
    }

    turn_to_face(newHeading);
    delay_ms(200);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF); // never steer from zero speed

    motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED,
                      SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);

    print("Starting optimized run...\n");
    motion.wait_until_position(SENSING_POSITION);

    for (uint16_t i = 0; i < num_moves; i++) {
      switch (mpath[i].direction) {
      case AHEAD: {
        if (i == 0)
          mpath[i].cell_count -= 1;
        float s = mpath[i].cell_count * FULL_CELL / 2;
        float u = SEARCH_SPEED;
        float v = u * u + 2 * SEARCH_ACCELERATION * s;
        v = sqrt(v);
        v = clamp(v, SEARCH_SPEED, 800);
        motion.start_move(s * 2, v, u, SEARCH_ACCELERATION);
        motion.set_position(SENSING_POSITION);
        for (uint8_t j = 1; j <= mpath[i].cell_count; j++) {
          motion.wait_until_position(j * FULL_CELL + HALF_CELL);
          set_steering_mode(STEERING_OFF);
          motion.wait_until_position(j * FULL_CELL + SENSING_POSITION);
          set_steering_mode(STEER_NORMAL);
          m_location = m_location.neighbour(m_heading);
          log_action_status('-', ' ', m_location, m_heading);
          print("\n");
        }
      } break;
      case RIGHT:
        m_location = m_location.neighbour(m_heading);
        log_action_status('-', ' ', m_location, m_heading);
        print("\n");
        turn_right();
        break;
      case LEFT:
        m_location = m_location.neighbour(m_heading);
        log_action_status('-', ' ', m_location, m_heading);
        print("\n");
        turn_left();
        break;
      case BACK:
        m_location = m_location.neighbour(m_heading);
        log_action_status('-', ' ', m_location, m_heading);
        print("\n");
        turn_back();
        break;
      default:
        print("?");
      }
    }

    m_location = m_location.neighbour(m_heading);
    log_action_status('-', ' ', m_location, m_heading);
    print("\n");

    print("Stopping in center\n");
    stop_at_center();
    print("\n");
    print("Arrived!  \n");
    delay_ms(250);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
    return 1;
  }
};

#endif // MOUSE_H
