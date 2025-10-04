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
 *
 * In the same way, a ine follower and a sumo robot are also subclasses
 * of Robot.
 *
 * This code combines Robot with Mouse. If you are writing for several
 * different events, consider creating a Robot class for all the basic
 * functionality and then extending it for the individual contest events.
 */
class Mouse;
extern Mouse mouse;

void wall_adjustment();

class Mouse {
private:
  Heading m_heading = NORTH;
  Location m_location = START;
  bool m_handStart = false;

public:
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

  //   Mouse() {
  //     init();
  //   }

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
  /**
   * Used to bring the mouse to a halt, centered in a cell.
   *
   * If there is a wall ahead, it will use that for a reference to make sure it
   * is well positioned.
   *
   * On entry the robot is at some position that is an offset from the threshold
   * of the previous cell. Remember that all offset positions start at zero for
   * the cell threshold with 90mm (HALF_CELL) being the centre. The robot is
   * moving forwards. Since this is generally used in the search, the position
   * is likely to have some value between 170 and 190mm and the offset of centre
   * of the next cell is (FULL_CELL + HALF_CELL) = 270mm in the classic contest.
   *
   * TODO: the critical values are robot-dependent.
   *
   * TODO: need a function just to adjust forward position
   *
   * TODO: It would be better to use the distance rather than sensor readigs
   * here
   */
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
  /**
   * These convenience functions will bring the robot to a halt
   * before actually turning.
   *
   * Note that they do not change the robot's heading. That is
   * the responsibility of the caller.
   */

  void turn_IP180() {
    static int direction = 1;
    direction *= -1; // alternate direction each time it is called
    motion.spin_turn(direction * 180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  void turn_IP90R() { motion.spin_turn(-90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN); }

  void turn_IP90L() { motion.spin_turn(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN); }

  //***************************************************************************//
  /** Search turns
   *
   * These turns assume that the robot is crossing the cell boundary but is
   * still short of the start position of the turn.
   *
   * The turn will be a smooth, coordinated turn that should finish short of
   * the next cell boundary.
   *
   * Does NOT update the mouse heading but it possibly should
   *
   *
   * TODO: There is only just enough space to get down to turn speed. Increase
   * turn speed?
   *
   */

  void turn_smooth_left() {
    set_steering_mode(STEERING_OFF);
    if (!maze.is_exit(m_location, m_heading)) {
      // motion.move(2*(FULL_CELL-SENSING_POSITION)-20, SEARCH_SPEED,
      // SEARCH_SPEED, SEARCH_ACCELERATION);
      // float remaining =((dis_reading[FL] + dis_reading[FR]) / 2 -
      // SENSING_POSITION) -POLE_WIDTH / 2;
      // mouse.is_front_adjust = 1;
      // motion.move(remaining, SEARCH_SPEED, SEARCH_SPEED,
      // SEARCH_ACCELERATION); mouse.is_front_adjust = 0;
    } else {
      motion.move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                  SEARCH_SPEED, SEARCH_ACCELERATION);
    }
    LED4_OFF;

    set_steering_mode(STEERING_OFF);

    float perimeter = 0.25 * 2.0f * 3.14159f * (SENSING_POSITION - HALF_CELL);
    float omega = 90 / (perimeter / SEARCH_SPEED); // deg/s
    motion.start_turn(90.0f, omega, omega, 100000);
    motion.move(perimeter, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_omega(0);
    motion.start_turn(-1.0f, 0, 0, 100000);

    motion.start_move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                      SEARCH_SPEED, SEARCH_ACCELERATION);
    // turn_smooth(SS90EL);
    m_heading = left_from(m_heading);
    set_steering_mode(STEER_NORMAL);
  }

  void turn_smooth_right() {
    set_steering_mode(STEERING_OFF);
    if (!maze.is_exit(m_location, m_heading)) {
      // motion.move(2*(FULL_CELL-SENSING_POSITION)-20, SEARCH_SPEED,
      // SEARCH_SPEED, SEARCH_ACCELERATION);
      //      float remaining =
      //          ((dis_reading[FL] + dis_reading[FR]) / 2 - SENSING_POSITION) -
      //          POLE_WIDTH / 2;
      // mouse.is_front_adjust = 1;
      // motion.move(remaining, SEARCH_SPEED, SEARCH_SPEED,
      // SEARCH_ACCELERATION); mouse.is_front_adjust = 0;
    } else {
      motion.move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                  SEARCH_SPEED, SEARCH_ACCELERATION);
    }
    LED4_OFF;

    set_steering_mode(STEERING_OFF);

    float perimeter = 0.25 * 2.0f * 3.14159f * (SENSING_POSITION - HALF_CELL);
    float omega = 90 / (perimeter / SEARCH_SPEED); // deg/s
    motion.start_turn(-90.0f, omega, omega, 10000);
    motion.move(perimeter, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_omega(0);
    motion.start_turn(1.0f, 0, 0, 10000);

    motion.start_move(2 * (FULL_CELL - SENSING_POSITION), SEARCH_SPEED,
                      SEARCH_SPEED, SEARCH_ACCELERATION);
    // turn_smooth(SS90EL);
    m_heading = right_from(m_heading);
    set_steering_mode(STEER_NORMAL);
  }

  //***************************************************************************//
  /***
   * bring the mouse to a halt in the center of the current cell. That is,
   * the cell it is entering.
   *
   * On entry, we assume that the mouse knows its position in terms of the
   * distance from the start of the last cell.
   */
  void stop_at_center() {
    bool has_wall = is_wall(FL);
    set_steering_mode(STEERING_OFF);
    if (has_wall) {
      float remaining = (get_front_dis() - HALF_CELL) - POLE_WIDTH / 2;
      motion.move(remaining, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    }
    wall_adjustment();
    // float remaining = (FULL_CELL + HALF_CELL) - motion.position();
    // // finish at very low speed so we can adjust from the wall ahead if
    // present motion.start_move(remaining, motion.velocity(), 30,
    // motion.acceleration()); if (has_wall) {
    //   while (get_front_sum() < FRONT_REFERENCE) {
    //     delay_ms(2);
    //   }
    // } else {
    //   while (not motion.move_finished()) {
    //     delay_ms(2);
    //   };
    // }
    // // Be sure robot has come to a halt.
    motion.stop();
  }

  //***************************************************************************//
  /**
   * The robot is already moving so it is enough to let it carry on until
   * the next sensing position is reached.
   * Subtracting one full cell from the current position tricks the motion
   * control into thinking it is at (or just before) the start of a new cell.
   * Then it just waits until it gets to the next sensing position.
   */

  void wall_adjustment() {
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
  /***
   * As with all the search turns, this command will be called after the robot
   * has reached the search decision point and decided its next move. It is not
   * known how long that takes or what the exact position will be.
   *
   * Turning around is always going to be an in-place operation so it is
   * important that the robot is stationary and as well centred as possible.
   *
   * It only takes 27mm of travel to come to a halt from normal search speed.
   */
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

  /****************************************************************************/
  /***
   * This function moves the mouse from the start position to a target location
   * which is specified as a distance from the start position.
   *
   * Use this function to help with calibration of the encoders and wheel
   * diameter.
   *
   * You could also modify it to generate sensor data or control
   * teemetry while running.
   *
   * @param mm the distance from the start location to the target position
   */
  void run(int mm) {
    print("Follow TO");
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    maze.initialise();
    wait_for_user_start();
    enable();
    motion.reset_drive_system();
    set_steering_mode(STEER_NORMAL);
    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED,
                SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    motion.move(mm, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    print("\n");
    print("Arrived!  ");
    delay_ms(250);
    disable();
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
  }

  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  //***************************************************************************//
  //************************************************************************************************************************
  //*/
  /****************************************************************************/
  /***
   * search_to will cause the mouse to move to the given target cell
   * using safe, exploration speeds and turns.
   *
   * During the search, walls will be mapped but only when first seen.
   * A wall will not be changed once it has been mapped.
   *
   * It is possible for the mapping process to make the mouse think it
   * is walled in with no route to the target if wals are falsely
   * identified as present.
   *
   * On entry, the mouse will know its location and heading and
   * will begin by moving forward. The assumption is that the mouse
   * is already facing in an appropriate direction.
   *
   * All paths will start with a straight.
   *
   * If the function is called with handstart set true, you can
   * assume that the mouse is already backed up to the wall behind.
   *
   * Otherwise, the mouse is assumed to be centrally placed in a cell
   * and may be stationary or moving.
   *
   * The walls for the current location are assumed to be correct in
   * the map since mapping is always done by looking ahead into the
   * cell that is about to be entered.
   *
   * On exit, the mouse will be centered in the target cell still
   * facing in the direction it entered that cell. This will
   * always be one of the four cardinal directions NESW
   *
   */

  void search_to(Location target) {
    maze.flood(target);
    delay_ms(200);
    enable();
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
    disable();
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

  /****************************************************************************/
  /***
   * The mouse is expected to be in the start cell heading NORTH
   * The maze may, or may not, have been searched.
   * There may, or may not, be a solution.
   *
   * This simple searcher will just search to goal, turn around and
   * search back to the start. At that point there will be a route
   * but it is unlikely to be optimal.
   *
   * the mouse can run this route by creating a path that does not
   * pass through unvisited cells.
   *
   * A better searcher will continue until a path generated through all
   * cells, regardless of visited state, does not pass through any
   * unvisited cells.
   *
   * The return value is not currently used but could indicate whether
   * the maze is 'solved'. That is, whether there is any need to search
   * further.
   *
   */
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
  //************  BELOW HERE ARE VARIOUS TEST FUNCTIONS
  //***********************//
  //********** THEY ARE NOT ESSENTIAL TO THE BUSINESS OF
  //**********************//
  //******** SOLVING THE MAZE BUT THEY MAY HELP WITH SETUP
  //********************//
  //***************************************************************************//

  /***
   * Visual feedback by flashing the LED indicators
   */
  void blink(int count) {
    for (int i = 0; i < count; i++) {
      LED1_ON;
      delay_ms(100);
      LED1_OFF;
      delay_ms(100);
    }
  }

  /***
   * just sit in a loop, flashing lights waiting for the button to be pressed
   */
  void panic() {
    while (!switches.key_pressed()) {
      blink(1);
    }
    switches.wait_for_key_release();
    LED1_OFF;
  }

  /***
   * You may want to log the front sensor readings as a function of distance
   * from the wall. This function does that. Place the robot hard up against
   * a wall ahead and run the command. You will get a table of values for
   * the sensors as a function of distance.
   *
   */
  void conf_log_front_sensor() {
    enable();
    motion.reset_drive_system();
    // reporter.front_sensor_track_header();
    motion.start_move(-200, 100, 0, 500);
    while (not motion.move_finished()) {
      // reporter.front_sensor_track();
    }
    motion.reset_drive_system();
    motion.disable_drive();
    set_steering_mode(STEERING_OFF);
    disable();
  }

  /**
   * By turning in place through 360 degrees, it should be possible to get a
   * sensor calibration for all sensors?
   *
   * At the least, it will tell you about the range of values reported and help
   * with alignment, You should be able to see clear maxima 180 degrees apart as
   * well as the left and right values crossing when the robot is parallel to
   * walls either side.
   *
   * Use either the normal report_sensor_track() for the normalised readings
   * or report_sensor_track_raw() for the readings straight off the sensor.
   *
   * Sensor sensitivity should be set so that the peaks from raw readings do
   * not exceed about 700-800 so that there is enough headroom to cope with
   * high ambient light levels.
   *
   * @brief turn in place while streaming sensors
   */

  void conf_sensor_spin_calibrate() {
    // int side = wait_for_user_start(); // cover front sensor w

    enable();
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
    // reporter.report_sensor_track_header();
    motion.start_turn(360, 180, 0, 1800);
    while (not motion.turn_finished()) {
      // reporter.report_radial_track(use_raw);
      // reporter.print_wall_sensors();
    }
    // reporter.report_radial_track(use_raw);
    motion.reset_drive_system();
    motion.disable_drive();
    delay_ms(100);
  }

  /**
   * TODO: move this out to the mazerunner-setup code
   *
   * Edge detection test displays the position at which an edge is found when
   * the robot is travelling down a straight.
   *
   * Start with the robot backed up to a wall.
   * Runs forward for 150mm and records the robot position when the trailing
   * edge of the adjacent wall(s) is found.
   *
   * The value is only recorded to the nearest millimeter to avoid any
   * suggestion of better accuracy than that being available.
   *
   * Note that UKMARSBOT, with its back to a wall, has its wheels 43mm from
   * the cell boundary.
   *
   * This value can be used to permit forward error correction of the robot
   * position while exploring.
   *
   * @brief find sensor wall edge detection positions
   */

  void conf_edge_detection() {
    bool left_edge_found = false;
    bool right_edge_found = false;
    int left_edge_position = 0;
    int right_edge_position = 0;

    wait_for_user_start(); // cover front sensor with hand to start
    enable();
    delay_ms(100);
    motion.reset_drive_system();
    set_steering_mode(STEER_NORMAL);
    print("Edge positions:\n");
    motion.start_move(FULL_CELL * 4, 500, 0, 1000);
    while (not motion.move_finished()) {
      // if (sensors.lss.value > left_max) {
      //   left_max = sensors.lss.value;
      // }

      // if (sensors.rss.value > right_max) {
      //   right_max = sensors.rss.value;
      // }

      // if (not left_edge_found) {
      //   if (sensors.lss.value < left_max / 2) {
      //     left_edge_position = int(0.5 + motion.position());
      //     left_edge_found = true;
      //   }
      // }
      // if (not right_edge_found) {
      //   if (sensors.rss.value < right_max / 2) {
      //     right_edge_position = int(0.5 + motion.position());
      //     right_edge_found = true;
      //   }
      // }
      // delay_ms(5);
    }
    // print("Robot distance: %d\n", encoders.robot_distance());
    print("Left: ");
    if (left_edge_found) {
      print("%d\n", BACK_WALL_TO_CENTER + left_edge_position);
    } else {
      print("-\n");
    }

    print("  Right: ");
    if (right_edge_found) {
      print("%d\n", BACK_WALL_TO_CENTER + right_edge_position);
    } else {
      print("-\n");
    }
    print("\n");
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
    disable();
    delay_ms(100);
  }

  /***
   * A basic function to let you test the configuration of the SS90Ex turns.
   *
   * These are the turns used during the search of the maze and need to be
   * accurate and repeatable.
   *
   * You may need to spend some time with this function to get the turns
   * just right.
   *
   * NOTE: that the turn parameters are stored in the robot config file
   * NOTE: that the left and right turns are likely to be different.
   *
   */
  void test_SS90E() {
    // note that changes to the speeds are likely to affect
    // the other turn parameters
    uint8_t side = wait_for_user_start();
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
    // move to the boundary with the next cell
    float distance = BACK_WALL_TO_CENTER + HALF_CELL;
    motion.move(distance, SEARCH_TURN_SPEED, SEARCH_TURN_SPEED,
                SEARCH_ACCELERATION);
    motion.set_position(FULL_CELL);

    if (side == RIGHT_START) {
      // turn_smooth(SS90ER);
    } else {
      // turn_smooth(SS90EL);
    }
    // after the turn, estimate the angle error by looking for
    // changes in the side sensor readings
    int sensor_left = dis_reading[L];
    int sensor_right = dis_reading[R];
    // move two cells. The resting position of the mouse have the
    // same offset as the turn ending
    motion.move(2 * FULL_CELL, SEARCH_TURN_SPEED, 0, SEARCH_ACCELERATION);
    sensor_left -= is_wall(L);
    ;
    sensor_right -= is_wall(R);
    // reporter.print_justified(sensor_left, 5);
    // reporter.print_justified(sensor_right, 5);
    motion.reset_drive_system();
    set_steering_mode(STEERING_OFF);
  }

  /***
   * loop until the user button is pressed while
   * pumping out sensor readings. The first four numbers are
   * the raw readings, the next four are normalised then there
   * are two values for the sum and difference of the front sensors
   *
   * The advanced user might use this as a start for auto calibration
   */
  void show_sensor_calibration() {
    // reporter.wall_sensor_header();
    enable();
    while (not switches.key_pressed()) {
      // reporter.print_wall_sensors();
    }
    switches.wait_for_key_release();
    print("\n");
    delay_ms(200);
    disable();
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
    enable();
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
    disable();
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
