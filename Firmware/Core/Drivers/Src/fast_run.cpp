#include "fast_run.h"

//------------------------------------------------------//
// Plan path for fast run
//------------------------------------------------------//
void FastRun::plan(Location start, Heading start_heading, Location target,
                   Maze &maze) {
  clear();
  maze.load_from_flash();
  maze.flood(target);

  m_location = start;
  m_target = target;
  m_heading = start_heading;

  Location current = start;
  Heading heading = start_heading;
  uint16_t i = 0;

  heading = maze.heading_to_smallest(current, heading);

  while (current != target) {
    Heading newHeading = maze.heading_to_smallest(current, heading);
    if (newHeading == BLOCKED) {
      // we are stuck!
      break;
    }

    Direction movedir;
    unsigned char hdgChange =
        (newHeading + HEADING_COUNT - heading) % HEADING_COUNT;
    heading = newHeading;
    movedir = static_cast<Direction>(hdgChange);

    if (mpath[i].direction == movedir && movedir == AHEAD) {
      mpath[i].direction = AHEAD;
      mpath[i].cell_count++;
    } else {
      i += 1;
      mpath[i].direction = movedir;
      mpath[i].cell_count = 1;
    }

    current = current.neighbour(newHeading);
  }

  if (current == target) {
    num_moves = i + 1;
  }
}

//------------------------------------------------------//
// Clear previous plan
//------------------------------------------------------//
void FastRun::clear() {
  num_moves = 0;
  // memset(mpath, 0, sizeof(mpath)); // optional
}

//------------------------------------------------------//
// Print current planned path
//------------------------------------------------------//
void FastRun::print_plan() {
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
uint8_t FastRun::start() {
  Heading newHeading = maze.heading_to_smallest(m_location, m_heading);
  if (newHeading == BLOCKED) {
    print("Stuck!\n");
    print("e %d\n", maze.heading_to_smallest(m_location, m_heading));
    return 0;
  }

  mouse.turn_to_face(newHeading);
  delay_ms(200);
  motion.reset_drive_system();
  set_steering_mode(STEERING_OFF); // never steer from zero speed

  motion.start_move(FULL_CELL, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
  motion.set_position(HALF_CELL);

  print("Starting optimized run...\n");
  motion.wait_until_position(SENSING_POSITION);

  for (uint16_t i = 1; i < num_moves; i++) {
    switch (mpath[i].direction) {
    case AHEAD: {
      float s = mpath[i].cell_count * FULL_CELL / 2;
      float u = SEARCH_SPEED;
      float v = u * u + 2 * SEARCH_ACCELERATION * s;
      v = sqrt(v);
      motion.start_move(s * 2, v, u, SEARCH_ACCELERATION);
      motion.set_position(SENSING_POSITION);
      for (uint8_t j = 1; j <= mpath[i].cell_count; j++) {
        motion.wait_until_position(j * FULL_CELL + HALF_CELL);
        set_steering_mode(STEERING_OFF);
        motion.wait_until_position(j * FULL_CELL + SENSING_POSITION);
        set_steering_mode(STEER_NORMAL);
      }
    } break;
    case RIGHT:
      mouse.turn_right();
      break;
    case LEFT:
      mouse.turn_left();
      break;
    case BACK:
      mouse.turn_back();
      break;
    default:
      print("?");
    }
  }

  print("Stopping in center\n");
  mouse.stop_at_center();
  print("\n");
  print("Arrived!  \n");
  delay_ms(250);
  motion.reset_drive_system();
  set_steering_mode(STEERING_OFF);
  return 1;
}

//------------------------------------------------------//
// Stop (placeholder for now)
//------------------------------------------------------//
void FastRun::stop() { motion.stop(); }
