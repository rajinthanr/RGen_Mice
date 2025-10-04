#ifndef FAST_RUN_H
#define FAST_RUN_H

#ifdef __cplusplus

#include "core.h"
#include "maze.h"
#include "motion.h"
#include "mouse.h"
#include "queue.h"
#include "usart.h"

class Mouse;
class Motion;
class Maze;

extern Motion motion;
extern Mouse mouse;
extern Maze maze;

extern "C" {
#endif

//------------------------------------------------------//
// FastRun class
//------------------------------------------------------//

struct movement {
  Direction direction = AHEAD;
  uint8_t cell_count = 0;
  float speed = 0;
};

class FastRun {
public:
  uint16_t num_moves = 0;
  movement mpath[MAZE_CELL_COUNT];
  Location m_location = START;
  Location m_target = START;
  Heading m_heading = NORTH;

  void plan(Location start, Heading start_heading, Location target, Maze &maze);
  void clear();
  void print_plan();
  uint8_t start();
  void stop();
};

#ifdef __cplusplus
}
#endif

#endif // FAST_RUN_H
