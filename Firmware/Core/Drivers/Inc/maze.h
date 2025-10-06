#ifndef MAZE_H
#define MAZE_H

#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define START Location(0, 0)
#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define MAX_COST (MAZE_CELL_COUNT - 1)

#define FLASH_MAZE_START 100

// ---- Wall states ----
enum WallState {
  EXIT = 0,
  WALL = 1,
  UNKNOWN = 2,
  VIRTUAL = 3,
};

struct WallInfo {
  WallState north : 2;
  WallState east : 2;
  WallState south : 2;
  WallState west : 2;
};

enum MazeMask {
  MASK_OPEN = 0x01,
  MASK_CLOSED = 0x03,
};

enum Heading { NORTH, EAST, SOUTH, WEST, HEADING_COUNT, BLOCKED = 99 };
enum Direction {
  AHEAD,
  RIGHT,
  BACK,
  LEFT,
  DIRECTION_COUNT,
  DIAGONAL_LEFT,
  DIAGONAL_RIGHT
};

class Location {
public:
  uint8_t x;
  uint8_t y;

  Location();
  Location(uint8_t ix, uint8_t iy);

  bool is_in_maze();
  bool operator==(const Location &obj) const;
  bool operator!=(const Location &obj) const;

  Location north() const;
  Location east() const;
  Location south() const;
  Location west() const;

  Location neighbour(const Heading heading) const;
};

Heading right_from(const Heading heading);
Heading left_from(const Heading heading);
Heading ahead_from(const Heading heading);
Heading behind_from(const Heading heading);

// ---- Maze class ----
class Maze {
public:
  uint8_t m_cost[MAZE_WIDTH][MAZE_HEIGHT];
  WallInfo m_walls[MAZE_WIDTH][MAZE_HEIGHT];

  Maze();

  Location goal() const;
  void set_goal(const Location goal);

  WallInfo walls(const Location cell) const;
  bool has_unknown_walls(const Location cell) const;
  int wall_count(const Location cell) const;
  bool cell_is_visited(const Location cell) const;
  bool is_exit(const Location cell, const Heading heading) const;

  void update_wall_state(const Location cell, const Heading heading,
                         const WallState state);
  void initialise();
  void set_mask(const MazeMask mask);
  MazeMask get_mask() const;

  uint16_t neighbour_cost(const Location cell, const Heading heading) const;
  uint16_t cost(const Location cell) const;

  void flood(const Location target);
  void fast_flood(const Location target);

  Heading heading_to_smallest(const Location cell,
                              const Heading start_heading) const;

  void save_to_flash();
  void load_from_flash();

private:
  void set_wall_state(const Location loc, const Heading heading,
                      const WallState state);

  MazeMask m_mask = MASK_OPEN;
  Location m_goal{7, 7};
};

extern Maze maze;

#ifdef __cplusplus
}
#endif

#endif // MAZE_H
