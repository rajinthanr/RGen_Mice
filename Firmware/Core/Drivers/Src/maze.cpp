#include "maze.h"
#include <stdio.h>

// ---- Location implementation ----
Location::Location() : x(0), y(0) {}
Location::Location(uint8_t ix, uint8_t iy) : x(ix), y(iy) {}

bool Location::is_in_maze() {
    return x < MAZE_WIDTH && y < MAZE_HEIGHT;
}

bool Location::operator==(const Location &obj) const {
    return x == obj.x && y == obj.y;
}

bool Location::operator!=(const Location &obj) const {
    return x != obj.x || y != obj.y;
}

Location Location::north() const { return Location(x, (y + 1) % MAZE_HEIGHT); }
Location Location::east() const  { return Location((x + 1) % MAZE_WIDTH, y); }
Location Location::south() const { return Location(x, (y + MAZE_HEIGHT - 1) % MAZE_HEIGHT); }
Location Location::west() const  { return Location((x + MAZE_WIDTH - 1) % MAZE_WIDTH, y); }

Location Location::neighbour(const Heading heading) const {
    switch (heading) {
        case NORTH: return north();
        case EAST:  return east();
        case SOUTH: return south();
        case WEST:  return west();
        default:    return *this;
    }
}

// ---- Heading helpers ----
Heading right_from(const Heading heading) {
    return static_cast<Heading>((heading + 1) % HEADING_COUNT);
}
Heading left_from(const Heading heading) {
    return static_cast<Heading>((heading + HEADING_COUNT - 1) % HEADING_COUNT);
}
Heading ahead_from(const Heading heading) {
    return heading;
}
Heading behind_from(const Heading heading) {
    return static_cast<Heading>((heading + 2) % HEADING_COUNT);
}

// ---- Maze implementation ----
Maze::Maze() {}

Location Maze::goal() const { return m_goal; }
void Maze::set_goal(const Location goal) { m_goal = goal; }

WallInfo Maze::walls(const Location cell) const { return m_walls[cell.x][cell.y]; }

bool Maze::has_unknown_walls(const Location cell) const {
    WallInfo walls_here = m_walls[cell.x][cell.y];
    return (walls_here.north == UNKNOWN || walls_here.east == UNKNOWN ||
            walls_here.south == UNKNOWN || walls_here.west == UNKNOWN);
}

int Maze::wall_count(const Location cell) const {
    int count = 4;
    if (is_exit(cell, NORTH)) count--;
    if (is_exit(cell, EAST))  count--;
    if (is_exit(cell, SOUTH)) count--;
    if (is_exit(cell, WEST))  count--;
    return count;
}

bool Maze::cell_is_visited(const Location cell) const {
    return !has_unknown_walls(cell);
}

bool Maze::is_exit(const Location cell, const Heading heading) const {
    WallInfo walls = m_walls[cell.x][cell.y];
    switch (heading) {
        case NORTH: return (walls.north & m_mask) == EXIT;
        case EAST:  return (walls.east  & m_mask) == EXIT;
        case SOUTH: return (walls.south & m_mask) == EXIT;
        case WEST:  return (walls.west  & m_mask) == EXIT;
        default:    return false;
    }
}

void Maze::update_wall_state(const Location cell, const Heading heading, const WallState state) {
   
   //  switch (heading) {
   //      case NORTH: if ((m_walls[cell.x][cell.y].north & UNKNOWN) != UNKNOWN) return; break;
   //      case EAST:  if ((m_walls[cell.x][cell.y].east  & UNKNOWN) != UNKNOWN) return; break;
   //      case WEST:  if ((m_walls[cell.x][cell.y].west  & UNKNOWN) != UNKNOWN) return; break;
   //      case SOUTH: if ((m_walls[cell.x][cell.y].south & UNKNOWN) != UNKNOWN) return; break;
   //      default:    return;
   //  }
    set_wall_state(cell, heading, state);
}

void Maze::initialise() {
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            m_walls[x][y].north = UNKNOWN;
            m_walls[x][y].east  = UNKNOWN;
            m_walls[x][y].south = UNKNOWN;
            m_walls[x][y].west  = UNKNOWN;
        }
    }
    for (int x = 0; x < MAZE_WIDTH; x++) {
        m_walls[x][0].south = WALL;
        m_walls[x][MAZE_HEIGHT - 1].north = WALL;
    }
    for (int y = 0; y < MAZE_HEIGHT; y++) {
        m_walls[0][y].west = WALL;
        m_walls[MAZE_WIDTH - 1][y].east = WALL;
    }
    set_wall_state(START, EAST, WALL);
    set_wall_state(START, NORTH, EXIT);

    set_mask(MASK_OPEN);
    flood(goal());
}

void Maze::set_mask(const MazeMask mask) { m_mask = mask; }
MazeMask Maze::get_mask() const { return m_mask; }

uint16_t Maze::neighbour_cost(const Location cell, const Heading heading) const {
    Location next_cell = cell.neighbour(heading);
    return m_cost[next_cell.x][next_cell.y];
}

uint16_t Maze::cost(const Location cell) const {
    return m_cost[cell.x][cell.y];
}

void Maze::flood(const Location target) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            m_cost[x][y] = (uint8_t)MAX_COST;
        }
    }

    Queue<Location, MAZE_CELL_COUNT / 4> queue;
    m_cost[target.x][target.y] = 0;
    queue.add(target);

    while (queue.size() > 0) {
        Location here = queue.head();
        uint16_t newCost = m_cost[here.x][here.y] + 1;
        for (int h = NORTH; h < HEADING_COUNT; h++) {
            Heading heading = static_cast<Heading>(h);
            if (is_exit(here, heading)) {
                Location nextCell = here.neighbour(heading);
                if (m_cost[nextCell.x][nextCell.y] > newCost) {
                    m_cost[nextCell.x][nextCell.y] = newCost;
                    queue.add(nextCell);
                }
            }
        }
    }
}

Heading Maze::heading_to_smallest(const Location cell, const Heading start_heading) const {
    Heading next_heading = start_heading;
    Heading best_heading = BLOCKED;
    uint16_t best_cost = cost(cell);

    uint16_t cost_val = neighbour_cost(cell, next_heading);
    if (cost_val < best_cost && is_exit(cell, next_heading)) { best_cost = cost_val; best_heading = next_heading; }

    next_heading = right_from(start_heading);
    cost_val = neighbour_cost(cell, next_heading);
    if (cost_val < best_cost && is_exit(cell, next_heading)) { best_cost = cost_val; best_heading = next_heading; }

    next_heading = left_from(start_heading);
    cost_val = neighbour_cost(cell, next_heading);
    if (cost_val < best_cost && is_exit(cell, next_heading)) { best_cost = cost_val; best_heading = next_heading; }

    next_heading = behind_from(start_heading);
    cost_val = neighbour_cost(cell, next_heading);
    if (cost_val < best_cost && is_exit(cell, next_heading)) { best_cost = cost_val; best_heading = next_heading; }

    if (best_cost == MAX_COST) best_heading = BLOCKED;
    return best_heading;
}

void Maze::set_wall_state(const Location loc, const Heading heading, const WallState state) {
    switch (heading) {
        case NORTH:
            m_walls[loc.x][loc.y].north = state;
            m_walls[loc.north().x][loc.north().y].south = state;
            break;
        case EAST:
            m_walls[loc.x][loc.y].east = state;
            m_walls[loc.east().x][loc.east().y].west = state;
            break;
        case WEST:
            m_walls[loc.x][loc.y].west = state;
            m_walls[loc.x][loc.y].west = state;
            m_walls[loc.west().x][loc.west().y].east = state;
            break;
        case SOUTH:
            m_walls[loc.x][loc.y].south = state;
            m_walls[loc.south().x][loc.south().y].north = state;
            break;
        default:
            break;
    }
}
