#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

constexpr int N = 16;
enum Direction { NORTH, EAST, SOUTH, WEST };
struct Cell {
  bool wall[4] = {false, false, false, false};
  int dist = 255;
};
struct Pos {
  int x, y;
};

Cell maze[N][N];
Pos mouse = {0, 0};
Direction heading = NORTH;

int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

bool in_bounds(int x, int y) { return x >= 0 && x < N && y >= 0 && y < N; }

void draw_maze() {
  system("clear"); // or "cls" on Windows
  for (int y = N - 1; y >= 0; y--) {
    // Draw top walls
    for (int x = 0; x < N; x++) {
      std::cout << "+";
      std::cout << (maze[x][y].wall[NORTH] ? "---" : "   ");
    }
    std::cout << "+\n";

    // Draw side walls and mouse
    for (int x = 0; x < N; x++) {
      std::cout << (maze[x][y].wall[WEST] ? "|" : " ");
      if (mouse.x == x && mouse.y == y)
        std::cout << " M ";
      else
        std::cout << "   ";
    }
    std::cout << "|\n";
  }

  // Bottom line
  for (int x = 0; x < N; x++)
    std::cout << "+---";
  std::cout << "+\n";
}

void move_mouse() {
  if (mouse.x < N - 1)
    mouse.x++;
}

int main() {
  // Example walls
  maze[1][0].wall[NORTH] = true;
  maze[1][1].wall[SOUTH] = true;
  maze[2][2].wall[WEST] = true;

  for (int i = 0; i < 5; i++) {
    draw_maze();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    move_mouse();
  }
}
