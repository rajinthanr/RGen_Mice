#include "usart.h"
#include "maze.h"
#include "mouse.h"
#include "variables.h"
#include "wall_handle.h"
#include <cstdarg>

uint8_t rxData;        // Single byte receive buffer
uint8_t rxBuffer[100]; // Main RX buffer
uint16_t rxIndex = 0;

extern UART_HandleTypeDef huart1; // Change to your UART instance

void print(const char *format, ...) {
  char buffer[200]; // Adjust size as needed
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (len > 0) {
    if (len > sizeof(buffer))
      len = sizeof(buffer); // Truncate if necessary
    for (int i = 0; i < len; i++) {
      uint8_t ss = __io_putchar(buffer[i]);
    }
  }
}

void UART_Configurations() {
  HAL_UART_Receive_IT(&huart1, &rxData, 1); // Receive 1 byte in interrupt mode
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // if (huart->Instance == USART1)
  if (huart1.gState == HAL_UART_STATE_READY) {
    if (cur_transmitting != cur_storing) {
      HAL_UART_Transmit_IT(&huart1, (uint8_t *)&buffer[cur_transmitting],
                           ptr[cur_transmitting]);
      ptr[cur_transmitting] = 0;
      cur_transmitting += 1;
      if (cur_transmitting >= MAX_LINES)
        cur_transmitting = 0;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) // check which UART
  {
    rxBuffer[rxIndex++] = rxData; // store received byte

    if (rxIndex >= sizeof(rxBuffer))
      rxIndex = 0; // prevent overflow
    if (rxData == '\n') {

      debug();
      rxIndex = 0;
    }

    // Restart interrupt reception for next byte
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
  }
}

#include <cstdlib>
#include <cstring>

void debug() {
  // Copy rxBuffer to a local buffer for parsing
  char cmdBuffer[100];
  memcpy(cmdBuffer, rxBuffer, rxIndex);
  cmdBuffer[rxIndex] = '\0'; // Null-terminate

  // Remove spaces from cmdBuffer
  int j = 0;
  for (int i = 0; i < rxIndex; ++i) {
    if (cmdBuffer[i] != ' ')
      cmdBuffer[j++] = cmdBuffer[i];
  }
  cmdBuffer[j] = '\0';

  for (int i = 0; i < j; ++i) {
    if (cmdBuffer[i] == '\n' || cmdBuffer[i] == '\r') {
      cmdBuffer[i] = '\0';
      break;
    }
  }

  // Parse command
  char *command = strtok(cmdBuffer, "=");
  char *valueStr = strtok(nullptr, "=");

  if (command && valueStr) {
    float value = 0;
    // Try to parse as float, but if not a number, keep as string
    if (valueStr &&
        (isdigit(valueStr[0]) || valueStr[0] == '-' || valueStr[0] == '+')) {
      value = atof(valueStr);
    }

    if (strcmp(command, "speed") == 0) {
      drive(value, 0);
    } else if (strcmp(command, "l") == 0) {
      motion.start_move(value, mouse.max_linear_speed, 0,
                        mouse.max_linear_accel);
    } else if (strcmp(command, "a") == 0) {
      motion.start_turn(value, mouse.max_angular_speed, 0,
                        mouse.max_angular_accel);
    } else if (strcmp(command, "ma") == 0) {
      mouse.max_angular_speed = value;
    } else if (strcmp(command, "ml") == 0) {
      mouse.max_linear_speed = value;
    } else if (strcmp(command, "maa") == 0) {
      mouse.max_angular_accel = value;
    } else if (strcmp(command, "mla") == 0) {
      mouse.max_linear_accel = value;
    }

    else if (strcmp(command, "turn") == 0) {
      print("Turning to %.2f degrees\r\n", value);
      is_run = 1;
    }

    else if (strcmp(cmdBuffer, "set_goal") == 0) {
      int gx = 0, gy = 0;
      if (sscanf(valueStr, "(%d,%d)", &gx, &gy) == 2) {
        maze.set_goal(Location(gx, gy));
        print("Goal set to (%d,%d)\n", gx, gy);
      } else {
        print("Invalid format for set_goal. Use (x,y)\n");
      }
    }
  }
  if (strcmp(cmdBuffer, "info") == 0) {
    print("-------\n%d  %d  %d  %d  \nOmega: %.2f Theta: %.2f Accelerration: "
          "%.2f \ncell1: %.3f cel2: %.3f \nlenc %d renc %d traveled: %.2f\n",
          reading[0], reading[1], reading[2], reading[3], get_gyroZ(), angle,
          get_accY(), cell_1 / 1000, cell_2 / 1000, getLeftEncCount(),
          getRightEncCount(), get_forward_dis());
    print("angular speed: %.2f\n", mouse.angular_speed);
    print("Target theta: %.2f Target dis: %.2f wall_front: %.2f\n-------\n ",
          mouse.target_angle, mouse.target_dis, wallFront());
  }

  else if (strcmp(cmdBuffer, "debug_mot") == 0) {
    debug_mot = !debug_mot;
    print("Motor debug: %d\n", debug_mot);
  }

  else if (strcmp(cmdBuffer, "dis") == 0) {
    for (int i = 0; i < 4; i++)
      print("  %.2f  |", dis_reading[i]);
    print("\r\n");
  }

  else if (strcmp(cmdBuffer, "enable_ir") == 0) {
    static uint8_t is_sensor_active = false;
    is_sensor_active = !is_sensor_active;
    if (is_sensor_active)
      enable();
    else
      disable();
    print("IR Enable: %d\n", is_sensor_active);
  } else if (strcmp(cmdBuffer, "run") == 0) {
    is_run = !is_run;
  } else if (strcmp(command, "stop") == 0) {
    mouse.target_angle = angle; // for safety
    mouse.target_dis = get_forward_dis();
    // Emergency stop: set speeds to zero, stop motors, etc.
    print("Emergency stop triggered!\r\n");
    // Add any additional emergency stop logic here
  } else if (strcmp(command, "reset") == 0) {
    NVIC_SystemReset();
  } else if (strcmp(command, "calibrate_wall") == 0) {
    is_calibrate = 1;
    print("Calibrating Wall\n");
  } else if (strcmp(command, "wall_front") == 0) {
    is_wall_front = 1;
    print("Wall Front: %d\r\n", is_wall_front);
    motion.start_move(60, mouse.max_linear_speed, 0, mouse.max_linear_accel);
    motion.start_turn(60, mouse.max_angular_speed, 0, mouse.max_angular_accel);
  } else if (strcmp(command, "mouse_enable") == 0) {
    is_mouse_enable = !is_mouse_enable;
    if (!is_mouse_enable) {
      drive_disable();
      // reset_pwm();
    } else
      drive_enable();
    print("Mouse Enable: %d\r\n", is_mouse_enable);
  } else if (strcmp(command, "wall_follow") == 0) {
    motion.start_move(600, mouse.max_linear_speed, 0, mouse.max_linear_accel);
    is_wall_follow = !is_wall_follow;
    print("Wall Follow: %d\r\n", is_wall_follow);
  } else if (strcmp(command, "map") == 0) {
    for (int i = 0; i < 1; i++) {
      print("Map\n");
      print("PLAIN\n");
      print("Ready...\n");
    }
    print_maze(PLAIN);
  }

  else if (strcmp(command, "mapc") == 0) {
    for (int i = 0; i < 1; i++) {
      print("Map\n");
      print("COST\n");
      print("Ready...\n");
    }
    int style = COSTS;
    print_maze(style);
  } else if (strcmp(command, "save_maze") == 0) {
    maze.save_to_flash();
    print("Maze saved to flash.\n");
  } else if (strcmp(command, "load_maze") == 0) {
    maze.load_from_flash();
    print("Maze loaded from flash.\n");
  }
}

#define POST 'o'
#define ERR '?'
#define GAP "         "
#define H_WALL "------"
#define H_EXIT "       "
#define H_UNKN "···"
#define H_VIRT "###"
#define V_WALL '|'
#define V_EXIT ' '
#define V_UNKN ':'
#define V_VIRT '#'

void print_justified(int32_t value, int width) {
  int v = value;
  int w = width;
  w--;
  if (v < 0) {
    w--;
  }
  while (v /= 10) {
    w--;
  }
  while (w > 0) {
    print("  ");
    --w;
  }
  print("%d", value);
}

void print_h_wall(uint8_t state) {
  if (state == EXIT) {
    print("%s", H_EXIT);
  } else if (state == WALL) {
    print("%s", H_WALL);
  } else if (state == VIRTUAL) {
    print("%s", H_VIRT);
  } else {
    print("%s", H_UNKN);
  }
}
void printNorthWalls(int y) {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    print("%c", POST);
    WallInfo walls = maze.walls(Location(x, y));
    print_h_wall(walls.north & maze.get_mask());
  }
  print("%c\n", POST);
}

void printSouthWalls(int y) {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    print("%c", POST);
    WallInfo walls = maze.walls(Location(x, y));
    print_h_wall(walls.south & maze.get_mask());
  }
  print("%c\n", POST);
}

void print_maze(int style) {
  const char dirChars[] = "^>v<* ";
  maze.flood(maze.goal());

  for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
    printNorthWalls(y);
    for (int x = 0; x < MAZE_WIDTH; x++) {
      Location location(x, y);
      WallInfo walls = maze.walls(location);
      uint8_t state = walls.west & maze.get_mask();
      if (state == EXIT) {
        print("%c", V_EXIT);
      } else if (state == WALL) {
        print("%c", V_WALL);
      } else if (state == VIRTUAL) {
        print("%c", V_VIRT);
      } else {
        print("%c", V_UNKN);
      }
      if (style == COSTS) {
        print_justified((int)maze.cost(location), 3);
      } else if (style == DIRS) {
        unsigned char direction = maze.heading_to_smallest(location, NORTH);
        if (location == maze.goal()) {
          direction = DIRECTION_COUNT;
        }
        char arrow = ' ';
        if (direction != BLOCKED) {
          arrow = dirChars[direction];
        }
        print(" ");
        print("%c", arrow);
        print(" ");
      } else {
        print("%s", GAP);
      }
    }
    print("%c\n", V_WALL);
  }
  printSouthWalls(0);
  print("\n");
}

const char hdg_letters[] = "NESW";

void log_action_status(char action, char note, Location location,
                       Heading heading) {
  print("{");
  print("%c", action);
  print("%c", note);
  print("[");
  print("%d", location.x);
  print(",");
  print("%d", location.y);
  print("]");
  print(" ");
  if (heading < HEADING_COUNT) {
    print("%c", hdg_letters[heading]);
  } else {
    print("!");
  }
  print_justified(get_front_sum(), 4);
  print("@");
  print_justified((int)motion.position(), 4);
  print(" ");
  // print_walls();
  print("}");
  print(" ");
}
