#ifndef USART_H
#define USART_H

#ifdef __cplusplus
//#include "core.h"
#include "maze.h"
#include <cstdarg>
void print(const char *format, ...);
void log_action_status(char action, char note, Location location, Heading heading);
extern "C"
{
#endif

#include <cstdarg>
//#include "core.h"
//#include "maze.h"

enum MazeView { PLAIN, COSTS, DIRS };

void UART_Configurations();
void debug();
void print_maze(int style);




#ifdef __cplusplus
}
#endif

#endif
