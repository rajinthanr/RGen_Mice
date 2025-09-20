#ifndef USART_H
#define USART_H

#ifdef __cplusplus
extern "C"
{
#endif

enum MazeView { PLAIN, COSTS, DIRS };

void UART_Configurations();
void debug();
void print_maze(int style);



#ifdef __cplusplus
}
#endif

#endif
