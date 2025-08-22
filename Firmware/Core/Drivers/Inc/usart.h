#ifndef USART_H
#define USART_H
#include "stm32f4xx_hal.h"
#include <stdio.h>

int _write(int file, char *ptr, int len);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
int _write(int file, char *ptr, int len);
int _read(int file, char *ptr, int len);

#endif
