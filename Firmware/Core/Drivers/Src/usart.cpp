#include "usart.h"
#include "core.h"
#include "profile.h"
#include "motion.h"

uint8_t rxData;        // Single byte receive buffer
uint8_t rxBuffer[100]; // Main RX buffer
uint16_t rxIndex = 0;

extern UART_HandleTypeDef huart1; // Change to your UART instance\

void UART_Configurations()
{
    HAL_UART_Receive_IT(&huart1, &rxData, 1); // Receive 1 byte in interrupt mode
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Transmission finished
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // check which UART
    {
        rxBuffer[rxIndex++] = rxData; // store received byte

        if (rxIndex >= sizeof(rxBuffer))
            rxIndex = 0; // prevent overflow
        if (rxData == '\n')
        {

            debug();
            rxIndex = 0;
        }

        // Restart interrupt reception for next byte
        HAL_UART_Receive_IT(&huart1, &rxData, 1);
    }
}

#include <cstring>
#include <cstdlib>

void debug()
{
    // Copy rxBuffer to a local buffer for parsing
    char cmdBuffer[100];
    memcpy(cmdBuffer, rxBuffer, rxIndex);
    cmdBuffer[rxIndex] = '\0'; // Null-terminate

    // Remove spaces from cmdBuffer
    int j = 0;
    for (int i = 0; i < rxIndex; ++i)
    {
        if (cmdBuffer[i] != ' ')
            cmdBuffer[j++] = cmdBuffer[i];
    }
    cmdBuffer[j] = '\0';

    for (int i = 0; i < j; ++i)
    {
        if (cmdBuffer[i] == '\n' || cmdBuffer[i] == '\r')
        {
            cmdBuffer[i] = '\0';
            break;
        }
    }

    // Parse command
    char *command = strtok(cmdBuffer, "=");
    char *valueStr = strtok(nullptr, "=");

    if (command && valueStr)
    {
        float value = atof(valueStr);

        if (strcmp(command, "speed") == 0)
        {
            drive(value, 0);
        }
        else if (strcmp(command, "l") == 0)
        {
            motion.start_move(value, mouse.max_linear_speed, 0, mouse.max_linear_accel);
        }
        else if (strcmp(command, "a") == 0)
        {
            motion.start_turn(value, mouse.max_angular_speed, 0, mouse.max_angular_accel);
        }
        else if (strcmp(command, "ma") == 0)
        {
            mouse.max_angular_speed = value;
        }
        else if (strcmp(command, "ml") == 0)
        {
            mouse.max_linear_speed = value;
        }
        else if (strcmp(command, "maa") == 0)
        {
            mouse.max_angular_accel = value;
        }
        else if (strcmp(command, "mla") == 0)
        {
            mouse.max_linear_accel = value;
        }

        else if (strcmp(command, "turn") == 0)
        {
            printf("Turning to %.2f degrees\r\n", value);
            is_run = 1;
        }
    }
    if (strcmp(cmdBuffer, "info") == 0)
        printf("%d  %d  %d  %d  aSpeed %.2f angle %.2f acc %.2f cell1 %.0f cell2 %.0f lenc %d renc %d for %.2f\r\n", reading[0], reading[1], reading[2], reading[3], get_gyroZ(), angle, get_accY(), cell_1, cell_2, getLeftEncCount(), getRightEncCount(), get_forward_dis());
    else if (strcmp(cmdBuffer, "dis") == 0)
    {
        for (int i = 0; i < 4; i++)
            printf("%.2f\  ", i, dis_reading[i]);
        printf("\r\n");
    }
    else if (strcmp(cmdBuffer, "run") == 0)
    {
        is_run = !is_run;
    }
    else if (strcmp(command, "stop") == 0)
    {
        // Emergency stop: set speeds to zero, stop motors, etc.
        is_mouse_enable = 0;
        is_wall_front = 0;
        is_wall_follow = 0;
        drive(0, 0);
        is_run = 0;
        printf("Emergency stop triggered!\r\n");
        // Add any additional emergency stop logic here
    }
    else if (strcmp(command, "reset") == 0)
    {
        NVIC_SystemReset();
    }
    else if (strcmp(command, "calibrate_wall") == 0)
    {
        is_calibrate = 1;
        printf("Calibrating Wall\n");
    }
    else if (strcmp(command, "wall_front") == 0)
    {
        is_wall_front = 1;
        printf("Wall Front: %d\r\n", is_wall_front);
        motion.start_move(60, mouse.max_linear_speed, 0, mouse.max_linear_accel);
        motion.start_turn(60, mouse.max_angular_speed, 0, mouse.max_angular_accel);
    }
    else if (strcmp(command, "mouse_enable") == 0)
    {
        is_mouse_enable = !is_mouse_enable;
        if (!is_mouse_enable)
        {
            drive(0, 0);
        }
        printf("Mouse Enable: %d\r\n", is_mouse_enable);
    }
    else if (strcmp(command, "wall_follow") == 0)
    {
        motion.start_move(600, mouse.max_linear_speed, 0, mouse.max_linear_accel);
        is_wall_follow = !is_wall_follow;
        printf("Wall Follow: %d\r\n", is_wall_follow);
    }

}
