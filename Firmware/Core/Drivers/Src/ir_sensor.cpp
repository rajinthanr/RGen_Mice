#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#include "ir_sensor.h"
#include "main.h"

// Transmitter GPIO definitions (order: FL, L, R, FR)
GPIO_TypeDef *const tx_ports[4] = {TR_FL_GPIO_Port, TR_L_GPIO_Port, TR_R_GPIO_Port, TR_FR_GPIO_Port};
const uint16_t tx_pins[4] = {TR_FL_Pin, TR_L_Pin, TR_R_Pin, TR_FR_Pin};

// Receiver ADC channel mapping (order: FL, L, R, FR)
// Using receiver pins REC_FL (ADC channel 1), REC_L (ADC channel 0), REC_R (ADC channel 4), REC_FR (ADC channel 5)

const int8_t rx_channels[4] = {11, 10, 4, 5};

using namespace std;

int reading[4];
float dis_reading[4];
float ir_distance[4];

void ir_emitor(uint32_t idx, bool state)
{
    if (idx >= 4)
        return;
    HAL_GPIO_WritePin(tx_ports[idx], tx_pins[idx], state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int get_ir(int i)
{
#define DELAY_TIME 80

#define ON 1
#define OFF 0

    for (int j = 0; j < 4; j++)
        ir_emitor(j, OFF);

    delayMicroseconds(DELAY_TIME);

    int32_t total_off = 0;
    int32_t total_on = 0;

    // Take 50 samples with emitter OFF
    const int NUM_SAMPLES = 10;

    for (int sample = 0; sample < NUM_SAMPLES; ++sample)
    {
        total_off += readADC(rx_channels[i]);
    }
    int avg_off = total_off / NUM_SAMPLES;

    // Turn emitter ON
    ir_emitor(i, ON);
    delayMicroseconds(DELAY_TIME);

    // Take samples with emitter ON
    for (int sample = 0; sample < NUM_SAMPLES; ++sample)
    {
        total_on += readADC(rx_channels[i]);
    }
    int avg_on = total_on / NUM_SAMPLES;

    // Turn emitter OFF
    ir_emitor(i, OFF);

    reading[i] = avg_on - avg_off;

    if (reading[i] < 0)
        reading[i] = 0;

    return reading[i];
}

void update_ir()
{
    for (int i = 0; i < 4; i++)
    {
        reading[i] = get_ir(i);
    }
}

float map(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
    value = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    value = (value < toLow) ? toLow : (value > toHigh ? toHigh : value);
    return value;
}

void print_sensor_readings()
{
    // update_ir();
    // string str = "";
    // for (int a = 0; a < 4; a++)
    // {
    //     printf("%d\t", reading[a]);
    //     // printf("%.2f\t", ir_distance[a]);
    // }

    // printf("\n");
}





double sum(float (*func)(int), double start, double end, double increment)
{
    double sum = 0.0;
    for (double x = start; x <= end; x += increment)
    {
        sum += func(x);
    }
    return sum;
}
