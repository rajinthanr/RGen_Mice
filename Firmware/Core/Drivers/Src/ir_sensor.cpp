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
// Re-order to match logical order: FL, L, R, FR -> channels: 1,0,4,5
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

uint16_t readADC(int8_t pin)
{
    ADC_ChannelConfTypeDef sConfig;
    uint16_t adc_val = 0;
    // if (pin == 7)
    //     pin = -1;
    // pin = pin + 1;

    if (pin >= 0 && pin <= 15)
    {
        switch (pin)
        {
        case 0:
            sConfig.Channel = ADC_CHANNEL_0;
            break;
        case 1:
            sConfig.Channel = ADC_CHANNEL_1;
            break;
        case 2:
            sConfig.Channel = ADC_CHANNEL_2;
            break;
        case 3:
            sConfig.Channel = ADC_CHANNEL_3;
            break;
        case 4:
            sConfig.Channel = ADC_CHANNEL_4;
            break;
        case 5:
            sConfig.Channel = ADC_CHANNEL_5;
            break;
        case 6:
            sConfig.Channel = ADC_CHANNEL_6;
            break;
        case 7:
            sConfig.Channel = ADC_CHANNEL_7;
            break;
        case 8:
            sConfig.Channel = ADC_CHANNEL_8;
            break;
        case 9:
            sConfig.Channel = ADC_CHANNEL_9;
            break;
        case 10:
            sConfig.Channel = ADC_CHANNEL_10;
            break;
        case 11:
            sConfig.Channel = ADC_CHANNEL_11;
            break;
        case 12:
            sConfig.Channel = ADC_CHANNEL_12;
            break;
        case 13:
            sConfig.Channel = ADC_CHANNEL_13;
            break;
        case 14:
            sConfig.Channel = ADC_CHANNEL_14;
            break;
        case 15:
            sConfig.Channel = ADC_CHANNEL_15;
            break;
        }

        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            adc_val = HAL_ADC_GetValue(&hadc1);
        }
    }

    return adc_val;
}

void delay(uint32_t ms)
{
    HAL_Delay(ms);
}

void delayMicroseconds(uint32_t us)
{
    uint64_t start = micros();
    while (micros() - start < us)
        ;
}

uint64_t millis()
{
    return HAL_GetTick();
}

uint64_t micros()
{
    static uint64_t overflow_count = 0;
    static uint16_t last_timer_value = 0;
    uint16_t current_timer_value = __HAL_TIM_GET_COUNTER(&htim10);

    if (current_timer_value < last_timer_value)
    {
        overflow_count++;
    }

    last_timer_value = current_timer_value;

    return (overflow_count << 16) | current_timer_value;
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