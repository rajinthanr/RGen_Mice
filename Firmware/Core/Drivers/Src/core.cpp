#include "main.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1; // Change to your UART instance

void systick(void)
{
}

void button1_interrupt(void)
{
}

void button2_interrupt(void)
{
}

int core(void)
{
	Systick_Configuration();
    LED_Configuration();
    IR_Configuration();
//    button_Configuration();
//    usart1_Configuration(9600);
//    SPI_Configuration();
//    TIM4_PWM_Init();
//    Encoder_Configration();
//    buzzer_Configuration();
    ADC_Config();
//
//    shortBeep(2000, 8000);
    while (1)
    {
    	LED_Blink(1, 1000, 1);
    	HAL_UART_Transmit(&huart1, "Hi\n",2, HAL_MAX_DELAY);
//        readSensor();
//        readGyro();
//        readVolMeter();
//        printf("LF %d RF %d DL %d DR %d aSpeed %d angle %d voltage %d lenc %d renc %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor, aSpeed, angle, voltage, getLeftEncCount(), getRightEncCount());
//        displayMatrix("mous");
//
//        setLeftPwm(100);
//        setRightPwm(100);
//        delay_ms(1000);
    	printf("what");
    }
}
