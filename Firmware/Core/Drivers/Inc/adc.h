#ifndef ADC_H
#define ADC_H   

extern ADC_HandleTypeDef hadc1;

void ADC_Config(void);
uint16_t readADC(int8_t pin);
uint16_t readVolt();

#define read_Vol_Meter     readVolt()

//#define	read_Outz	       readADC(ADC1, 15,  ADC_SampleTime_84Cycles)
//#define	read_Vref	       readADC(ADC1, 14,  ADC_SampleTime_84Cycles)

#endif
