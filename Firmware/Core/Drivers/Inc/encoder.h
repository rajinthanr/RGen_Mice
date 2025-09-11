#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

    // Encoder configuration and control
    void Encoder_Configration(void);
    float getLeftSpeed(void);
    float getRightSpeed(void);
    int32_t getLeftEncCount(void);
    int32_t getRightEncCount(void);
    float get_forward_dis(void);

    // Prototypes for recent functions and variables
    void updateEncoderCounts(void);
    void setEncoderDirection(int leftDir, int rightDir);
    extern volatile int32_t leftEncoderCount;
    extern volatile int32_t rightEncoderCount;

#ifdef __cplusplus
}
#endif

#endif
