#ifndef ROD_STEPPER_H_
#define ROD_STEPPER_H_

#include <stdint.h>

void RodStepper_Init(void);
void RodStepper_Enable(uint8_t enable);
void RodStepper_SetSpeed(float steps_per_second);
void RodStepper_Stop(void);
void RodStepper_ResetLogicalPosition(void);

float RodStepper_GetCommandSpeed(void);
int32_t RodStepper_GetLogicalPosition(void);
uint8_t RodStepper_IsAtSoftLimit(void);

#endif
