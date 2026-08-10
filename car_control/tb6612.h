#ifndef TB6612_H_
#define TB6612_H_

#include <stdint.h>

void TB6612_Init(void);
void TB6612_Enable(void);
void TB6612_Disable(void);
void TB6612_SetMotor(int left_pwm, int right_pwm);
void TB6612_Stop(void);

#endif
