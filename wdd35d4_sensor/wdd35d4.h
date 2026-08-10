#ifndef WDD35D4_H_
#define WDD35D4_H_

#include <stdint.h>

void WDD35D4_Init(void);
void WDD35D4_Update(void);
void WDD35D4_SetCurrentAsZero(void);
void WDD35D4_SetZeroAdc(float adc_value);

uint16_t WDD35D4_GetRawAdc(void);
float WDD35D4_GetFilteredAdc(void);
float WDD35D4_GetAngleDeg(void);
float WDD35D4_GetZeroAdc(void);
uint8_t WDD35D4_IsValid(void);

#endif
