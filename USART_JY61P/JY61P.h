#ifndef JY61P_H_
#define JY61P_H_

#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * These three names match the SysConfig instance used by the supplied code.
 * If the instance is renamed, only change this block.
 */
#define JY61P_UART_INST  UART_JY61P_INST
#define JY61P_UART_IRQN  UART_JY61P_INST_INT_IRQN
#define JY61P_UART_ISR   UART_JY61P_INST_IRQHandler

#define JY61P_AXIS_X     0U
#define JY61P_AXIS_Y     1U
#define JY61P_AXIS_Z     2U

/* Values are updated by the UART interrupt after checksum validation. */
extern volatile float JY61P_AccXG;
extern volatile float JY61P_AccYG;
extern volatile float JY61P_AccZG;
extern volatile float Roll;
extern volatile float Pitch;
extern volatile float Yaw;

extern volatile uint32_t JY61P_RxByteCount;
extern volatile uint32_t JY61P_ValidAccelFrameCount;
extern volatile uint32_t JY61P_ValidAngleFrameCount;
extern volatile uint32_t JY61P_BadFrameCount;

void JY61P_Init(void);
void JY61P_Poll(void);
float JY61P_GetAccelerationG(uint8_t axis);
void Serial_JY61P_Zero_Yaw(void);

#endif
