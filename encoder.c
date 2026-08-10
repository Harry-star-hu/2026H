#include "encoder.h"
#include "control.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t gLeftCount = 0U;
static volatile uint32_t gRightCount = 0U;
static volatile uint32_t gLeftTotal = 0U;
static volatile uint32_t gRightTotal = 0U;
static volatile uint32_t gLeftSpeed = 0U;
static volatile uint32_t gRightSpeed = 0U;

uint32_t Encoder_GetLeftSpeed(void)
{
    return gLeftSpeed;
}

uint32_t Encoder_GetRightSpeed(void)
{
    return gRightSpeed;
}

uint32_t Encoder_GetLeftTotal(void)
{
    return gLeftTotal;
}

uint32_t Encoder_GetRightTotal(void)
{
    return gRightTotal;
}

/* Only pulse count is needed; the chassis normally moves forward. */
void GROUP1_IRQHandler(void)
{
    uint32_t left = DL_GPIO_getEnabledInterruptStatus(
        GPIOA, GPIO_EncoderA_PIN_0_PIN | GPIO_EncoderA_PIN_1_PIN);
    uint32_t right = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, GPIO_EncoderB_PIN_2_PIN | GPIO_EncoderB_PIN_3_PIN);

    if ((left & GPIO_EncoderA_PIN_0_PIN) != 0U) {
        gLeftCount++;
        gLeftTotal++;
    }
    if ((left & GPIO_EncoderA_PIN_1_PIN) != 0U) {
        gLeftCount++;
        gLeftTotal++;
    }
    if ((right & GPIO_EncoderB_PIN_2_PIN) != 0U) {
        gRightCount++;
        gRightTotal++;
    }
    if ((right & GPIO_EncoderB_PIN_3_PIN) != 0U) {
        gRightCount++;
        gRightTotal++;
    }

    DL_GPIO_clearInterruptStatus(
        GPIOA, GPIO_EncoderA_PIN_0_PIN | GPIO_EncoderA_PIN_1_PIN);
    DL_GPIO_clearInterruptStatus(
        GPIOB, GPIO_EncoderB_PIN_2_PIN | GPIO_EncoderB_PIN_3_PIN);
}

void TIMER_Encoder_Read_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(
            TIMER_Encoder_Read_INST) == DL_TIMER_IIDX_ZERO) {
        gLeftSpeed = gLeftCount;
        gRightSpeed = gRightCount;
        gLeftCount = 0U;
        gRightCount = 0U;
        CarControl_On10msTick();
    }
}
