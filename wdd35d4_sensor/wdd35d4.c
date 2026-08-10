#include "wdd35d4.h"
#include "ball_config.h"

static uint16_t gRawAdc;
static float gFilteredAdc;
static float gZeroAdc;
static uint8_t gInitialized;
static uint8_t gValid;

void WDD35D4_Init(void)
{
    gRawAdc = 0U;
    gFilteredAdc = WDD_DEFAULT_ZERO_ADC;
    gZeroAdc = WDD_DEFAULT_ZERO_ADC;
    gInitialized = 0U;
    gValid = 0U;

    /* SysConfig 中将 ADC 配成 Repeat Single、Software trigger、MEM0。 */
    DL_ADC12_startConversion(WDD_ADC_INST);
}

void WDD35D4_Update(void)
{
    uint16_t raw = DL_ADC12_getMemResult(WDD_ADC_INST, WDD_ADC_MEM);

    gRawAdc = raw;
    gValid = (uint8_t)((raw > WDD_VALID_ADC_MIN) &&
                       (raw < WDD_VALID_ADC_MAX));

    if (gInitialized == 0U) {
        gFilteredAdc = (float)raw;
        gInitialized = 1U;
    } else {
        gFilteredAdc += WDD_FILTER_ALPHA *
                        ((float)raw - gFilteredAdc);
    }
}

void WDD35D4_SetCurrentAsZero(void)
{
    if (gValid != 0U) {
        gZeroAdc = gFilteredAdc;
    }
}

void WDD35D4_SetZeroAdc(float adc_value)
{
    gZeroAdc = adc_value;
}

uint16_t WDD35D4_GetRawAdc(void)
{
    return gRawAdc;
}

float WDD35D4_GetFilteredAdc(void)
{
    return gFilteredAdc;
}

float WDD35D4_GetAngleDeg(void)
{
    return WDD_ANGLE_SIGN *
           (gFilteredAdc - gZeroAdc) / WDD_COUNTS_PER_DEGREE;
}

float WDD35D4_GetZeroAdc(void)
{
    return gZeroAdc;
}

uint8_t WDD35D4_IsValid(void)
{
    return gValid;
}
