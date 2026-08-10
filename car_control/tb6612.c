#include "tb6612.h"
#include "ti_msp_dl_config.h"

#define PWM_PERIOD_TICKS       2500U

/* Change one value to 1U if that wheel runs backward. */
#define LEFT_FORWARD_IN1_HIGH  0U
#define RIGHT_FORWARD_IN1_HIGH 0U

#define AIN1_HIGH() DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN1_PIN)
#define AIN1_LOW()  DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN1_PIN)
#define AIN2_HIGH() DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN2_PIN)
#define AIN2_LOW()  DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN2_PIN)
#define BIN1_HIGH() DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN1_PIN)
#define BIN1_LOW()  DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN1_PIN)
#define BIN2_HIGH() DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN2_PIN)
#define BIN2_LOW()  DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN2_PIN)

static int ClampPwm(int value)
{
    if (value > 100) {
        return 100;
    }
    if (value < -100) {
        return -100;
    }
    return value;
}

static void SetDuty(uint8_t duty, uint8_t channel)
{
    uint32_t compare;

    if (duty > 100U) {
        duty = 100U;
    }
    compare = PWM_PERIOD_TICKS -
              (PWM_PERIOD_TICKS * duty) / 100U;

    DL_Timer_setCaptureCompareValue(
        PWM_0_INST,
        compare,
        (channel == 0U) ? DL_TIMER_CC_0_INDEX :
                          DL_TIMER_CC_1_INDEX);
}

static void SetLeftDirection(uint8_t forward)
{
    uint8_t in1 = (forward != 0U) ?
                  LEFT_FORWARD_IN1_HIGH :
                  (uint8_t)(!LEFT_FORWARD_IN1_HIGH);

    if (in1 != 0U) {
        AIN1_HIGH();
        AIN2_LOW();
    } else {
        AIN1_LOW();
        AIN2_HIGH();
    }
}

static void SetRightDirection(uint8_t forward)
{
    uint8_t in1 = (forward != 0U) ?
                  RIGHT_FORWARD_IN1_HIGH :
                  (uint8_t)(!RIGHT_FORWARD_IN1_HIGH);

    if (in1 != 0U) {
        BIN1_HIGH();
        BIN2_LOW();
    } else {
        BIN1_LOW();
        BIN2_HIGH();
    }
}

void TB6612_Init(void)
{
    TB6612_Disable();
    TB6612_Stop();
}

void TB6612_Enable(void)
{
    DL_GPIO_setPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);
}

void TB6612_Disable(void)
{
    DL_GPIO_clearPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);
}

void TB6612_SetMotor(int left_pwm, int right_pwm)
{
    uint8_t left_duty;
    uint8_t right_duty;

    left_pwm = ClampPwm(left_pwm);
    right_pwm = ClampPwm(right_pwm);
    left_duty = (uint8_t)((left_pwm < 0) ? -left_pwm : left_pwm);
    right_duty = (uint8_t)((right_pwm < 0) ? -right_pwm : right_pwm);

    TB6612_Enable();

    if (left_pwm == 0) {
        AIN1_LOW();
        AIN2_LOW();
    } else {
        SetLeftDirection((left_pwm > 0) ? 1U : 0U);
    }

    if (right_pwm == 0) {
        BIN1_LOW();
        BIN2_LOW();
    } else {
        SetRightDirection((right_pwm > 0) ? 1U : 0U);
    }

    SetDuty(left_duty, 1U);   /* PWMA = CC1 */
    SetDuty(right_duty, 0U);  /* PWMB = CC0 */
}

void TB6612_Stop(void)
{
    SetDuty(0U, 1U);
    SetDuty(0U, 0U);
    AIN1_LOW();
    AIN2_LOW();
    BIN1_LOW();
    BIN2_LOW();
}
