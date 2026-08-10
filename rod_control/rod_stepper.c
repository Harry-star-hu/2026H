#include "rod_stepper.h"
#include "ball_config.h"

static volatile int32_t gSpeedStepsPerSecond;
static volatile int32_t gLogicalPosition;
static volatile uint32_t gPhase;
static volatile int8_t gLastDirection;
static volatile uint8_t gEnabled;
static volatile uint8_t gAtSoftLimit;

static void WriteEnable(uint8_t enable)
{
    uint8_t high;

    if (ROD_ENABLE_ACTIVE_LOW != 0U) {
        high = (uint8_t)(enable == 0U);
    } else {
        high = (uint8_t)(enable != 0U);
    }

    if (high != 0U) {
        DL_GPIO_setPins(ROD_EN_PORT, ROD_EN_PIN);
    } else {
        DL_GPIO_clearPins(ROD_EN_PORT, ROD_EN_PIN);
    }
}

static void WriteDirection(int8_t direction)
{
    uint8_t positiveHigh = ROD_POSITIVE_DIR_HIGH;

    if (ROD_MOTOR_SIGN < 0) {
        direction = (int8_t)-direction;
    }

    if (((direction > 0) && (positiveHigh != 0U)) ||
        ((direction < 0) && (positiveHigh == 0U))) {
        DL_GPIO_setPins(ROD_DIR_PORT, ROD_DIR_PIN);
    } else {
        DL_GPIO_clearPins(ROD_DIR_PORT, ROD_DIR_PIN);
    }
}

void RodStepper_Init(void)
{
    gSpeedStepsPerSecond = 0;
    gLogicalPosition = 0;
    gPhase = 0U;
    gLastDirection = 0;
    gEnabled = 0U;
    gAtSoftLimit = 0U;

    DL_GPIO_clearPins(ROD_STEP_PORT, ROD_STEP_PIN);
    WriteEnable(0U);

    NVIC_ClearPendingIRQ(ROD_TIMER_IRQN);
    NVIC_EnableIRQ(ROD_TIMER_IRQN);
    DL_Timer_startCounter(ROD_TIMER_INST);
}

void RodStepper_Enable(uint8_t enable)
{
    gEnabled = (uint8_t)(enable != 0U);
    if (gEnabled == 0U) {
        gSpeedStepsPerSecond = 0;
        gPhase = 0U;
        DL_GPIO_clearPins(ROD_STEP_PORT, ROD_STEP_PIN);
    }
    WriteEnable(gEnabled);
}

void RodStepper_SetSpeed(float steps_per_second)
{
    if (steps_per_second > ROD_MAX_STEP_RATE) {
        steps_per_second = ROD_MAX_STEP_RATE;
    } else if (steps_per_second < -ROD_MAX_STEP_RATE) {
        steps_per_second = -ROD_MAX_STEP_RATE;
    }

    if (steps_per_second >= 0.0f) {
        gSpeedStepsPerSecond = (int32_t)(steps_per_second + 0.5f);
    } else {
        gSpeedStepsPerSecond = (int32_t)(steps_per_second - 0.5f);
    }
}

void RodStepper_Stop(void)
{
    gSpeedStepsPerSecond = 0;
    gPhase = 0U;
}

void RodStepper_ResetLogicalPosition(void)
{
    gLogicalPosition = 0;
    gAtSoftLimit = 0U;
}

float RodStepper_GetCommandSpeed(void)
{
    return (float)gSpeedStepsPerSecond;
}

int32_t RodStepper_GetLogicalPosition(void)
{
    return gLogicalPosition;
}

uint8_t RodStepper_IsAtSoftLimit(void)
{
    return gAtSoftLimit;
}

void ROD_TIMER_ISR(void)
{
    int32_t speed;
    uint32_t magnitude;
    int8_t direction;
    int32_t nextPosition;

    switch (DL_TimerG_getPendingInterrupt(ROD_TIMER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            break;
        default:
            return;
    }

    /* STEP 高电平只保持一个 20 kHz 中断周期。 */
    DL_GPIO_clearPins(ROD_STEP_PORT, ROD_STEP_PIN);

    if (gEnabled == 0U) {
        return;
    }

    speed = gSpeedStepsPerSecond;
    if (speed == 0) {
        return;
    }

    direction = (speed > 0) ? 1 : -1;
    magnitude = (uint32_t)((speed > 0) ? speed : -speed);

    /* 改变方向后空出一个中断周期，满足 DIR 建立时间。 */
    if (direction != gLastDirection) {
        WriteDirection(direction);
        gLastDirection = direction;
        gPhase = 0U;
        return;
    }

    gPhase += magnitude;
    if (gPhase < ROD_TIMER_HZ) {
        return;
    }
    gPhase -= ROD_TIMER_HZ;

    nextPosition = gLogicalPosition + (int32_t)direction;
    if ((nextPosition > ROD_SOFT_LIMIT_STEPS) ||
        (nextPosition < -ROD_SOFT_LIMIT_STEPS)) {
        gAtSoftLimit = 1U;
        gSpeedStepsPerSecond = 0;
        return;
    }

    gAtSoftLimit = 0U;
    gLogicalPosition = nextPosition;
    DL_GPIO_setPins(ROD_STEP_PORT, ROD_STEP_PIN);
}
