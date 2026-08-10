#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"

#include "control.h"
#include "ball_config.h"
#include "ball_control.h"
#include "ball_vision.h"
#include "rod_stepper.h"
#include "Delay.h"
#include "OLED.h"

#include <stdio.h>

#define KEY_START_PORT   GPIO_Key_PORT
#define KEY_START_PIN    GPIO_Key_PIN_S1_PIN

#define KEY_SELECT_PORT  GPIO_Key_PORT
#define KEY_SELECT_PIN   GPIO_Key_PIN_S2_PIN

#define KEY_ZERO_PORT    GPIO_Key_PORT
#define KEY_ZERO_PIN     GPIO_Key_PIN_S3_PIN

static uint8_t gBallSubsystemReady;
static uint8_t gCarMotionEstimateValid;
static uint32_t gCarMotionLastTick10ms;
static float gCarSpeedFiltered;
static float gCarLastSpeedFiltered;
static float gCarAccelerationFiltered;
static uint8_t gItem6Ready;
static uint8_t gItem6StableWindowActive;
static uint32_t gItem6StableStartMs;
static uint32_t gItem6AppliedTargetVersion;
static uint8_t gStopCaptureTriggered;
static uint8_t gTaskTimerStarted;
static uint8_t gTaskTimerRunning;
static uint32_t gTaskTimerStart10ms;
static uint32_t gTaskTimerStop10ms;

static float LimitMainFloat(
    float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float AbsMainFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void ResetTaskTimer(uint32_t now10ms)
{
    gTaskTimerStarted = 0U;
    gTaskTimerRunning = 0U;
    gTaskTimerStart10ms = now10ms;
    gTaskTimerStop10ms = now10ms;
}

static void StartTaskTimer(uint32_t now10ms)
{
    gTaskTimerStarted = 1U;
    gTaskTimerRunning = 1U;
    gTaskTimerStart10ms = now10ms;
    gTaskTimerStop10ms = now10ms;
}

static void StopTaskTimer(uint32_t now10ms)
{
    if (gTaskTimerRunning != 0U) {
        gTaskTimerStop10ms = now10ms;
        gTaskTimerRunning = 0U;
    }
}

static uint32_t GetTaskElapsed10ms(void)
{
    uint32_t end10ms;

    if (gTaskTimerStarted == 0U) {
        return 0U;
    }

    end10ms = (gTaskTimerRunning != 0U) ?
        CarControl_GetTick10ms() : gTaskTimerStop10ms;
    return end10ms - gTaskTimerStart10ms;
}

static void ResetCarMotionEstimate(uint32_t now10ms)
{
    gCarMotionEstimateValid = 0U;
    gCarMotionLastTick10ms = now10ms;
    gCarSpeedFiltered = 0.0f;
    gCarLastSpeedFiltered = 0.0f;
    gCarAccelerationFiltered = 0.0f;
}

static void UpdateCarMotionEstimate(
    uint32_t now10ms, uint8_t selectedItem)
{
    float rawSpeed;
    float acceleration;
    float speedFilterAlpha =
        (selectedItem == 6U) ?
        ITEM6_HOLD_CAR_SPEED_FILTER_ALPHA :
        HOLD_CAR_SPEED_FILTER_ALPHA;
    float accelerationFilterAlpha =
        (selectedItem == 6U) ?
        ITEM6_HOLD_CAR_ACCEL_FILTER_ALPHA :
        HOLD_CAR_ACCEL_FILTER_ALPHA;

    if (now10ms == gCarMotionLastTick10ms) {
        return;
    }
    gCarMotionLastTick10ms = now10ms;

    rawSpeed =
        ((float)CarControl_GetLeftEncoderSpeed() +
         (float)CarControl_GetRightEncoderSpeed()) * 0.5f;

    if (gCarMotionEstimateValid == 0U) {
        gCarSpeedFiltered = rawSpeed;
        gCarLastSpeedFiltered = rawSpeed;
        gCarAccelerationFiltered = 0.0f;
        gCarMotionEstimateValid = 1U;
        return;
    }

    gCarSpeedFiltered += speedFilterAlpha *
        (rawSpeed - gCarSpeedFiltered);
    acceleration =
        gCarSpeedFiltered - gCarLastSpeedFiltered;
    gCarLastSpeedFiltered = gCarSpeedFiltered;
    gCarAccelerationFiltered += accelerationFilterAlpha *
        (acceleration - gCarAccelerationFiltered);
}

static float CalculateCarMotionFeedforward(
    uint8_t selectedItem,
    uint8_t runActive)
{
    float feedforward;
    float accelerationAngle = 0.0f;
    float startFeedforwardDeg = HOLD_CAR_START_FF_DEG;
    float cruiseFeedforwardDeg = HOLD_CAR_CRUISE_FF_DEG;
    uint32_t startFeedforwardMs = HOLD_CAR_START_FF_MS;
    float feedforwardMaximumDeg =
        HOLD_CAR_FEEDFORWARD_MAX_DEG;
    float accelerationGainDeg = HOLD_CAR_ACCEL_GAIN_DEG;
    float accelerationDeadband = HOLD_CAR_ACCEL_DEADBAND;
    float stopBackwardAngleDeg =
        HOLD_STOP_FIXED_BACKWARD_ANGLE_DEG;

    if ((selectedItem != 4U) &&
        (selectedItem != 5U) &&
        (selectedItem != 6U)) {
        return 0.0f;
    }

    if (gCarMotionEstimateValid == 0U) {
        return 0.0f;
    }

    if (selectedItem == 6U) {
        startFeedforwardDeg = ITEM6_HOLD_CAR_START_FF_DEG;
        cruiseFeedforwardDeg = ITEM6_HOLD_CAR_CRUISE_FF_DEG;
        startFeedforwardMs = ITEM6_HOLD_CAR_START_FF_MS;
        feedforwardMaximumDeg =
            ITEM6_HOLD_CAR_FEEDFORWARD_MAX_DEG;
        accelerationGainDeg = ITEM6_HOLD_CAR_ACCEL_GAIN_DEG;
        accelerationDeadband = ITEM6_HOLD_CAR_ACCEL_DEADBAND;
        stopBackwardAngleDeg =
            ITEM6_HOLD_STOP_FIXED_BACKWARD_ANGLE_DEG;
    }

    /*
     * Once wheel PWM reaches zero, remove all vehicle feedforward.  The ball
     * feedback controller remains active, but stale encoder deceleration can
     * no longer keep a positive tilt and push the ball to the rear end.
     */
    if ((runActive == 0U) ||
        (CarControl_IsRunning() == 0U)) {
        return 0.0f;
    }

    if (CarControl_IsSoftStopping() != 0U) {
        float u =
            (float)CarControl_GetSoftStopProgressPermille() /
            1000.0f;
        float peakAngle = stopBackwardAngleDeg;
        float remaining = 1.0f - u;
        float stopFeedforward =
            BALL_OUTER_LOOP_SIGN *
            peakAngle * 6.75f * u *
            remaining * remaining;

        return LimitMainFloat(
            stopFeedforward,
            -feedforwardMaximumDeg,
            feedforwardMaximumDeg);
    }

    {
        uint32_t elapsedMs =
            CarControl_GetElapsed10ms() * 10U;

        if (elapsedMs < startFeedforwardMs) {
            return startFeedforwardDeg;
        }
        feedforward = cruiseFeedforwardDeg;
    }

    if (AbsMainFloat(gCarAccelerationFiltered) >=
        accelerationDeadband) {
        accelerationAngle =
            -accelerationGainDeg *
            gCarAccelerationFiltered;
        accelerationAngle = LimitMainFloat(
            accelerationAngle,
            -feedforwardMaximumDeg,
            feedforwardMaximumDeg);
    }

    return LimitMainFloat(
        feedforward + accelerationAngle,
        -feedforwardMaximumDeg,
        feedforwardMaximumDeg);
}

/*
 * 任务二直接上电测试时不使能视觉接收中断，也不启动步进定时器。
 * 只有切换到任务3～6，或执行滚球功能时才初始化一次。
 */
static void EnsureBallSubsystemInitialized(void)
{
    if (gBallSubsystemReady != 0U) {
        return;
    }

    RodStepper_Init();
    BallVision_Init();
    BallControl_Init();
    gBallSubsystemReady = 1U;
}

/*
 * 非阻塞按键检测。
 *
 * released为0时必须先检测到高电平，之后的高->低才算一次按下。
 * 即使某个按键接线错误而一直为低，也不会卡住整个主循环。
 */
static uint8_t KeyPressed(
    GPIO_Regs *port, uint32_t pin, uint8_t *released)
{
    if (DL_GPIO_readPins(port, pin) != 0U) {
        *released = 1U;
        return 0U;
    }

    if (*released == 0U) {
        return 0U;
    }

    Delay_ms(10);
    if (DL_GPIO_readPins(port, pin) == 0U) {
        *released = 0U;
        return 1U;
    }

    return 0U;
}

static int16_t FloatToTenth(float value)
{
    if (value >= 0.0f) {
        return (int16_t)(value * 10.0f + 0.5f);
    }
    return (int16_t)(value * 10.0f - 0.5f);
}

static void ShowStatus(uint8_t selectedItem)
{
    uint8_t zeroReady = BallControl_HasZero();
    uint8_t visionReady = BallVision_IsFresh(
    CarControl_GetTick10ms() * 10U,
    VISION_TIMEOUT_MS);
    char line1[32];
    char line2[32];
    char line3[32];
    uint32_t time10ms = GetTaskElapsed10ms();
    uint32_t seconds = time10ms / 100U;
    uint32_t hundredths = time10ms % 100U;
    int16_t x = FloatToTenth(BallControl_GetBallPositionCm());
    int16_t xt = FloatToTenth(BallControl_GetBallTargetCm());
    int16_t v = FloatToTenth(BallControl_GetHoldSpeedCmS());
    int16_t a = FloatToTenth(BallControl_GetRodAngleDeg());
    int16_t at = FloatToTenth(BallControl_GetRodTargetDeg());

    if (selectedItem == 2U) {
        (void)sprintf(
            line1, "M:2 C:%u L:%u",
            (unsigned int)CarControl_GetState(),
            (unsigned int)CarControl_GetCompletedLaps());
        (void)sprintf(
            line2, "T:%02lu.%02lu",
            (unsigned long)seconds,
            (unsigned long)hundredths);
        (void)sprintf(
            line3, "G:%02X V:%lu/%lu",
            CarControl_GetBlackMask(),
            (unsigned long)CarControl_GetLeftEncoderSpeed(),
            (unsigned long)CarControl_GetRightEncoderSpeed());
    }
#if BALL_HOLD_BENCH_TEST
    else if (selectedItem == 4U) {
        (void)sprintf(
            line1, "M4T%02lu.%02lu Z%uV%uF%u",
            (unsigned long)seconds,
            (unsigned long)hundredths,
            (unsigned int)zeroReady,
            (unsigned int)visionReady,
            (unsigned int)BallControl_GetFault());
        (void)sprintf(
            line2, "X:%d V:%d", (int)x, (int)v);
        (void)sprintf(
            line3, "A:%d T:%d", (int)a, (int)at);
    }
#endif
    else if (selectedItem == 6U) {
        (void)sprintf(
            line1, "6T%02lu.%02lu Z%uV%uP%uR%u",
            (unsigned long)seconds,
            (unsigned long)hundredths,
            (unsigned int)zeroReady,
            (unsigned int)visionReady,
            (unsigned int)BallVision_HasTarget(),
            (unsigned int)gItem6Ready);
        (void)sprintf(line2, "X:%d T:%d", (int)x, (int)xt);
        (void)sprintf(
            line3, "A:%d T:%d F:%u",
            (int)a, (int)at, (unsigned int)BallControl_GetFault());
    }
    else {
        (void)sprintf(
            line1, "M%uT%02lu.%02lu B%uZ%uV%u",
            selectedItem,
            (unsigned long)seconds,
            (unsigned long)hundredths,
            (unsigned int)BallControl_GetState(),
            (unsigned int)zeroReady,
            (unsigned int)visionReady);
        if ((selectedItem == 4U) || (selectedItem == 5U)) {
            (void)sprintf(line2, "X:%d V:%d", (int)x, (int)v);
        } else {
            (void)sprintf(line2, "X:%d T:%d", (int)x, (int)xt);
        }
        (void)sprintf(
            line3, "A:%d T:%d F:%u",
            (int)a, (int)at, (unsigned int)BallControl_GetFault());
    }

    OLED_ClearArea(0, 0, 128, 64);
    OLED_ShowString(0, 0, line1, OLED_8X16);
    OLED_ShowString(0, 22, line2, OLED_8X16);
    OLED_ShowString(0, 44, line3, OLED_8X16);
    OLED_Update();
}

static uint8_t NextItem(uint8_t item)
{
    if (item == 2U) {
        return 3U;
    }
    if (item == 3U) {
        return 4U;
    }
    if (item == 4U) {
        return 5U;
    }
    if (item == 5U) {
        return 6U;
    }
    return 2U;
}

static void ResetItem6Preparation(void)
{
    gItem6Ready = 0U;
    gItem6StableWindowActive = 0U;
    gItem6StableStartMs = 0U;
}

/*
 * Apply the touch target while Item 6 is selected and the car is idle.
 * This positions the ball before S1; the vehicle itself has no pre-tilt wait.
 */
static void UpdateItem6Preparation(
    uint8_t selectedItem, uint8_t runActive, uint32_t nowMs)
{
    uint32_t targetVersion;
    float targetCm;
    uint8_t inReadyWindow;

    if ((selectedItem != 6U) || (runActive != 0U) ||
        (gBallSubsystemReady == 0U)) {
        return;
    }

    if ((BallControl_HasZero() == 0U) ||
        (BallVision_HasTarget() == 0U) ||
        (BallVision_IsFresh(nowMs, VISION_TIMEOUT_MS) == 0U) ||
        (BallControl_HasFault() != 0U)) {
        ResetItem6Preparation();
        return;
    }

    targetVersion = BallVision_GetTargetVersion();
    targetCm = BallVision_GetTargetCm();
    BallControl_UseItem6HoldParameters();

    if (BallControl_GetState() != BALL_STATE_HOLD) {
        if (BallControl_StartItem6Hold(targetCm) == 0U) {
            ResetItem6Preparation();
            return;
        }
        gItem6AppliedTargetVersion = targetVersion;
        ResetItem6Preparation();
    } else if ((targetVersion != gItem6AppliedTargetVersion) ||
               (AbsMainFloat(
                    BallControl_GetBallTargetCm() - targetCm) > 0.01f)) {
        BallControl_SetHoldTarget(targetCm);
        BallControl_SetHoldFeedforwardDeg(0.0f);
        gItem6AppliedTargetVersion = targetVersion;
        ResetItem6Preparation();
    }

    inReadyWindow = (uint8_t)(
        (AbsMainFloat(
            BallControl_GetBallPositionCm() -
            BallControl_GetBallTargetCm()) <= ITEM6_READY_ERROR_CM) &&
        (AbsMainFloat(BallControl_GetBallSpeedCmS()) <=
         ITEM6_READY_SPEED_CM_S));

    if (inReadyWindow != 0U) {
        if (gItem6StableWindowActive == 0U) {
            gItem6StableWindowActive = 1U;
            gItem6StableStartMs = nowMs;
        } else if ((uint32_t)(nowMs - gItem6StableStartMs) >=
                   ITEM6_READY_STABLE_MS) {
            gItem6Ready = 1U;
        }
    } else {
        ResetItem6Preparation();
    }
}

static uint8_t StartSelectedItem(uint8_t item, uint32_t nowMs)
{
    /*
     * 第2项只测试小车循迹，不要求安装滚球机构。
     * 直接按S1即可启动，同时确保步进电机保持关闭。
     */
    if (item == 2U) {
        if (gBallSubsystemReady != 0U) {
            BallControl_Stop();
        }
        CarControl_Start(CAR_TEST_ITEM_2);
        return 1U;
    }

    EnsureBallSubsystemInitialized();

    /* 第3～6项使用滚球控制，必须先按S3完成水平标零。 */
    if (BallControl_HasZero() == 0U) {
        return 0U;
    }

    if (BallVision_IsFresh(nowMs, VISION_TIMEOUT_MS) == 0U) {
        return 0U;
    }

    if (item == 3U) {
        return BallControl_StartStaticSequence();
    }
    if (item == 4U) {
        if (BallControl_StartHold(0.0f) == 0U) {
            return 0U;
        }
#if BALL_HOLD_BENCH_TEST
        /* Bench test: keep TB6612 stopped and leave only ball hold active. */
        CarControl_Abort();
        return 0U;
#else
        /* Task 4 now has its own B indication and finish-stop state machine. */
        CarControl_Start(CAR_TEST_ITEM_4);
        return 1U;
#endif
    }
    if (item == 5U) {
        if (BallControl_StartHold(0.0f) == 0U) {
            return 0U;
        }
        CarControl_Start(CAR_TEST_ITEM_5);
        return 1U;
    }
    if (item == 6U) {
        if ((BallVision_HasTarget() == 0U) ||
            (gItem6Ready == 0U) ||
            (BallControl_GetState() != BALL_STATE_HOLD)) {
            return 0U;
        }
        CarControl_Start(CAR_TEST_ITEM_6);
        return 1U;
    }
    return 0U;
}

int main(void)
{
    uint8_t selectedItem = 2U;
    uint8_t runActive = 0U;
    uint8_t keyStartReleased = 0U;
    uint8_t keySelectReleased = 0U;
    uint8_t keyZeroReleased = 0U;
    uint32_t lastOled10ms = 0U;

    SYSCFG_DL_init();
    Delay_ms(100);
    

    OLED_Init();
    CarControl_Init();
    gBallSubsystemReady = 0U;
    gItem6AppliedTargetVersion = 0U;
    gStopCaptureTriggered = 0U;
    ResetItem6Preparation();
    ResetCarMotionEstimate(CarControl_GetTick10ms());
    ResetTaskTimer(CarControl_GetTick10ms());

    NVIC_ClearPendingIRQ(GPIO_EncoderA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_EncoderA_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_EncoderB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_EncoderB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_Encoder_Read_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_Encoder_Read_INST_INT_IRQN);

    DL_Timer_startCounter(TIMER_Encoder_Read_INST);
    DL_Timer_startCounter(PWM_0_INST);

    while (1) {
        uint32_t now10ms = CarControl_GetTick10ms();
        uint32_t nowMs = now10ms * 10U;

        CarControl_Update();
        UpdateCarMotionEstimate(now10ms, selectedItem);

        if (gBallSubsystemReady != 0U) {
            BallVision_Update(nowMs);

            if ((runActive != 0U) &&
                (gStopCaptureTriggered == 0U) &&
                ((CarControl_IsSoftStopping() != 0U) ||
                 (CarControl_HasFault() != 0U))) {
                BallControl_StartStopCapture();
                if (BallControl_IsStopCaptureActive() != 0U) {
                    gStopCaptureTriggered = 1U;
                }
            }

            if (gStopCaptureTriggered != 0U) {
                BallControl_SetStopCaptureVehicleStopped(
                    (uint8_t)(CarControl_IsRunning() == 0U));
            }

            BallControl_SetHoldFeedforwardDeg(
                CalculateCarMotionFeedforward(
                    selectedItem, runActive));
            BallControl_Update(nowMs);
            UpdateItem6Preparation(
                selectedItem, runActive, nowMs);
        }

        if ((gBallSubsystemReady != 0U) &&
            (BallControl_HasFault() != 0U)) {
            CarControl_Abort();
            ResetItem6Preparation();
            gStopCaptureTriggered = 0U;
            runActive = 0U;
            StopTaskTimer(now10ms);
        } else if ((runActive != 0U) && (selectedItem == 3U) &&
                   (BallControl_IsFinished() != 0U)) {
            /* 第3项完成后继续保持在-5 cm，等待下一次按键。 */
            runActive = 0U;
            StopTaskTimer(now10ms);
        } else if ((runActive != 0U) && (selectedItem != 3U) &&
                   ((CarControl_IsFinished() != 0U) ||
                    (CarControl_HasFault() != 0U))) {
            /*
             * Tasks 4/5 keep the ball at O after the car stops; Task 6 keeps
             * the selected live target.  Do not disable the stepper at its
             * current tilt.  Task 2 is unchanged.
             */
            if ((gBallSubsystemReady != 0U) &&
                (selectedItem != 4U) &&
                (selectedItem != 5U) &&
                (selectedItem != 6U)) {
                BallControl_Stop();
            }
            runActive = 0U;
            StopTaskTimer(now10ms);
        }

        if (runActive == 0U) {
            if (KeyPressed(
                    KEY_SELECT_PORT,
                    KEY_SELECT_PIN,
                    &keySelectReleased) != 0U) {
                if (BallControl_GetState() ==
                    BALL_STATE_STATIC_FINISHED) {
                    BallControl_Stop();
                }
                ResetCarMotionEstimate(now10ms);
                if (gBallSubsystemReady != 0U) {
                    BallControl_SetHoldFeedforwardDeg(0.0f);
                    BallControl_CancelStopCapture();
                }
                gStopCaptureTriggered = 0U;
                ResetItem6Preparation();
                selectedItem = NextItem(selectedItem);
                ResetTaskTimer(CarControl_GetTick10ms());
                if (selectedItem == 2U) {
                    if (gBallSubsystemReady != 0U) {
                        BallControl_Stop();
                    }
                } else {
                    EnsureBallSubsystemInitialized();
                }
            }

            /*
             * 任务3～6：先手动把水管调到水平，再按S3。
             * 此时软件把步进脉冲位置清零；运行中不能手动转动机构。
             */
            if ((selectedItem != 2U) &&
                (KeyPressed(
                    KEY_ZERO_PORT,
                    KEY_ZERO_PIN,
                    &keyZeroReleased) != 0U)) {
                EnsureBallSubsystemInitialized();
                BallControl_Stop();
                ResetCarMotionEstimate(now10ms);
                BallControl_ClearFault();
                BallControl_SetCurrentRodAsZero();
                gStopCaptureTriggered = 0U;
                ResetItem6Preparation();
                ResetTaskTimer(CarControl_GetTick10ms());
            }

            if (KeyPressed(
                    KEY_START_PORT,
                    KEY_START_PIN,
                    &keyStartReleased) != 0U) {
                ResetCarMotionEstimate(now10ms);
                if (gBallSubsystemReady != 0U) {
                    BallControl_SetHoldFeedforwardDeg(0.0f);
                }
                runActive =
                    StartSelectedItem(selectedItem, nowMs);
                if (runActive != 0U) {
                    if (gBallSubsystemReady != 0U) {
                        BallControl_CancelStopCapture();
                    }
                    gStopCaptureTriggered = 0U;
                    StartTaskTimer(CarControl_GetTick10ms());
                }
            }
        }

        if ((uint32_t)(now10ms - lastOled10ms) >= 10U) {
            lastOled10ms = now10ms;
            ShowStatus(selectedItem);
        }
    }
}
