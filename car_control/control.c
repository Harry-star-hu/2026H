/*
 * H problem line tracking.
 * This file contains only sensors, PD steering and the A-marker state machine.
 */

#include "control.h"
#include "encoder.h"
#include "tb6612.h"
#include "ti_msp_dl_config.h"

/* ======================== Parameters to tune ========================== */

#define BLACK_IS_LOW                   1U

#define FAST_LEFT_PWM                 42
#define FAST_RIGHT_PWM                45
#define FAST_CURVE_LEFT_PWM           36
#define FAST_CURVE_RIGHT_PWM          39

#define BALANCE_LEFT_PWM              25
#define BALANCE_RIGHT_PWM             28
#define BALANCE_CURVE_LEFT_PWM        25
#define BALANCE_CURVE_RIGHT_PWM       28

#define LINE_KP                       1.8f
#define LINE_KD                       0.15f
#define STEERING_SIGN                -1.0f
#define CURVE_ERROR                   1.8f
#define MAX_STEERING_PWM              20.0f
#define MAX_TURN_CHANGE_10MS           4.0f

#define LOST_LINE_TIMEOUT_10MS        20U
#define LOST_FAST_PWM                 18
#define LOST_SLOW_PWM                 5

#define A_MARKER_BLACK_COUNT          3U
#define A_MARKER_CONFIRM_10MS         2U
#define LEAVE_A_MAX_BLACK_COUNT       3U
#define LEAVE_A_CONFIRM_10MS          10U
#define MIN_LAP_TIME_10MS             500U

/* Task 4: timed B indication, then arm the later finish marker. */
#define ITEM4_B_LED_START_10MS         700U
#define ITEM4_LED_ON_TIME_10MS         100U
#define ITEM4_FINISH_CLEAR_10MS         10U

/* Task 5: A indication for 1 s, start soft stop 5 s after detection. */
#define ITEM5_LED_ON_TIME_10MS         100U
#define ITEM5_FINISH_DELAY_10MS        500U

/* Task 6 keeps the previous 5 s delayed-stop behavior. */
#define ITEM6_FINISH_DELAY_10MS        500U

#define FINISH_ADVANCE_ENCODER_COUNTS 0U
#define FINISH_ADVANCE_TIMEOUT_10MS   100U
#define PASS_A_AFTER_MARKER_10MS      20U

#define ITEM2_TIMEOUT_10MS            3500U
#define BALANCE_TIMEOUT_10MS          4500U

/* Tasks 4/5/6 tilt and start their wheel ramps at the same instant. */
#define ITEM45_START_RAMP_10MS          60U
#define ITEM5_STOP_RAMP_10MS            80U
#define ITEM45_RAMP_MIN_PERCENT          0U

/* Tasks 4/5/6 share the same road-mode detection and speed transitions. */
#define ITEM5_CURVE_FILTER_ALPHA          0.20f
#define ITEM5_CURVE_ENTER_ERROR           1.40f
#define ITEM5_CURVE_EXIT_ERROR            0.80f
#define ITEM5_CURVE_ENTER_TURN            2.50f
#define ITEM5_CURVE_EXIT_TURN             1.20f
#define ITEM5_CURVE_ENTER_WHEEL_DIFF      0.18f
#define ITEM5_CURVE_EXIT_WHEEL_DIFF       0.10f
#define ITEM5_CURVE_ENTER_CONFIRM_10MS   10U
#define ITEM5_CURVE_EXIT_CONFIRM_10MS    25U
#define ITEM5_BASE_SLEW_PWM_10MS          0.11f

/* Keep the final line correction small while the stop envelope decays. */
#define ITEM5_STOP_STEERING_DELTA_MAX     4.0f
#define ITEM5_STOP_PWM_CHANGE_10MS        2

#define P1 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_1_PORT, GPIO_Gray_PIN_Gray_1_PIN)
#define P2 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_2_PORT, GPIO_Gray_PIN_Gray_2_PIN)
#define P3 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_3_PORT, GPIO_Gray_PIN_Gray_3_PIN)
#define P4 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_4_PORT, GPIO_Gray_PIN_Gray_4_PIN)
#define P5 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_5_PORT, GPIO_Gray_PIN_Gray_5_PIN)
#define P6 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_6_PORT, GPIO_Gray_PIN_Gray_6_PIN)
#define P7 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_7_PORT, GPIO_Gray_PIN_Gray_7_PIN)
#define P8 DL_GPIO_readPins(GPIO_Gray_PIN_Gray_8_PORT, GPIO_Gray_PIN_Gray_8_PIN)

/* ======================== Runtime state =============================== */

static volatile uint32_t gTick10ms = 0U;

static CarControlState gState = CAR_STATE_IDLE;
static CarTestMode gMode = CAR_TEST_ITEM_2;
static uint8_t gRunning = 0U;
static uint8_t gFinished = 0U;
static uint8_t gFault = 0U;
static uint8_t gCompletedLaps = 0U;
static uint8_t gBlackMask = 0U;

static uint8_t gLeaveTicks = 0U;
static uint8_t gMarkerTicks = 0U;
static uint8_t gLostTicks = 0U;
static uint8_t gItem4FinishArmed = 0U;
static uint8_t gItem4ClearTicks = 0U;
static float gLastError = 0.0f;
static float gLastTurn = 0.0f;

static int gLastLeftPwm = 0;
static int gLastRightPwm = 0;
static int gItem5StopStartLeftPwm = 0;
static int gItem5StopStartRightPwm = 0;
static float gItem5StopStartTurn = 0.0f;

static uint8_t gItem5CurveActive = 0U;
static uint8_t gItem5CurveEnterTicks = 0U;
static uint8_t gItem5CurveExitTicks = 0U;
static float gItem5CurveErrorFiltered = 0.0f;
static float gItem5CurveTurnFiltered = 0.0f;
static float gItem5CurveWheelDiffFiltered = 0.0f;
static float gItem5BaseLeftPwm = (float)BALANCE_LEFT_PWM;
static float gItem5BaseRightPwm = (float)BALANCE_RIGHT_PWM;

static uint32_t gLastUpdateTick = 0U;
static uint32_t gRunStartTick = 0U;
static uint32_t gRunStopTick = 0U;
static uint32_t gStateStartTick = 0U;
static uint32_t gMarkerLeftTotal = 0U;
static uint32_t gMarkerRightTotal = 0U;

static int gStraightLeft = FAST_LEFT_PWM;
static int gStraightRight = FAST_RIGHT_PWM;
static int gCurveLeft = FAST_CURVE_LEFT_PWM;
static int gCurveRight = FAST_CURVE_RIGHT_PWM;

/* ======================== Sensors ===================================== */

static uint8_t IsBlack(uint32_t level)
{
#if BLACK_IS_LOW
    return (level == 0U) ? 1U : 0U;
#else
    return (level != 0U) ? 1U : 0U;
#endif
}

static uint8_t ReadBlackMask(void)
{
    uint8_t mask = 0U;
    if (IsBlack(P1)) { mask |= (1U << 0); }
    if (IsBlack(P2)) { mask |= (1U << 1); }
    if (IsBlack(P3)) { mask |= (1U << 2); }
    if (IsBlack(P4)) { mask |= (1U << 3); }
    if (IsBlack(P5)) { mask |= (1U << 4); }
    if (IsBlack(P6)) { mask |= (1U << 5); }
    if (IsBlack(P7)) { mask |= (1U << 6); }
    if (IsBlack(P8)) { mask |= (1U << 7); }
    return mask;
}

static uint8_t CountBlack(uint8_t mask)
{
    uint8_t count = 0U;
    while (mask != 0U) {
        count += (uint8_t)(mask & 1U);
        mask >>= 1;
    }
    return count;
}

/* Negative = line on left; positive = line on right. */
static float GetLineError(uint8_t mask, uint8_t *valid)
{
    static const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    uint8_t i;
    uint8_t count = 0U;
    int sum = 0;

    for (i = 0U; i < 8U; i++) {
        if ((mask & (1U << i)) != 0U) {
            sum += weight[i];
            count++;
        }
    }

    if (count == 0U) {
        *valid = 0U;
        return gLastError;
    }

    *valid = 1U;
    return (float)sum / (float)count;
}

/* ======================== Simple PD tracking ========================== */

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int ClampPwm(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 100) {
        return 100;
    }
    return value;
}

static int RoundPwm(float value)
{
    if (value <= 0.0f) {
        return 0;
    }
    return ClampPwm((int)(value + 0.5f));
}

static int SlewPwm(int current, int target, int maximumChange)
{
    if (target > (current + maximumChange)) {
        return current + maximumChange;
    }
    if (target < (current - maximumChange)) {
        return current - maximumChange;
    }
    return target;
}

static float SlewFloat(float current, float target, float maximumChange)
{
    if (target > (current + maximumChange)) {
        return current + maximumChange;
    }
    if (target < (current - maximumChange)) {
        return current - maximumChange;
    }
    return target;
}

static void ApplyMotorCommand(int left, int right)
{
    gLastLeftPwm = ClampPwm(left);
    gLastRightPwm = ClampPwm(right);
    TB6612_SetMotor(gLastLeftPwm, gLastRightPwm);
}

static void StopMotorCommand(void)
{
    gLastLeftPwm = 0;
    gLastRightPwm = 0;
    TB6612_Stop();
}

static int ScaleBalancePwm(int pwm)
{
    uint32_t runTime;
    uint32_t percent = 100U;
    uint32_t rampRange = 100U - ITEM45_RAMP_MIN_PERCENT;
    float u;
    float smoothStep;

    if ((gMode != CAR_TEST_ITEM_4) &&
        (gMode != CAR_TEST_ITEM_5) &&
        (gMode != CAR_TEST_ITEM_6)) {
        return pwm;
    }

    runTime = gTick10ms - gRunStartTick;

    /* No pre-tilt wait: rod compensation and wheel ramp both start at S1. */
    if (runTime < ITEM45_START_RAMP_10MS) {
        u = (float)runTime /
            (float)ITEM45_START_RAMP_10MS;
        smoothStep = 3.0f * u * u -
            2.0f * u * u * u;
        percent = ITEM45_RAMP_MIN_PERCENT +
            (uint32_t)((float)rampRange * smoothStep);
        if ((runTime > 0U) && (percent == 0U)) {
            percent = 1U;
        }
    }

    return (int)((uint32_t)pwm * percent / 100U);
}

static void ApplyFollowMotorCommand(int left, int right)
{
    ApplyMotorCommand(
        ScaleBalancePwm(ClampPwm(left)),
        ScaleBalancePwm(ClampPwm(right)));
}

static float CalculateNormalizedWheelDifference(void)
{
    float left = (float)Encoder_GetLeftSpeed() /
                 (float)BALANCE_LEFT_PWM;
    float right = (float)Encoder_GetRightSpeed() /
                  (float)BALANCE_RIGHT_PWM;
    float average = (AbsFloat(left) + AbsFloat(right)) * 0.5f;

    if (average < 0.10f) {
        return 0.0f;
    }

    return AbsFloat(left - right) / average;
}

static void ResetItem5RoadMode(void)
{
    gItem5CurveActive = 0U;
    gItem5CurveEnterTicks = 0U;
    gItem5CurveExitTicks = 0U;
    gItem5CurveErrorFiltered = 0.0f;
    gItem5CurveTurnFiltered = 0.0f;
    gItem5CurveWheelDiffFiltered = 0.0f;
    gItem5BaseLeftPwm = (float)BALANCE_LEFT_PWM;
    gItem5BaseRightPwm = (float)BALANCE_RIGHT_PWM;
}

static void UpdateItem5RoadMode(float error, float turn)
{
    float wheelDifference = CalculateNormalizedWheelDifference();

    gItem5CurveErrorFiltered += ITEM5_CURVE_FILTER_ALPHA *
        (AbsFloat(error) - gItem5CurveErrorFiltered);
    gItem5CurveTurnFiltered += ITEM5_CURVE_FILTER_ALPHA *
        (AbsFloat(turn) - gItem5CurveTurnFiltered);
    gItem5CurveWheelDiffFiltered += ITEM5_CURVE_FILTER_ALPHA *
        (wheelDifference - gItem5CurveWheelDiffFiltered);

    if (gItem5CurveActive == 0U) {
        uint8_t curveEvidence =
            ((gItem5CurveErrorFiltered >= ITEM5_CURVE_ENTER_ERROR) ||
             (gItem5CurveTurnFiltered >= ITEM5_CURVE_ENTER_TURN) ||
             (gItem5CurveWheelDiffFiltered >=
              ITEM5_CURVE_ENTER_WHEEL_DIFF)) ? 1U : 0U;

        gItem5CurveExitTicks = 0U;
        if (curveEvidence != 0U) {
            if (gItem5CurveEnterTicks <
                ITEM5_CURVE_ENTER_CONFIRM_10MS) {
                gItem5CurveEnterTicks++;
            }
            if (gItem5CurveEnterTicks >=
                ITEM5_CURVE_ENTER_CONFIRM_10MS) {
                gItem5CurveActive = 1U;
                gItem5CurveEnterTicks = 0U;
            }
        } else {
            gItem5CurveEnterTicks = 0U;
        }
    } else {
        uint8_t straightEvidence =
            ((gItem5CurveErrorFiltered <= ITEM5_CURVE_EXIT_ERROR) &&
             (gItem5CurveTurnFiltered <= ITEM5_CURVE_EXIT_TURN) &&
             (gItem5CurveWheelDiffFiltered <=
              ITEM5_CURVE_EXIT_WHEEL_DIFF)) ? 1U : 0U;

        gItem5CurveEnterTicks = 0U;
        if (straightEvidence != 0U) {
            if (gItem5CurveExitTicks <
                ITEM5_CURVE_EXIT_CONFIRM_10MS) {
                gItem5CurveExitTicks++;
            }
            if (gItem5CurveExitTicks >=
                ITEM5_CURVE_EXIT_CONFIRM_10MS) {
                gItem5CurveActive = 0U;
                gItem5CurveExitTicks = 0U;
            }
        } else {
            gItem5CurveExitTicks = 0U;
        }
    }

    gItem5BaseLeftPwm = SlewFloat(
        gItem5BaseLeftPwm,
        (gItem5CurveActive != 0U) ?
            (float)BALANCE_CURVE_LEFT_PWM :
            (float)BALANCE_LEFT_PWM,
        ITEM5_BASE_SLEW_PWM_10MS);
    gItem5BaseRightPwm = SlewFloat(
        gItem5BaseRightPwm,
        (gItem5CurveActive != 0U) ?
            (float)BALANCE_CURVE_RIGHT_PWM :
            (float)BALANCE_RIGHT_PWM,
        ITEM5_BASE_SLEW_PWM_10MS);
}

static float ClampTurn(float value)
{
    if (value > MAX_STEERING_PWM) {
        return MAX_STEERING_PWM;
    }
    if (value < -MAX_STEERING_PWM) {
        return -MAX_STEERING_PWM;
    }
    return value;
}

static void UpdateItem5SoftStop(uint8_t mask, uint32_t stateTime)
{
    uint8_t valid;
    float error = GetLineError(mask, &valid);
    float stopTurn = gItem5StopStartTurn;
    float turnDelta = 0.0f;
    float u;
    float smoothStep;
    float scale;
    int targetLeft;
    int targetRight;

    if (stateTime >= ITEM5_STOP_RAMP_10MS) {
        ApplyMotorCommand(0, 0);
        return;
    }

    u = (float)stateTime / (float)ITEM5_STOP_RAMP_10MS;
    smoothStep = 3.0f * u * u - 2.0f * u * u * u;
    scale = 1.0f - smoothStep;

    /*
     * Preserve a small amount of line correction while decelerating.  The
     * correction is relative to the steering already present when A was
     * confirmed, so the first stop command cannot jump back to a new base PWM.
     */
    if (valid != 0U) {
        stopTurn = STEERING_SIGN *
            (LINE_KP * error + LINE_KD * (error - gLastError));
        stopTurn = ClampTurn(stopTurn);

        if (stopTurn > (gLastTurn + MAX_TURN_CHANGE_10MS)) {
            stopTurn = gLastTurn + MAX_TURN_CHANGE_10MS;
        } else if (stopTurn < (gLastTurn - MAX_TURN_CHANGE_10MS)) {
            stopTurn = gLastTurn - MAX_TURN_CHANGE_10MS;
        }

        turnDelta = stopTurn - gItem5StopStartTurn;
        if (turnDelta > ITEM5_STOP_STEERING_DELTA_MAX) {
            turnDelta = ITEM5_STOP_STEERING_DELTA_MAX;
        } else if (turnDelta < -ITEM5_STOP_STEERING_DELTA_MAX) {
            turnDelta = -ITEM5_STOP_STEERING_DELTA_MAX;
        }

        gLastTurn = stopTurn;
        gLastError = error;
    }

    targetLeft = RoundPwm(
        ((float)gItem5StopStartLeftPwm + turnDelta) * scale);
    targetRight = RoundPwm(
        ((float)gItem5StopStartRightPwm - turnDelta) * scale);

    targetLeft = SlewPwm(
        gLastLeftPwm, targetLeft, ITEM5_STOP_PWM_CHANGE_10MS);
    targetRight = SlewPwm(
        gLastRightPwm, targetRight, ITEM5_STOP_PWM_CHANGE_10MS);
    ApplyMotorCommand(targetLeft, targetRight);
}

static void SelectSpeed(CarTestMode mode)
{
    if (mode == CAR_TEST_ITEM_2) {
        gStraightLeft = FAST_LEFT_PWM;
        gStraightRight = FAST_RIGHT_PWM;
        gCurveLeft = FAST_CURVE_LEFT_PWM;
        gCurveRight = FAST_CURVE_RIGHT_PWM;
    } else {
        gStraightLeft = BALANCE_LEFT_PWM;
        gStraightRight = BALANCE_RIGHT_PWM;
        gCurveLeft = BALANCE_CURVE_LEFT_PWM;
        gCurveRight = BALANCE_CURVE_RIGHT_PWM;
    }
}

static void SetFault(void);

static void FollowLine(uint8_t mask)
{
    uint8_t valid;
    float error = GetLineError(mask, &valid);
    float turn;
    int base_left;
    int base_right;

    if (valid == 0U) {
        gLostTicks++;

        if (gLostTicks > LOST_LINE_TIMEOUT_10MS) {
            SetFault();
        } else if (gLastError > 0.0f) {
            ApplyMotorCommand(LOST_FAST_PWM, LOST_SLOW_PWM);
        } else if (gLastError < 0.0f) {
            ApplyMotorCommand(LOST_SLOW_PWM, LOST_FAST_PWM);
        } else {
            StopMotorCommand();
        }
        return;
    }

    gLostTicks = 0U;

    turn = STEERING_SIGN *
           (LINE_KP * error + LINE_KD * (error - gLastError));
    turn = ClampTurn(turn);

    /*
     * 数字灰度在相邻探头之间跳变时，限制每10 ms的转向变化量，
     * 避免左右PWM瞬间反复跳动造成圆弧抖动。
     */
    if (turn > (gLastTurn + MAX_TURN_CHANGE_10MS)) {
        turn = gLastTurn + MAX_TURN_CHANGE_10MS;
    } else if (turn < (gLastTurn - MAX_TURN_CHANGE_10MS)) {
        turn = gLastTurn - MAX_TURN_CHANGE_10MS;
    }

    if ((gMode == CAR_TEST_ITEM_4) ||
        (gMode == CAR_TEST_ITEM_5) ||
        (gMode == CAR_TEST_ITEM_6)) {
        UpdateItem5RoadMode(error, turn);
        base_left = RoundPwm(gItem5BaseLeftPwm);
        base_right = RoundPwm(gItem5BaseRightPwm);
    } else if (AbsFloat(error) >= CURVE_ERROR) {
        base_left = gCurveLeft;
        base_right = gCurveRight;
    } else {
        base_left = gStraightLeft;
        base_right = gStraightRight;
    }

    ApplyFollowMotorCommand(
        base_left + (int)turn,
        base_right - (int)turn);

    gLastTurn = turn;
    gLastError = error;
}

/* ======================== Task state machine =========================== */

static void SetFinished(void)
{
    StopMotorCommand();
    gRunning = 0U;
    gFinished = 1U;
    gFault = 0U;
    gState = CAR_STATE_FINISHED;
    gRunStopTick = gTick10ms;
}

static void SetFault(void)
{
    StopMotorCommand();
    TB6612_Disable();
    gRunning = 0U;
    gFinished = 0U;
    gFault = 1U;
    gState = CAR_STATE_FAULT;
    gRunStopTick = gTick10ms;
}

static uint32_t AdvanceAfterMarker(void)
{
    uint32_t left = Encoder_GetLeftTotal() - gMarkerLeftTotal;
    uint32_t right = Encoder_GetRightTotal() - gMarkerRightTotal;
    return (left + right) / 2U;
}

static void ConfirmMarker(uint32_t now)
{
    gCompletedLaps = 1U;
    gMarkerLeftTotal = Encoder_GetLeftTotal();
    gMarkerRightTotal = Encoder_GetRightTotal();

    if ((gMode == CAR_TEST_ITEM_2) &&
        (FINISH_ADVANCE_ENCODER_COUNTS == 0U)) {
        SetFinished();
    } else if (gMode == CAR_TEST_ITEM_4) {
        gItem5StopStartLeftPwm = gLastLeftPwm;
        gItem5StopStartRightPwm = gLastRightPwm;
        gItem5StopStartTurn = gLastTurn;
        gState = CAR_STATE_FINISH_ADVANCE;
        gStateStartTick = now;
    } else if ((gMode == CAR_TEST_ITEM_5) ||
               (gMode == CAR_TEST_ITEM_6)) {
        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_PIN);
        gState = CAR_STATE_FINISH_DELAY;
        gStateStartTick = now;
    } else {
        gState = CAR_STATE_FINISH_ADVANCE;
        gStateStartTick = now;
    }
}

void CarControl_Init(void)
{
    TB6612_Init();
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_PIN);
    gLastLeftPwm = 0;
    gLastRightPwm = 0;
    ResetItem5RoadMode();
    gBlackMask = ReadBlackMask();
}

void CarControl_Start(CarTestMode mode)
{
    if ((mode != CAR_TEST_ITEM_2) &&
        (mode != CAR_TEST_ITEM_4) &&
        (mode != CAR_TEST_ITEM_5) &&
        (mode != CAR_TEST_ITEM_6)) {
        mode = CAR_TEST_ITEM_2;
    }

    gMode = mode;
    SelectSpeed(mode);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_PIN);

    gRunning = 1U;
    gFinished = 0U;
    gFault = 0U;
    gCompletedLaps = 0U;
    gLeaveTicks = 0U;
    gMarkerTicks = 0U;
    gLostTicks = 0U;
    gItem4FinishArmed = 0U;
    gItem4ClearTicks = 0U;
    gLastError = 0.0f;
    gLastTurn = 0.0f;
    gLastLeftPwm = 0;
    gLastRightPwm = 0;
    gItem5StopStartLeftPwm = 0;
    gItem5StopStartRightPwm = 0;
    gItem5StopStartTurn = 0.0f;
    ResetItem5RoadMode();

    gRunStartTick = gTick10ms;
    gRunStopTick = gTick10ms;
    gLastUpdateTick = gTick10ms - 1U;
    gState = CAR_STATE_WAIT_LINE;
    TB6612_Enable();
}

void CarControl_Abort(void)
{
    if (gRunning != 0U) {
        SetFault();
    }
}

void CarControl_Update(void)
{
    uint32_t now = gTick10ms;
    uint32_t run_time;
    uint8_t black_count;

    if ((uint32_t)(now - gLastUpdateTick) < 1U) {
        return;
    }
    gLastUpdateTick = now;

    if (gRunning == 0U) {
        StopMotorCommand();
        return;
    }

    run_time = now - gRunStartTick;

    if (((gMode == CAR_TEST_ITEM_2) &&
         (run_time > ITEM2_TIMEOUT_10MS)) ||
        ((gMode != CAR_TEST_ITEM_2) &&
         (run_time > BALANCE_TIMEOUT_10MS))) {
        SetFault();
        return;
    }

    gBlackMask = ReadBlackMask();
    black_count = CountBlack(gBlackMask);

    if (gMode == CAR_TEST_ITEM_4) {
        uint32_t item4LedEnd =
            ITEM4_B_LED_START_10MS + ITEM4_LED_ON_TIME_10MS;

        if ((run_time >= ITEM4_B_LED_START_10MS) &&
            (run_time < item4LedEnd)) {
            DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_PIN);
        }

        /*
         * Ignore B even if it is a wide black marker.  Only after the timed
         * B indication has ended and the sensors have seen normal line again
         * is the later three-black finish marker allowed to stop Task 4.
         */
        if ((gItem4FinishArmed == 0U) &&
            (run_time >= item4LedEnd)) {
            if (black_count < A_MARKER_BLACK_COUNT) {
                if (gItem4ClearTicks < ITEM4_FINISH_CLEAR_10MS) {
                    gItem4ClearTicks++;
                }
                if (gItem4ClearTicks >= ITEM4_FINISH_CLEAR_10MS) {
                    gItem4FinishArmed = 1U;
                }
            } else {
                gItem4ClearTicks = 0U;
            }
        }
    }

    switch (gState) {
    case CAR_STATE_WAIT_LINE:
        if ((gBlackMask & ((1U << 3) | (1U << 4))) != 0U) {
            gState = CAR_STATE_LEAVE_START_MARKER;
            FollowLine(gBlackMask);
        } else {
            StopMotorCommand();
        }
        break;

    case CAR_STATE_LEAVE_START_MARKER:
        FollowLine(gBlackMask);
        if (gRunning == 0U) {
            break;
        }
        if (black_count <= LEAVE_A_MAX_BLACK_COUNT) {
            if (++gLeaveTicks >= LEAVE_A_CONFIRM_10MS) {
                gState = CAR_STATE_TRACK;
            }
        } else {
            gLeaveTicks = 0U;
        }
        break;

    case CAR_STATE_TRACK:
        if ((run_time >= MIN_LAP_TIME_10MS) &&
            ((gMode != CAR_TEST_ITEM_4) ||
             (gItem4FinishArmed != 0U)) &&
            (black_count >= A_MARKER_BLACK_COUNT)) {
            if ((gMode == CAR_TEST_ITEM_4) ||
                (gMode == CAR_TEST_ITEM_5) ||
                (gMode == CAR_TEST_ITEM_6)) {
                /* Keep the current line-following PWM until A is confirmed. */
                FollowLine(gBlackMask);
            } else {
                ApplyMotorCommand(18, 20);
            }
            if (++gMarkerTicks >= A_MARKER_CONFIRM_10MS) {
                gMarkerTicks = 0U;
                ConfirmMarker(now);
            }
        } else {
            gMarkerTicks = 0U;
            FollowLine(gBlackMask);
        }
        break;

    case CAR_STATE_FINISH_DELAY:
    {
        uint32_t delayTime = now - gStateStartTick;
        uint32_t stopDelay = ITEM6_FINISH_DELAY_10MS;

        FollowLine(gBlackMask);
        if (gRunning == 0U) {
            break;
        }

        if (gMode == CAR_TEST_ITEM_5) {
            stopDelay = ITEM5_FINISH_DELAY_10MS;
            if (delayTime >= ITEM5_LED_ON_TIME_10MS) {
                DL_GPIO_clearPins(
                    GPIO_LED_PORT, GPIO_LED_PIN_LED_PIN);
            }
        }

        if (delayTime >= stopDelay) {
            gItem5StopStartLeftPwm = gLastLeftPwm;
            gItem5StopStartRightPwm = gLastRightPwm;
            gItem5StopStartTurn = gLastTurn;
            gState = CAR_STATE_FINISH_ADVANCE;
            gStateStartTick = now;
        }
        break;
    }

    case CAR_STATE_FINISH_ADVANCE:
        if ((gMode == CAR_TEST_ITEM_4) ||
            (gMode == CAR_TEST_ITEM_5) ||
            (gMode == CAR_TEST_ITEM_6)) {
            uint32_t stopTime = now - gStateStartTick;

            UpdateItem5SoftStop(gBlackMask, stopTime);
            if (stopTime >= ITEM5_STOP_RAMP_10MS) {
                SetFinished();
            }
        } else {
            FollowLine(gBlackMask);
            if (gRunning == 0U) {
                break;
            }
        }

        if (gMode == CAR_TEST_ITEM_2) {
            if (AdvanceAfterMarker() >=
                FINISH_ADVANCE_ENCODER_COUNTS) {
                SetFinished();
            } else if ((uint32_t)(now - gStateStartTick) >=
                       FINISH_ADVANCE_TIMEOUT_10MS) {
                SetFault();
            }
        } else if ((gMode != CAR_TEST_ITEM_4) &&
                   (gMode != CAR_TEST_ITEM_5) &&
                   (gMode != CAR_TEST_ITEM_6) &&
                   ((uint32_t)(now - gStateStartTick) >=
                    PASS_A_AFTER_MARKER_10MS)) {
            SetFinished();
        }
        break;

    default:
        SetFault();
        break;
    }
}

/* ======================== Public data ================================= */

uint8_t CarControl_IsRunning(void)       { return gRunning; }
uint8_t CarControl_IsFinished(void)      { return gFinished; }
uint8_t CarControl_HasFault(void)        { return gFault; }
uint8_t CarControl_IsSoftStopping(void)
{
    if (gRunning == 0U) {
        return 0U;
    }

    if ((gMode == CAR_TEST_ITEM_4) ||
        (gMode == CAR_TEST_ITEM_5) ||
        (gMode == CAR_TEST_ITEM_6)) {
        return (uint8_t)(
            gState == CAR_STATE_FINISH_ADVANCE);
    }

    return 0U;
}
uint16_t CarControl_GetSoftStopProgressPermille(void)
{
    uint32_t elapsed;
    uint32_t duration;

    if (gRunning == 0U) {
        return 0U;
    }

    if (((gMode == CAR_TEST_ITEM_4) ||
         (gMode == CAR_TEST_ITEM_5) ||
         (gMode == CAR_TEST_ITEM_6)) &&
        (gState == CAR_STATE_FINISH_ADVANCE)) {
        elapsed = gTick10ms - gStateStartTick;
        duration = ITEM5_STOP_RAMP_10MS;
    } else {
        return 0U;
    }

    if (elapsed >= duration) {
        return 1000U;
    }
    return (uint16_t)(elapsed * 1000U / duration);
}
uint8_t CarControl_GetCompletedLaps(void){ return gCompletedLaps; }
CarTestMode CarControl_GetTestMode(void) { return gMode; }
uint8_t CarControl_GetBlackMask(void)    { return gBlackMask; }
uint32_t CarControl_GetLeftEncoderSpeed(void)
{
    return Encoder_GetLeftSpeed();
}
uint32_t CarControl_GetRightEncoderSpeed(void)
{
    return Encoder_GetRightSpeed();
}
uint32_t CarControl_GetElapsed10ms(void)
{
    uint32_t end = (gRunning != 0U) ? gTick10ms : gRunStopTick;
    return end - gRunStartTick;
}
uint32_t CarControl_GetTick10ms(void)    { return gTick10ms; }
CarControlState CarControl_GetState(void){ return gState; }

uint8_t CarControl_GetSensorLevelMask(void)
{
    uint8_t mask = 0U;
    if (P1 != 0U) { mask |= (1U << 0); }
    if (P2 != 0U) { mask |= (1U << 1); }
    if (P3 != 0U) { mask |= (1U << 2); }
    if (P4 != 0U) { mask |= (1U << 3); }
    if (P5 != 0U) { mask |= (1U << 4); }
    if (P6 != 0U) { mask |= (1U << 5); }
    if (P7 != 0U) { mask |= (1U << 6); }
    if (P8 != 0U) { mask |= (1U << 7); }
    return mask;
}

void CarControl_On10msTick(void)
{
    gTick10ms++;
}
