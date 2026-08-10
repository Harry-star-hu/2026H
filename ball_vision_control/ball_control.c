#include "ball_control.h"
#include "ball_config.h"
#include "ball_vision.h"
#include "rod_stepper.h"

#define STATIC_PULSE_DRIVE 0U
#define STATIC_PULSE_COAST 1U

#define TASK3_PHASE_OPEN_TO_PLUS    0U
#define TASK3_PHASE_CLOSED_TO_MINUS 1U

#define ACTIVE_HOLD_PARAMETER(normalValue, item6Value) \
    ((gHoldUseItem6Parameters != 0U) ? (item6Value) : (normalValue))

static BallControlState gState;
static BallFault gFault;
static uint8_t gHasZero;
static uint32_t gLastControlMs;

static float gBallPositionCm;
static float gBallTargetCm;
static float gBallSpeedCmS;
static float gBallHoldSpeedCmS;
static float gBallRawSpeedCmS;
static float gLastFramePositionCm;
static uint16_t gLastFrameSequence;
static uint32_t gLastFrameMs;
static uint8_t gHavePreviousFrame;
static uint8_t gBallOutsideFrames;

static float gRodAngleDeg;
static float gLastRodAngleDeg;
static float gRodRateDegS;
static float gRodTargetCommandDeg;
static float gRodTargetDeg;
static float gRodIntegral;

static uint32_t gStaticStableMs;
static uint8_t gStaticPulseState;
static uint32_t gStaticPulseMs;
static float gStaticKickDeg;
static float gStaticPulseStartDistanceCm;
static uint8_t gStaticMotionConfirmed;
static uint32_t gStaticNoProgressMs;
static uint8_t gStaticFinalCapture;
static uint32_t gStaticCaptureStallMs;
static uint8_t gStaticFinalNudge;
static uint32_t gStaticFinalNudgeMs;

static uint8_t gTask3OpenPhase;
static float gTask3ClosedSpeedCmS;
static uint8_t gTask3ClosedCentered;
static uint8_t gTask3ClosedAccepted;
static uint8_t gTask3ClosedTargetCrossed;
static float gTask3ClosedIntegralDeg;
static float gTask3ClosedMotionReferenceCm;
static float gTask3ClosedLastCorrectionSign;
static uint32_t gTask3ClosedMotionWindowMs;
static uint8_t gTask3ClosedMotionReady;
static uint8_t gTask3ClosedStaticBoostActive;
static uint8_t gHoldCentered;
static uint8_t gHoldUseItem6Parameters;
static float gHoldFeedforwardDeg;
static float gHoldPositionIntegralDeg;
static float gHoldMotionReferencePositionCm;
static float gHoldLastCorrectionSign;
static uint32_t gHoldMotionWindowMs;
static uint8_t gHoldMotionReady;
static uint8_t gHoldStaticBoostActive;
static uint8_t gHoldStopCaptureActive;
static uint8_t gHoldStopVehicleStopped;
static uint32_t gHoldStopCaptureMs;
static uint32_t gHoldStopStableMs;

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float LimitFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float SignFloat(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

static void UpdateTask3ClosedPositionControl(void);

static void ResetStaticPulse(void)
{
    gStaticPulseState = STATIC_PULSE_DRIVE;
    gStaticPulseMs = 0U;
    gStaticKickDeg = STATIC_KICK_START_DEG;
    gStaticPulseStartDistanceCm =
        AbsFloat(gBallTargetCm - gBallPositionCm);
    gStaticMotionConfirmed = 0U;
    gStaticNoProgressMs = 0U;
}

static void ResetStaticFinalCapture(void)
{
    gStaticFinalCapture = 0U;
    gStaticCaptureStallMs = 0U;
    gStaticFinalNudge = 0U;
    gStaticFinalNudgeMs = 0U;
}

static void ResetTask3Controller(void)
{
    gTask3OpenPhase = TASK3_PHASE_OPEN_TO_PLUS;
    gTask3ClosedSpeedCmS = 0.0f;
    gTask3ClosedCentered = 0U;
    gTask3ClosedAccepted = 0U;
    gTask3ClosedTargetCrossed = 0U;
    gTask3ClosedIntegralDeg = 0.0f;
    gTask3ClosedMotionReferenceCm = gBallPositionCm;
    gTask3ClosedLastCorrectionSign = 1.0f;
    gTask3ClosedMotionWindowMs = 0U;
    gTask3ClosedMotionReady = 0U;
    gTask3ClosedStaticBoostActive = 0U;
}

static void ResetHoldStopCapture(void)
{
    gHoldStopCaptureActive = 0U;
    gHoldStopVehicleStopped = 0U;
    gHoldStopCaptureMs = 0U;
    gHoldStopStableMs = 0U;
}

static float GetStaticKickMaximum(float distance)
{
    return (distance <= STATIC_KICK_NEAR_DISTANCE_CM) ?
        STATIC_KICK_NEAR_MAX_DEG :
        STATIC_KICK_MAX_DEG;
}

static void RestartStaticPulseAfterStall(float distance)
{
    float kickMaximum = GetStaticKickMaximum(distance);

    gStaticPulseState = STATIC_PULSE_DRIVE;
    gStaticPulseMs = 0U;
    gStaticPulseStartDistanceCm = distance;
    gStaticMotionConfirmed = 0U;
    gStaticNoProgressMs = 0U;

    /*
     * Keep the force already learned for this run.  A re-stall means the
     * previous pulse was insufficient, so increase it instead of returning
     * to STATIC_KICK_START_DEG.
     */
    gStaticKickDeg += STATIC_KICK_STEP_DEG;
    if (gStaticKickDeg > kickMaximum) {
        gStaticKickDeg = kickMaximum;
    }
}

static void SetFault(BallFault fault)
{
    gFault = fault;
    gState = BALL_STATE_FAULT;
    gRodTargetCommandDeg = 0.0f;
    gRodIntegral = 0.0f;
    ResetHoldStopCapture();
}

static uint8_t StartCommon(void)
{
    if (gHasZero == 0U) {
        SetFault(BALL_FAULT_NO_ZERO);
        return 0U;
    }

    gFault = BALL_FAULT_NONE;
    gRodIntegral = 0.0f;
    gBallSpeedCmS = 0.0f;
    gBallHoldSpeedCmS = 0.0f;
    gBallRawSpeedCmS = 0.0f;
    gHavePreviousFrame = 0U;
    gBallOutsideFrames = 0U;
    gStaticStableMs = 0U;
    ResetStaticFinalCapture();
    ResetStaticPulse();
    ResetHoldStopCapture();
    RodStepper_Enable(1U);
    return 1U;
}

static uint8_t UpdateVisionMeasurement(uint32_t now_ms)
{
    BallVisionFrame frame;
    float position;
    float dt;
    float measuredSpeed;

    (void)BallVision_GetFrame(&frame);
    if ((frame.valid == 0U) ||
        (frame.sequence == gLastFrameSequence)) {
        return 0U;
    }

    position = (float)frame.x_tenth_mm / 100.0f;
    gBallPositionCm = position;
    gLastFrameSequence = frame.sequence;

    if ((gHavePreviousFrame != 0U) && (now_ms > gLastFrameMs)) {
        dt = (float)(now_ms - gLastFrameMs) / 1000.0f;
        measuredSpeed =
            (position - gLastFramePositionCm) / dt;
        gBallRawSpeedCmS = measuredSpeed;
        gBallSpeedCmS += BALL_SPEED_FILTER_ALPHA *
                         (measuredSpeed - gBallSpeedCmS);
        gBallHoldSpeedCmS +=
            ACTIVE_HOLD_PARAMETER(
                HOLD_SPEED_FAST_FILTER_ALPHA,
                ITEM6_HOLD_SPEED_FAST_FILTER_ALPHA) *
            (measuredSpeed - gBallHoldSpeedCmS);
        gTask3ClosedSpeedCmS +=
            TASK3_CLOSED_SPEED_FILTER_ALPHA *
            (measuredSpeed - gTask3ClosedSpeedCmS);
    } else {
        gBallSpeedCmS = 0.0f;
        gBallHoldSpeedCmS = 0.0f;
        gTask3ClosedSpeedCmS = 0.0f;
        gBallRawSpeedCmS = 0.0f;
        gHavePreviousFrame = 1U;
    }

    gLastFramePositionCm = position;
    gLastFrameMs = now_ms;
    return 1U;
}

static float CalculateEndpointCaptureDistance(float error)
{
    float towardTargetSpeed =
        SignFloat(error) * gBallSpeedCmS;
    float captureDistance;

    if (towardTargetSpeed < 0.0f) {
        towardTargetSpeed = 0.0f;
    }

    captureDistance =
        ENDPOINT_CAPTURE_BASE_CM +
        ENDPOINT_CAPTURE_SPEED_GAIN_S * towardTargetSpeed;

    return LimitFloat(
        captureDistance,
        ENDPOINT_CAPTURE_BASE_CM,
        ENDPOINT_CAPTURE_MAX_CM);
}

static void UpdateStaticSequence(void)
{
    float error = gBallTargetCm - gBallPositionCm;
    float distance = AbsFloat(error);
    float captureDistance =
        CalculateEndpointCaptureDistance(error);
    uint8_t captureStable;

    if (gState == BALL_STATE_STATIC_GO_PLUS) {
        /*
         * Dynamic capture slows the ball before +5.  The first actual target
         * crossing then starts the return immediately.
         */
        if (gBallPositionCm >= gBallTargetCm) {
            /*
             * Reaching +5 is an event, not a dwell condition.  Waiting for
             * camera-derived speed to remain below a threshold kept B at 3
             * indefinitely when coordinate jitter reset the stable timer.
             * Switch to B=4 in this same control cycle on the first crossing.
             */
            gState = BALL_STATE_STATIC_GO_MINUS;
            gBallTargetCm = -5.0f;
            gStaticStableMs = 0U;
            gRodTargetCommandDeg = 0.0f;
            gRodTargetDeg = 0.0f;
            gRodIntegral = 0.0f;
            ResetStaticFinalCapture();
            ResetStaticPulse();
            gStaticKickDeg = STATIC_RETURN_START_DEG;
            return;
        }

        if ((gStaticFinalCapture == 0U) &&
            (distance <= captureDistance)) {
            gStaticFinalCapture = 1U;
            gStaticCaptureStallMs = 0U;
            gStaticPulseStartDistanceCm = distance;
            gStaticKickDeg = STATIC_KICK_START_DEG;
            gRodTargetCommandDeg = 0.0f;
            gRodTargetDeg = 0.0f;
            gRodIntegral = 0.0f;
        }

        gStaticStableMs = 0U;
        return;
    }

    if (gState == BALL_STATE_STATIC_GO_MINUS) {
        if ((gStaticFinalCapture == 0U) &&
            ((distance <= captureDistance) ||
             (gBallPositionCm <= gBallTargetCm))) {
            gStaticFinalCapture = 1U;
            gStaticCaptureStallMs = 0U;
            gStaticPulseStartDistanceCm = distance;
            gStaticKickDeg = STATIC_RETURN_START_DEG;
            gRodTargetCommandDeg = 0.0f;
            gRodTargetDeg = 0.0f;
            gRodIntegral = 0.0f;
        }

        captureStable = (uint8_t)(
            (gStaticFinalCapture != 0U) &&
            (gStaticFinalNudge == 0U) &&
            (distance <= STATIC_FINISH_ERROR_CM) &&
            (AbsFloat(gBallSpeedCmS) <=
             STATIC_FINISH_SPEED_CM_S));

        if (captureStable != 0U) {
            gStaticStableMs += CONTROL_PERIOD_MS;
        } else {
            gStaticStableMs = 0U;
        }

        if (gStaticStableMs >= STATIC_FINISH_TIME_MS) {
            gState = BALL_STATE_STATIC_FINISHED;
            gStaticStableMs = 0U;
            ResetStaticFinalCapture();
            gRodTargetCommandDeg = 0.0f;
            gRodTargetDeg = 0.0f;
            gRodIntegral = 0.0f;
            ResetStaticPulse();
        }
    }
}

static float CalculateBrakeAngle(float speed)
{
    float angle =
        STATIC_BRAKE_BASE_ANGLE_DEG +
        STATIC_BRAKE_SPEED_GAIN * AbsFloat(speed);

    return LimitFloat(
        angle,
        STATIC_BRAKE_BASE_ANGLE_DEG,
        STATIC_BRAKE_MAX_ANGLE_DEG);
}

static void UpdateTask3SequenceState(void)
{
    float distance;
    float speedMagnitude;

    /* First +5 crossing permanently transfers Task 3 to closed-loop return. */
    if ((gTask3OpenPhase == TASK3_PHASE_OPEN_TO_PLUS) &&
        (gBallPositionCm >= TASK3_OPEN_SWITCH_CM)) {
        gTask3OpenPhase = TASK3_PHASE_CLOSED_TO_MINUS;
        gState = BALL_STATE_STATIC_GO_MINUS;
        gBallTargetCm = TASK3_CLOSED_TARGET_CM;
        gStaticStableMs = 0U;
        gRodTargetCommandDeg = 0.0f;
        gRodIntegral = 0.0f;
        gTask3ClosedCentered = 0U;
        gTask3ClosedAccepted = 0U;
        gTask3ClosedTargetCrossed = 0U;
        gTask3ClosedIntegralDeg = 0.0f;
        gTask3ClosedMotionReferenceCm = gBallPositionCm;
        gTask3ClosedLastCorrectionSign = 1.0f;
        gTask3ClosedMotionWindowMs = 0U;
        gTask3ClosedMotionReady = 0U;
        gTask3ClosedStaticBoostActive = 0U;
        return;
    }

    if ((gTask3OpenPhase != TASK3_PHASE_CLOSED_TO_MINUS) ||
        (gState != BALL_STATE_STATIC_GO_MINUS)) {
        return;
    }

    distance = AbsFloat(gBallPositionCm - gBallTargetCm);
    speedMagnitude = AbsFloat(gTask3ClosedSpeedCmS);

    if ((distance <= TASK3_CLOSED_FINISH_ERROR_CM) &&
        (speedMagnitude <= TASK3_CLOSED_FINISH_SPEED_CM_S)) {
        gStaticStableMs += CONTROL_PERIOD_MS;
    } else {
        gStaticStableMs = 0U;
    }

    if (gStaticStableMs >= TASK3_CLOSED_FINISH_TIME_MS) {
        gState = BALL_STATE_STATIC_FINISHED;
        gStaticStableMs = 0U;
    }
}

static void SetTask3RodDirect(float targetDeg)
{
    gRodTargetCommandDeg = targetDeg;
    gRodTargetDeg = targetDeg;
    gRodIntegral = 0.0f;
}

#if 0
/* Disabled: previous closed-loop Task 3 implementation. */
static void ResetTask3MotionProgress(void)
{
    gTask3BestDistanceCm =
        AbsFloat(gBallTargetCm - gBallPositionCm);
    gTask3NoProgressMs = 0U;
    gTask3MotionConfirmed = 0U;
    gTask3ErrorSign = SignFloat(
        gBallPositionCm - gBallTargetCm);
}

static void EnterTask3RightLevel(void)
{
    gTask3Phase = TASK3_PHASE_RIGHT_LEVEL;
    gTask3PhaseMs = 0U;
    gTask3LevelMs = 0U;
    if (gBallPositionCm > gTask3PeakPositionCm) {
        gTask3PeakPositionCm = gBallPositionCm;
    }
    SetTask3RodDirect(0.0f);
}

static void EnterTask3LeftDrive(void)
{
    gTask3Phase = TASK3_PHASE_LEFT_DRIVE;
    gTask3PhaseMs = 0U;
    gState = BALL_STATE_STATIC_GO_MINUS;
    gBallTargetCm = TASK3_MINUS_TARGET_CM;
    gStaticStableMs = 0U;
    ResetTask3MotionProgress();
    SetTask3RodDirect(0.0f);
}

static void EnterTask3FinalCapture(void)
{
    gTask3Phase = TASK3_PHASE_CAPTURE;
    gTask3PhaseMs = 0U;
    gTask3CaptureKickActive = 0U;
    gTask3CaptureKickMs = 0U;
    ResetTask3MotionProgress();
    SetTask3RodDirect(0.0f);
}

static void UpdateTask3MotionProgress(float distance)
{
    if ((gTask3BestDistanceCm - distance) >=
        TASK3_PROGRESS_CONFIRM_CM) {
        gTask3BestDistanceCm = distance;
        gTask3NoProgressMs = 0U;
        gTask3MotionConfirmed = 1U;
    } else if (AbsFloat(gBallSpeedCmS) <=
               TASK3_CAPTURE_STILL_SPEED_CM_S) {
        gTask3NoProgressMs += CONTROL_PERIOD_MS;
        if (gTask3NoProgressMs >=
            TASK3_NO_PROGRESS_MS) {
            gTask3MotionConfirmed = 0U;
        }
    } else {
        gTask3NoProgressMs = 0U;
    }
}

static void UpdateTask3FinalCaptureController(void)
{
    float positionError =
        gBallPositionCm - gBallTargetCm;
    float distance = AbsFloat(positionError);
    float speedMagnitude = AbsFloat(gBallSpeedCmS);
    float errorSign = SignFloat(positionError);
    float targetDirection =
        errorSign * BALL_OUTER_LOOP_SIGN;
    float progress =
        gTask3BestDistanceCm - distance;
    float command;

    /* Crossing -5 starts a fresh, low-energy correction approach. */
    if (errorSign != gTask3ErrorSign) {
        gTask3ErrorSign = errorSign;
        gTask3BestDistanceCm = distance;
        gTask3NoProgressMs = 0U;
        gTask3MotionConfirmed = 0U;
        gTask3CaptureKickActive = 0U;
        gTask3CaptureKickMs = 0U;
        progress = 0.0f;
    }

    if (gTask3CaptureKickActive != 0U) {
        /* End the nudge immediately when the ball really starts moving. */
        if ((progress >= TASK3_PROGRESS_CONFIRM_CM) ||
            (speedMagnitude >
             TASK3_CAPTURE_STILL_SPEED_CM_S)) {
            gTask3CaptureKickActive = 0U;
            gTask3CaptureKickMs = 0U;
            gTask3BestDistanceCm = distance;
            gTask3NoProgressMs = 0U;
        } else if (gTask3CaptureKickMs <
                   TASK3_CAPTURE_KICK_TIME_MS) {
            SetTask3RodDirect(
                targetDirection *
                TASK3_CAPTURE_KICK_DEG);
            gTask3CaptureKickMs +=
                CONTROL_PERIOD_MS;
            return;
        } else {
            gTask3CaptureKickActive = 0U;
            gTask3CaptureKickMs = 0U;
            gTask3NoProgressMs = 0U;
            SetTask3RodDirect(0.0f);
            return;
        }
    }

    progress = gTask3BestDistanceCm - distance;
    if (progress >= TASK3_PROGRESS_CONFIRM_CM) {
        gTask3BestDistanceCm = distance;
        gTask3NoProgressMs = 0U;
        gTask3MotionConfirmed = 1U;
    } else if (speedMagnitude <=
               TASK3_CAPTURE_STILL_SPEED_CM_S) {
        gTask3NoProgressMs += CONTROL_PERIOD_MS;
    } else {
        /* Never start a static-friction nudge while the ball is moving. */
        gTask3NoProgressMs = 0U;
    }

    command =
        BALL_OUTER_LOOP_SIGN *
        (TASK3_CAPTURE_KP * positionError +
         TASK3_CAPTURE_KD * gBallSpeedCmS);
    command = LimitFloat(
        command,
        -TASK3_CAPTURE_MAX_ANGLE_DEG,
        TASK3_CAPTURE_MAX_ANGLE_DEG);

    if ((distance <= TASK3_FINISH_ERROR_CM) &&
        (speedMagnitude <=
         TASK3_FINISH_SPEED_CM_S)) {
        SetTask3RodDirect(0.0f);
        gTask3NoProgressMs = 0U;
        return;
    }

    if ((distance > TASK3_FINISH_ERROR_CM) &&
        (speedMagnitude <=
         TASK3_CAPTURE_STILL_SPEED_CM_S) &&
        (gTask3NoProgressMs >=
         TASK3_NO_PROGRESS_MS)) {
        gTask3CaptureKickActive = 1U;
        gTask3CaptureKickMs = 0U;
        gTask3NoProgressMs = 0U;
        SetTask3RodDirect(
            targetDirection *
            TASK3_CAPTURE_KICK_DEG);
        return;
    }

    /* The capture command is small, so apply it without the old slew delay. */
    SetTask3RodDirect(command);
}

static void UpdateTask3SingleController(void)
{
    float positionError =
        gBallPositionCm - gBallTargetCm;
    float distance = AbsFloat(positionError);
    float targetDirection =
        SignFloat(positionError) *
        BALL_OUTER_LOOP_SIGN;
    float towardTargetSpeed =
        -SignFloat(positionError) *
        gBallSpeedCmS;
    float driveAngle;

    if ((gTask3Phase == TASK3_PHASE_CAPTURE) ||
        (gState == BALL_STATE_STATIC_FINISHED)) {
        UpdateTask3FinalCaptureController();
        return;
    }

    if (gTask3Phase == TASK3_PHASE_RIGHT_DRIVE) {
        if (gBallPositionCm >=
            TASK3_RIGHT_LEVEL_POSITION_CM) {
            EnterTask3RightLevel();
            return;
        }

        UpdateTask3MotionProgress(distance);
        if (gTask3MotionConfirmed == 0U) {
            driveAngle = TASK3_RIGHT_BREAKAWAY_DEG;
        } else {
            driveAngle = TASK3_RIGHT_RUN_DEG;
        }

        gRodTargetCommandDeg =
            targetDirection * driveAngle;
        if (gTask3MotionConfirmed == 0U) {
            SetTask3RodDirect(gRodTargetCommandDeg);
        }
        return;
    }

    if (gTask3Phase == TASK3_PHASE_RIGHT_LEVEL) {
        SetTask3RodDirect(0.0f);
        gTask3PhaseMs += CONTROL_PERIOD_MS;

        if (gBallPositionCm > gTask3PeakPositionCm) {
            gTask3PeakPositionCm = gBallPositionCm;
        }

        if (AbsFloat(gRodAngleDeg) <=
            TASK3_LEVEL_ANGLE_TOL_DEG) {
            gTask3LevelMs += CONTROL_PERIOD_MS;
        }

        /*
         * Once the rod is truly level, let the ball coast briefly.  Reaching
         * +4.8 cm is enough: the next phase is permanently latched left and
         * can never request another rightward drive.
         */
        if ((gTask3LevelMs >=
             TASK3_LEVEL_MIN_TIME_MS) &&
            (gTask3PeakPositionCm >=
             TASK3_RIGHT_REACHED_CM)) {
            EnterTask3LeftDrive();
            return;
        }

        if ((gTask3LevelMs >=
             TASK3_LEVEL_MAX_TIME_MS) ||
            (gTask3PhaseMs >=
             TASK3_LEVEL_ROD_TIMEOUT_MS)) {
            if (gTask3RightNudgeUsed == 0U) {
                gTask3RightNudgeUsed = 1U;
                gTask3Phase =
                    TASK3_PHASE_RIGHT_NUDGE;
                gTask3PhaseMs = 0U;
            } else {
                EnterTask3LeftDrive();
            }
            return;
        }
        return;
    }

    if (gTask3Phase == TASK3_PHASE_RIGHT_NUDGE) {
        gTask3PhaseMs += CONTROL_PERIOD_MS;
        SetTask3RodDirect(
            -BALL_OUTER_LOOP_SIGN *
            TASK3_RIGHT_NUDGE_DEG);

        if ((gBallPositionCm >=
             TASK3_RIGHT_REACHED_CM) ||
            (gTask3PhaseMs >=
             TASK3_RIGHT_NUDGE_TIME_MS)) {
            EnterTask3RightLevel();
        }
        return;
    }

    if (gTask3Phase == TASK3_PHASE_LEFT_DRIVE) {
        if (gBallPositionCm <=
            TASK3_FINAL_CAPTURE_POSITION_CM) {
            EnterTask3FinalCapture();
            return;
        }

        UpdateTask3MotionProgress(distance);
        if ((gTask3MotionConfirmed != 0U) &&
            (towardTargetSpeed >=
             TASK3_LEFT_MAX_SPEED_CM_S)) {
            SetTask3RodDirect(0.0f);
            return;
        }

        driveAngle =
            (gTask3MotionConfirmed == 0U) ?
            TASK3_LEFT_BREAKAWAY_DEG :
            TASK3_LEFT_RUN_DEG;
        gRodTargetCommandDeg =
            targetDirection * driveAngle;
        if (gTask3MotionConfirmed == 0U) {
            SetTask3RodDirect(gRodTargetCommandDeg);
        }
        return;
    }

    SetTask3RodDirect(0.0f);
}

#endif

static void UpdateTask3SingleController(void)
{
    if (gTask3OpenPhase == TASK3_PHASE_OPEN_TO_PLUS) {
        SetTask3RodDirect(
            -BALL_OUTER_LOOP_SIGN *
            TASK3_OPEN_TILT_DEG);
        return;
    }

    /*
     * Closed loop remains active after the finish flag, so the ball keeps
     * holding -5 cm instead of releasing the pipe back to horizontal.
     */
    UpdateTask3ClosedPositionControl();
}

static uint8_t UpdateStaticFinalCaptureControl(void)
{
    float positionError = gBallPositionCm - gBallTargetCm;
    float distance =
        AbsFloat(gBallTargetCm - gBallPositionCm);
    float speedMagnitude = AbsFloat(gBallSpeedCmS);
    float progress =
        gStaticPulseStartDistanceCm - distance;
    float targetDirection =
        -SignFloat(gBallTargetCm - gBallPositionCm) *
        BALL_OUTER_LOOP_SIGN;
    float captureCommand;
    float captureLimit;

    /*
     * If friction stopped the ball just before an endpoint, apply one complete
     * near-target pulse in the target direction.  This state remains inside
     * capture, so it cannot be cancelled on the next control cycle.
     */
    if (gStaticFinalNudge != 0U) {
        gRodTargetCommandDeg =
            targetDirection * gStaticKickDeg;
        gStaticFinalNudgeMs += CONTROL_PERIOD_MS;

        if (gStaticFinalNudgeMs >=
            STATIC_KICK_NEAR_TIME_MS) {
            gStaticFinalNudge = 0U;
            gStaticFinalNudgeMs = 0U;
            gStaticCaptureStallMs = 0U;
            gStaticPulseStartDistanceCm = distance;
            gRodTargetCommandDeg = 0.0f;
            gRodTargetDeg = 0.0f;
            gRodIntegral = 0.0f;
            gStaticKickDeg += STATIC_KICK_STEP_DEG;
            if (gStaticKickDeg >
                STATIC_KICK_NEAR_MAX_DEG) {
                gStaticKickDeg =
                    STATIC_KICK_NEAR_MAX_DEG;
            }
        }
        return 1U;
    }

    /*
     * Use a small, bounded PD command for the last 1.5 cm.  The same capture
     * law is used at +5 and -5, so crossing either target is corrected before
     * the sequence is allowed to continue.
     */
    if ((gState == BALL_STATE_STATIC_GO_MINUS) &&
        (distance <= STATIC_FINISH_ERROR_CM) &&
        (speedMagnitude <= STATIC_FINISH_SPEED_CM_S)) {
        gRodTargetCommandDeg = 0.0f;
        gRodTargetDeg = 0.0f;
        gRodIntegral = 0.0f;
        gStaticCaptureStallMs = 0U;
        return 1U;
    }

    captureCommand =
        BALL_OUTER_LOOP_SIGN *
        (ENDPOINT_POSITION_KP * positionError +
         ENDPOINT_SPEED_KD * gBallSpeedCmS);
    captureLimit = ENDPOINT_MAX_ANGLE_DEG;
    gRodTargetCommandDeg = LimitFloat(
        captureCommand,
        -captureLimit,
        captureLimit);

    if (progress >= STATIC_PROGRESS_CONFIRM_CM) {
        gStaticPulseStartDistanceCm = distance;
        gStaticCaptureStallMs = 0U;
    } else {
        /*
         * Stall detection must use real position progress only.  Camera
         * coordinate jitter can produce a false instantaneous speed above
         * STATIC_MOVING_SPEED_CM_S even while the ball is physically stuck;
         * using that speed to reset this timer left the ball at X=5.8 with
         * only a small PD command forever.
         */
        gStaticCaptureStallMs += CONTROL_PERIOD_MS;
        if (gStaticCaptureStallMs >=
            STATIC_NO_PROGRESS_MS) {
            /*
             * The ball stopped before the endpoint.  Start a short nudge
             * toward the current target without leaving capture mode.
             */
            gStaticFinalNudge = 1U;
            gStaticFinalNudgeMs = 0U;
            gStaticCaptureStallMs = 0U;
            gRodTargetCommandDeg =
                targetDirection *
                gStaticKickDeg;
        }
    }

    return 1U;
}

static void UpdateStaticPulseControl(void)
{
    float error = gBallTargetCm - gBallPositionCm;
    float distance = AbsFloat(error);
    float speedMagnitude = AbsFloat(gBallSpeedCmS);
    float targetDirection;
    float towardTargetSpeed;
    float stopDistance;
    float kickMaximum;
    float progress;
    float runAngle;
    uint32_t kickTimeMs;

    if (((gState == BALL_STATE_STATIC_GO_PLUS) ||
         (gState == BALL_STATE_STATIC_GO_MINUS)) &&
        (gStaticFinalCapture != 0U)) {
        if (UpdateStaticFinalCaptureControl() != 0U) {
            return;
        }
    }

    /*
     * Positive ball motion requires a negative pipe angle.  targetDirection
     * is therefore the pipe-angle sign that accelerates toward the target.
     */
    targetDirection =
        -SignFloat(error) * BALL_OUTER_LOOP_SIGN;

    /*
     * Inside the position band, brake any remaining motion.  Once both
     * position and speed are small, level the pipe.
     */
    if (distance <= STATIC_POSITION_DEADBAND_CM) {
        if (speedMagnitude <= STATIC_MOVING_SPEED_CM_S) {
            gRodTargetCommandDeg = 0.0f;
            ResetStaticPulse();
        } else {
            gRodTargetCommandDeg =
                SignFloat(gBallSpeedCmS) *
                BALL_OUTER_LOOP_SIGN *
                CalculateBrakeAngle(gBallSpeedCmS);
        }
        return;
    }

    towardTargetSpeed =
        SignFloat(error) * gBallSpeedCmS;

    /*
     * Do not use camera-derived speed to end a start pulse.  A pulse is
     * considered successful only after the measured position has made real
     * progress toward the target.
     */
    progress = gStaticPulseStartDistanceCm - distance;
    if ((gStaticMotionConfirmed == 0U) &&
        (progress >= STATIC_MOVE_CONFIRM_CM)) {
        gStaticMotionConfirmed = 1U;
        gStaticPulseStartDistanceCm = distance;
        gStaticNoProgressMs = 0U;
    }

    if (gStaticMotionConfirmed != 0U) {
        /*
         * Require real progress after motion starts.  Camera speed jitter
         * cannot keep the controller in the moving state indefinitely.
         */
        progress = gStaticPulseStartDistanceCm - distance;
        if (progress >= STATIC_PROGRESS_CONFIRM_CM) {
            gStaticPulseStartDistanceCm = distance;
            gStaticNoProgressMs = 0U;
        } else {
            gStaticNoProgressMs += CONTROL_PERIOD_MS;
        }

        if (gStaticNoProgressMs >= STATIC_NO_PROGRESS_MS) {
            RestartStaticPulseAfterStall(distance);
        } else if (towardTargetSpeed >
                   STATIC_MOVING_SPEED_CM_S) {
            if (gState == BALL_STATE_STATIC_GO_MINUS) {
                stopDistance =
                    STATIC_RETURN_STOP_BASE_CM +
                    STATIC_RETURN_STOP_GAIN_S *
                    AbsFloat(towardTargetSpeed);
                runAngle = STATIC_RETURN_RUN_ANGLE_DEG;
            } else {
                stopDistance =
                    STATIC_STOP_DISTANCE_BASE_CM +
                    STATIC_STOP_DISTANCE_GAIN_S *
                    AbsFloat(towardTargetSpeed);
                runAngle = STATIC_RUN_ANGLE_DEG;
            }

            if (distance <= stopDistance) {
                gRodTargetCommandDeg =
                    -targetDirection *
                    CalculateBrakeAngle(towardTargetSpeed);
            } else {
                gRodTargetCommandDeg =
                    targetDirection * runAngle;
            }
            return;
        } else if (towardTargetSpeed <
                   -STATIC_MOVING_SPEED_CM_S) {
            gRodTargetCommandDeg =
                targetDirection *
                CalculateBrakeAngle(towardTargetSpeed);
            return;
        } else {
            /*
             * Wait only until the real-progress timer expires.  If the ball
             * remains stuck, the pulse code below restarts automatically.
             */
            gRodTargetCommandDeg = 0.0f;
            return;
        }

        /*
         * ResetStaticPulse() above intentionally falls through so the first
         * new breakaway pulse is issued in this same control cycle.
         */
    }

    /*
     * No real movement has been confirmed: always finish the drive/coast
     * cycle.  Increase pulse amplitude only after a complete failed cycle.
     */
    kickMaximum = GetStaticKickMaximum(distance);
    kickTimeMs =
        (distance <= STATIC_KICK_NEAR_DISTANCE_CM) ?
            STATIC_KICK_NEAR_TIME_MS :
            STATIC_KICK_TIME_MS;

    if (gStaticPulseState == STATIC_PULSE_DRIVE) {
        gRodTargetCommandDeg =
            targetDirection * gStaticKickDeg;
        gStaticPulseMs += CONTROL_PERIOD_MS;

        if (gStaticPulseMs >= kickTimeMs) {
            gStaticPulseState = STATIC_PULSE_COAST;
            gStaticPulseMs = 0U;
        }
    } else {
        gRodTargetCommandDeg = 0.0f;
        gStaticPulseMs += CONTROL_PERIOD_MS;

        if (gStaticPulseMs >= STATIC_COAST_TIME_MS) {
            gStaticPulseState = STATIC_PULSE_DRIVE;
            gStaticPulseMs = 0U;
            gStaticKickDeg += STATIC_KICK_STEP_DEG;
            if (gStaticKickDeg > kickMaximum) {
                gStaticKickDeg = kickMaximum;
            }
        }
    }

    gRodTargetCommandDeg = LimitFloat(
        gRodTargetCommandDeg,
        -ROD_TARGET_MAX_DEG,
        ROD_TARGET_MAX_DEG);
}

static uint8_t UpdateHoldStopCaptureController(void)
{
    float positionError;
    float speed;
    float command;
    uint8_t stableNow;
    uint8_t stableFinished = 0U;

    if (gHoldStopCaptureActive == 0U) {
        return 0U;
    }

    positionError = gBallPositionCm - gBallTargetCm;
    speed = LimitFloat(
        gBallHoldSpeedCmS,
        -ACTIVE_HOLD_PARAMETER(
            HOLD_SPEED_LIMIT_CM_S,
            ITEM6_HOLD_SPEED_LIMIT_CM_S),
        ACTIVE_HOLD_PARAMETER(
            HOLD_SPEED_LIMIT_CM_S,
            ITEM6_HOLD_SPEED_LIMIT_CM_S));

    /*
     * The newest unfiltered frame detects a real direction reversal before
     * the smoothed speed does.  Switch to that sign immediately so a braking
     * angle cannot keep pushing the ball all the way to the opposite end.
     */
    if (((gBallRawSpeedCmS * speed) < 0.0f) &&
        (AbsFloat(gBallRawSpeedCmS) >=
         ACTIVE_HOLD_PARAMETER(
             HOLD_STOP_RAW_REVERSE_SPEED_CM_S,
             ITEM6_HOLD_STOP_RAW_REVERSE_SPEED_CM_S))) {
        speed = LimitFloat(
            gBallRawSpeedCmS,
            -ACTIVE_HOLD_PARAMETER(
                HOLD_SPEED_LIMIT_CM_S,
                ITEM6_HOLD_SPEED_LIMIT_CM_S),
            ACTIVE_HOLD_PARAMETER(
                HOLD_SPEED_LIMIT_CM_S,
                ITEM6_HOLD_SPEED_LIMIT_CM_S));
    }

    /*
     * Do not apply the normal centered deadband here.  During vehicle stop,
     * even a modest ball speed must immediately create an opposing pipe
     * angle; otherwise the ball can coast almost one centimetre before the
     * ordinary hold controller wakes up.
     */
    command = BALL_OUTER_LOOP_SIGN *
        (ACTIVE_HOLD_PARAMETER(
             HOLD_STOP_POSITION_KP,
             ITEM6_HOLD_STOP_POSITION_KP) * positionError +
         ACTIVE_HOLD_PARAMETER(
             HOLD_STOP_SPEED_KD,
             ITEM6_HOLD_STOP_SPEED_KD) * speed);

    gRodTargetCommandDeg = LimitFloat(
        command + gHoldFeedforwardDeg,
        -ACTIVE_HOLD_PARAMETER(
            HOLD_STOP_MAX_ANGLE_DEG,
            ITEM6_HOLD_STOP_MAX_ANGLE_DEG),
        ACTIVE_HOLD_PARAMETER(
            HOLD_STOP_MAX_ANGLE_DEG,
            ITEM6_HOLD_STOP_MAX_ANGLE_DEG));

    /*
     * Use one fixed, directly tunable backward angle for the whole wheel
     * braking interval.  After the wheels stop, keep that same angle while
     * the ball is still in front of its target.  Feedback is allowed to take
     * over only after the ball has actually returned to the target window.
     */
    if ((gHoldStopVehicleStopped == 0U) ||
        (positionError > ACTIVE_HOLD_PARAMETER(
            HOLD_STOP_RELEASE_ERROR_CM,
            ITEM6_HOLD_STOP_RELEASE_ERROR_CM))) {
        gRodTargetCommandDeg =
            ACTIVE_HOLD_PARAMETER(
                HOLD_STOP_FIXED_BACKWARD_ANGLE_DEG,
                ITEM6_HOLD_STOP_FIXED_BACKWARD_ANGLE_DEG);
    }

    if (gHoldStopCaptureMs < ACTIVE_HOLD_PARAMETER(
        HOLD_STOP_MAX_TIME_MS,
        ITEM6_HOLD_STOP_MAX_TIME_MS)) {
        gHoldStopCaptureMs += CONTROL_PERIOD_MS;
    }

    stableNow = (uint8_t)(
        (AbsFloat(positionError) <=
         ACTIVE_HOLD_PARAMETER(
             HOLD_STOP_STABLE_ERROR_CM,
             ITEM6_HOLD_STOP_STABLE_ERROR_CM)) &&
        (AbsFloat(speed) <=
         ACTIVE_HOLD_PARAMETER(
             HOLD_STOP_STABLE_SPEED_CM_S,
             ITEM6_HOLD_STOP_STABLE_SPEED_CM_S)));

    /* Stability is counted only after wheel PWM has actually reached zero. */
    if ((gHoldStopVehicleStopped != 0U) &&
        (stableNow != 0U)) {
        if (gHoldStopStableMs < ACTIVE_HOLD_PARAMETER(
            HOLD_STOP_STABLE_MS,
            ITEM6_HOLD_STOP_STABLE_MS)) {
            gHoldStopStableMs += CONTROL_PERIOD_MS;
        }
        if (gHoldStopStableMs >= ACTIVE_HOLD_PARAMETER(
            HOLD_STOP_STABLE_MS,
            ITEM6_HOLD_STOP_STABLE_MS)) {
            stableFinished = 1U;
        }
    } else {
        gHoldStopStableMs = 0U;
    }

    if ((stableFinished != 0U) ||
        (gHoldStopCaptureMs >= ACTIVE_HOLD_PARAMETER(
            HOLD_STOP_MAX_TIME_MS,
            ITEM6_HOLD_STOP_MAX_TIME_MS))) {
        gHoldStopCaptureActive = 0U;
        gHoldStopVehicleStopped = 0U;
        gHoldStopCaptureMs = 0U;
        gHoldStopStableMs = 0U;
        gHoldCentered = stableFinished;
        gHoldPositionIntegralDeg = 0.0f;
        gHoldMotionReferencePositionCm = gBallPositionCm;
        gHoldLastCorrectionSign =
            SignFloat(BALL_OUTER_LOOP_SIGN * positionError);
        gHoldMotionWindowMs = 0U;
        gHoldMotionReady = 0U;
        gHoldStaticBoostActive = 0U;
    }

    return 1U;
}

static void SetTask3ClosedCenteredCommand(
    float positionError, float speed)
{
    float feedback = BALL_OUTER_LOOP_SIGN *
        (TASK3_CLOSED_CENTER_POSITION_KP * positionError +
         TASK3_CLOSED_CENTER_SPEED_KD * speed);

    feedback = LimitFloat(
        feedback,
        -TASK3_CLOSED_CENTER_MAX_ANGLE_DEG,
        TASK3_CLOSED_CENTER_MAX_ANGLE_DEG);
    gRodTargetCommandDeg = LimitFloat(
        feedback,
        -TASK3_CLOSED_COMMAND_MAX_ANGLE_DEG,
        TASK3_CLOSED_COMMAND_MAX_ANGLE_DEG);
}

static void UpdateTask3ClosedPositionControl(void)
{
    float positionError = gBallPositionCm - gBallTargetCm;
    float speed = LimitFloat(
        gTask3ClosedSpeedCmS,
        -TASK3_CLOSED_SPEED_LIMIT_CM_S,
        TASK3_CLOSED_SPEED_LIMIT_CM_S);
    float errorMagnitude = AbsFloat(positionError);
    float speedMagnitude = AbsFloat(speed);
    float correctionDirection =
        BALL_OUTER_LOOP_SIGN * positionError;
    float currentCorrectionSign =
        SignFloat(correctionDirection);
    float movedDistanceCm;
    float minimumMoveAngleDeg =
        TASK3_CLOSED_NEGATIVE_MIN_MOVE_DEG;
    float feedbackMinimumDeg =
        -TASK3_CLOSED_MAX_ANGLE_DEG;
    float feedbackMaximumDeg =
        TASK3_CLOSED_MAX_ANGLE_DEG;
    float command;

    /* Never accept an early stop around -4 cm on the first approach. */
    if (positionError <= 0.0f) {
        gTask3ClosedTargetCrossed = 1U;
    }

    /*
     * Task 3 only needs to finish near -5 cm, not chase exactly -5.00 cm.
     * Once the ball has reached the acceptance window and has either slowed
     * down or started rolling back, level the pipe immediately.  Keeping this
     * state latched prevents the normal PD/static-friction controller from
     * injecting another large correction and sending the ball back to zero.
     */
    if (gTask3ClosedAccepted != 0U) {
        if (errorMagnitude <=
            TASK3_CLOSED_ACCEPT_RELEASE_CM) {
            gTask3ClosedCentered = 0U;
            gTask3ClosedIntegralDeg = 0.0f;
            gTask3ClosedMotionWindowMs = 0U;
            gTask3ClosedMotionReady = 0U;
            gTask3ClosedStaticBoostActive = 0U;
            SetTask3RodDirect(0.0f);
            return;
        }

        gTask3ClosedAccepted = 0U;
        gTask3ClosedMotionReferenceCm = gBallPositionCm;
        gTask3ClosedMotionWindowMs = 0U;
        gTask3ClosedMotionReady = 0U;
        gTask3ClosedStaticBoostActive = 0U;
        gTask3ClosedIntegralDeg = 0.0f;
    }

    if ((gTask3ClosedTargetCrossed != 0U) &&
        (errorMagnitude <= TASK3_CLOSED_ACCEPT_ERROR_CM) &&
        ((speed >= 0.0f) ||
         (speedMagnitude <=
          TASK3_CLOSED_ACCEPT_SPEED_CM_S))) {
        gTask3ClosedAccepted = 1U;
        gTask3ClosedCentered = 0U;
        gTask3ClosedIntegralDeg = 0.0f;
        gTask3ClosedMotionWindowMs = 0U;
        gTask3ClosedMotionReady = 0U;
        gTask3ClosedStaticBoostActive = 0U;
        SetTask3RodDirect(0.0f);
        return;
    }

    if (gTask3ClosedCentered != 0U) {
        if ((errorMagnitude <= TASK3_CLOSED_LEAVE_CENTER_CM) &&
            (speedMagnitude <=
             TASK3_CLOSED_CENTER_SPEED_CM_S)) {
            gTask3ClosedIntegralDeg = 0.0f;
            gTask3ClosedMotionReady = 0U;
            gTask3ClosedMotionWindowMs = 0U;
            gTask3ClosedStaticBoostActive = 0U;
            SetTask3ClosedCenteredCommand(
                positionError, speed);
            gRodIntegral = 0.0f;
            return;
        }
        gTask3ClosedCentered = 0U;
    } else if ((errorMagnitude <=
                TASK3_CLOSED_ENTER_CENTER_CM) &&
               (speedMagnitude <=
                TASK3_CLOSED_CENTER_SPEED_CM_S)) {
        gTask3ClosedCentered = 1U;
        gTask3ClosedIntegralDeg = 0.0f;
        gTask3ClosedMotionReady = 0U;
        gTask3ClosedMotionWindowMs = 0U;
        gTask3ClosedStaticBoostActive = 0U;
        SetTask3ClosedCenteredCommand(positionError, speed);
        gRodIntegral = 0.0f;
        return;
    }

    if (gTask3ClosedMotionReady == 0U) {
        gTask3ClosedMotionReferenceCm = gBallPositionCm;
        gTask3ClosedLastCorrectionSign =
            currentCorrectionSign;
        gTask3ClosedMotionWindowMs = 0U;
        gTask3ClosedStaticBoostActive = 0U;
        gTask3ClosedMotionReady = 1U;
    }

    if ((currentCorrectionSign *
         gTask3ClosedLastCorrectionSign) < 0.0f) {
        gTask3ClosedMotionReferenceCm = gBallPositionCm;
        gTask3ClosedLastCorrectionSign =
            currentCorrectionSign;
        gTask3ClosedMotionWindowMs = 0U;
        gTask3ClosedStaticBoostActive = 0U;
        gTask3ClosedIntegralDeg = 0.0f;
    }

    movedDistanceCm = AbsFloat(
        gBallPositionCm - gTask3ClosedMotionReferenceCm);

    if (movedDistanceCm >=
        TASK3_CLOSED_MOTION_CONFIRM_CM) {
        gTask3ClosedMotionReferenceCm = gBallPositionCm;
        gTask3ClosedMotionWindowMs = 0U;
        gTask3ClosedStaticBoostActive = 0U;
        gTask3ClosedIntegralDeg = 0.0f;
    } else if (errorMagnitude >
               TASK3_CLOSED_LEAVE_CENTER_CM) {
        if (gTask3ClosedMotionWindowMs <
            TASK3_CLOSED_STILL_WINDOW_MS) {
            gTask3ClosedMotionWindowMs += CONTROL_PERIOD_MS;
        }

        if (gTask3ClosedMotionWindowMs >=
            TASK3_CLOSED_STILL_WINDOW_MS) {
            gTask3ClosedStaticBoostActive = 1U;
            gTask3ClosedIntegralDeg +=
                currentCorrectionSign *
                TASK3_CLOSED_STATIC_I_RAMP_DEG_S *
                ((float)CONTROL_PERIOD_MS / 1000.0f);
            gTask3ClosedIntegralDeg = LimitFloat(
                gTask3ClosedIntegralDeg,
                -TASK3_CLOSED_I_LIMIT_DEG,
                TASK3_CLOSED_I_LIMIT_DEG);
        }
    }

    command =
        BALL_OUTER_LOOP_SIGN *
        (TASK3_CLOSED_POSITION_KP * positionError +
         TASK3_CLOSED_SPEED_KD * speed) +
        gTask3ClosedIntegralDeg;

    if ((gTask3ClosedStaticBoostActive != 0U) &&
        (currentCorrectionSign > 0.0f)) {
        minimumMoveAngleDeg =
            TASK3_CLOSED_POSITIVE_MIN_MOVE_DEG;
        feedbackMaximumDeg =
            TASK3_CLOSED_POSITIVE_STATIC_MAX_DEG;
    }

    if ((errorMagnitude > TASK3_CLOSED_LEAVE_CENTER_CM) &&
        (gTask3ClosedStaticBoostActive != 0U) &&
        (AbsFloat(command) < minimumMoveAngleDeg)) {
        command = BALL_OUTER_LOOP_SIGN *
                  SignFloat(positionError) *
                  minimumMoveAngleDeg;
    }

    command = LimitFloat(
        command,
        feedbackMinimumDeg,
        feedbackMaximumDeg);

    gRodTargetCommandDeg = LimitFloat(
        command,
        -TASK3_CLOSED_COMMAND_MAX_ANGLE_DEG,
        TASK3_CLOSED_COMMAND_MAX_ANGLE_DEG);
}

static void SetCenteredHoldCommand(
    float positionError, float speed)
{
    float feedback = BALL_OUTER_LOOP_SIGN *
        (ACTIVE_HOLD_PARAMETER(
             HOLD_CENTER_POSITION_KP,
             ITEM6_HOLD_CENTER_POSITION_KP) * positionError +
         ACTIVE_HOLD_PARAMETER(
             HOLD_CENTER_SPEED_KD,
             ITEM6_HOLD_CENTER_SPEED_KD) * speed);

    feedback = LimitFloat(
        feedback,
        -ACTIVE_HOLD_PARAMETER(
            HOLD_CENTER_FEEDBACK_MAX_DEG,
            ITEM6_HOLD_CENTER_FEEDBACK_MAX_DEG),
        ACTIVE_HOLD_PARAMETER(
            HOLD_CENTER_FEEDBACK_MAX_DEG,
            ITEM6_HOLD_CENTER_FEEDBACK_MAX_DEG));
    gRodTargetCommandDeg = LimitFloat(
        feedback + gHoldFeedforwardDeg,
        -ACTIVE_HOLD_PARAMETER(
            HOLD_TRANSIENT_MAX_ANGLE_DEG,
            ITEM6_HOLD_TRANSIENT_MAX_ANGLE_DEG),
        ACTIVE_HOLD_PARAMETER(
            HOLD_TRANSIENT_MAX_ANGLE_DEG,
            ITEM6_HOLD_TRANSIENT_MAX_ANGLE_DEG));
}

static void UpdateGeneralPositionControl(void)
{
    float positionError = gBallPositionCm - gBallTargetCm;
    float speed = LimitFloat(
        gBallHoldSpeedCmS,
        -ACTIVE_HOLD_PARAMETER(
            HOLD_SPEED_LIMIT_CM_S,
            ITEM6_HOLD_SPEED_LIMIT_CM_S),
        ACTIVE_HOLD_PARAMETER(
            HOLD_SPEED_LIMIT_CM_S,
            ITEM6_HOLD_SPEED_LIMIT_CM_S));
    float errorMagnitude = AbsFloat(positionError);
    float speedMagnitude = AbsFloat(speed);
    float correctionDirection =
        BALL_OUTER_LOOP_SIGN * positionError;
    float currentCorrectionSign =
        SignFloat(correctionDirection);
    float movedDistanceCm;
    float minimumMoveAngleDeg =
        ACTIVE_HOLD_PARAMETER(
            HOLD_NEGATIVE_MIN_MOVE_ANGLE_DEG,
            ITEM6_HOLD_NEGATIVE_MIN_MOVE_ANGLE_DEG);
    float feedbackMinimumDeg = -ACTIVE_HOLD_PARAMETER(
        HOLD_MAX_ANGLE_DEG,
        ITEM6_HOLD_MAX_ANGLE_DEG);
    float feedbackMaximumDeg = ACTIVE_HOLD_PARAMETER(
        HOLD_MAX_ANGLE_DEG,
        ITEM6_HOLD_MAX_ANGLE_DEG);
    float command;

    if (UpdateHoldStopCaptureController() != 0U) {
        return;
    }

    /*
     * Hysteresis still prevents static-friction chatter, but centered mode no
     * longer turns feedback off.  The continuous speed term damps small ball
     * motion before it can grow into a large position error.
     */
    if (gHoldCentered != 0U) {
        if ((errorMagnitude <= ACTIVE_HOLD_PARAMETER(
                 HOLD_LEAVE_CENTER_CM,
                 ITEM6_HOLD_LEAVE_CENTER_CM)) &&
            (speedMagnitude <= ACTIVE_HOLD_PARAMETER(
                 HOLD_CENTER_SPEED_CM_S,
                 ITEM6_HOLD_CENTER_SPEED_CM_S))) {
            gHoldPositionIntegralDeg = 0.0f;
            gHoldMotionReady = 0U;
            gHoldMotionWindowMs = 0U;
            gHoldStaticBoostActive = 0U;
            SetCenteredHoldCommand(positionError, speed);
            gRodIntegral = 0.0f;
            return;
        }
        gHoldCentered = 0U;
    } else if ((errorMagnitude <= ACTIVE_HOLD_PARAMETER(
                    HOLD_ENTER_CENTER_CM,
                    ITEM6_HOLD_ENTER_CENTER_CM)) &&
               (speedMagnitude <= ACTIVE_HOLD_PARAMETER(
                    HOLD_CENTER_SPEED_CM_S,
                    ITEM6_HOLD_CENTER_SPEED_CM_S))) {
        gHoldCentered = 1U;
        gHoldPositionIntegralDeg = 0.0f;
        gHoldMotionReady = 0U;
        gHoldMotionWindowMs = 0U;
        gHoldStaticBoostActive = 0U;
        SetCenteredHoldCommand(positionError, speed);
        gRodIntegral = 0.0f;
        return;
    }

    /*
     * Do not use "progress toward O" to detect a stuck ball.  After crossing
     * O, a ball rolling away is still physically moving; treating that as
     * stuck injects another breakaway kick and makes each oscillation larger.
     * Any real displacement, in either direction, disables the static boost.
     */
    if (gHoldMotionReady == 0U) {
        gHoldMotionReferencePositionCm = gBallPositionCm;
        gHoldLastCorrectionSign = currentCorrectionSign;
        gHoldMotionWindowMs = 0U;
        gHoldStaticBoostActive = 0U;
        gHoldMotionReady = 1U;
    }

    if ((currentCorrectionSign * gHoldLastCorrectionSign) < 0.0f) {
        gHoldMotionReferencePositionCm = gBallPositionCm;
        gHoldLastCorrectionSign = currentCorrectionSign;
        gHoldMotionWindowMs = 0U;
        gHoldStaticBoostActive = 0U;
        gHoldPositionIntegralDeg = 0.0f;
    }

    movedDistanceCm = AbsFloat(
        gBallPositionCm - gHoldMotionReferencePositionCm);

    if (movedDistanceCm >= ACTIVE_HOLD_PARAMETER(
        HOLD_MOTION_CONFIRM_CM,
        ITEM6_HOLD_MOTION_CONFIRM_CM)) {
        gHoldMotionReferencePositionCm = gBallPositionCm;
        gHoldMotionWindowMs = 0U;
        gHoldStaticBoostActive = 0U;
        gHoldPositionIntegralDeg = 0.0f;
    } else if (errorMagnitude > ACTIVE_HOLD_PARAMETER(
                   HOLD_LEAVE_CENTER_CM,
                   ITEM6_HOLD_LEAVE_CENTER_CM)) {
        if (gHoldMotionWindowMs < ACTIVE_HOLD_PARAMETER(
            HOLD_STILL_WINDOW_MS,
            ITEM6_HOLD_STILL_WINDOW_MS)) {
            gHoldMotionWindowMs += CONTROL_PERIOD_MS;
        }

        if (gHoldMotionWindowMs >= ACTIVE_HOLD_PARAMETER(
            HOLD_STILL_WINDOW_MS,
            ITEM6_HOLD_STILL_WINDOW_MS)) {
            gHoldStaticBoostActive = 1U;
            gHoldPositionIntegralDeg +=
                currentCorrectionSign *
                ACTIVE_HOLD_PARAMETER(
                    HOLD_STATIC_I_RAMP_DEG_S,
                    ITEM6_HOLD_STATIC_I_RAMP_DEG_S) *
                ((float)CONTROL_PERIOD_MS / 1000.0f);
            gHoldPositionIntegralDeg = LimitFloat(
                gHoldPositionIntegralDeg,
                -ACTIVE_HOLD_PARAMETER(
                    HOLD_POSITION_I_LIMIT_DEG,
                    ITEM6_HOLD_POSITION_I_LIMIT_DEG),
                ACTIVE_HOLD_PARAMETER(
                    HOLD_POSITION_I_LIMIT_DEG,
                    ITEM6_HOLD_POSITION_I_LIMIT_DEG));
        }
    }

    /* Position P restores O; measured speed supplies predictive braking. */

    command =
        BALL_OUTER_LOOP_SIGN *
        (ACTIVE_HOLD_PARAMETER(
             HOLD_POSITION_KP,
             ITEM6_HOLD_POSITION_KP) * positionError +
         ACTIVE_HOLD_PARAMETER(
             HOLD_SPEED_KD,
             ITEM6_HOLD_SPEED_KD) * speed) +
        gHoldPositionIntegralDeg;

    /*
     * The linkage needs a little more breakaway angle only when the requested
     * OLED T direction is positive.  Keep the normal moving-ball limit at
     * +/-HOLD_MAX_ANGLE_DEG; the extra range exists only while physically
     * stationary, and is removed by the first confirmed displacement.
     */
    if ((gHoldStaticBoostActive != 0U) &&
        (currentCorrectionSign > 0.0f)) {
        minimumMoveAngleDeg =
            ACTIVE_HOLD_PARAMETER(
                HOLD_POSITIVE_MIN_MOVE_ANGLE_DEG,
                ITEM6_HOLD_POSITIVE_MIN_MOVE_ANGLE_DEG);
        feedbackMaximumDeg =
            ACTIVE_HOLD_PARAMETER(
                HOLD_POSITIVE_STATIC_MAX_DEG,
                ITEM6_HOLD_POSITIVE_STATIC_MAX_DEG);
    }

    /*
     * Static friction can hold the ball outside the allowed center region.
     * The minimum useful angle is allowed only after the position-based still
     * detector has fired.  The first confirmed displacement cancels it.
     */
    if ((errorMagnitude > ACTIVE_HOLD_PARAMETER(
             HOLD_LEAVE_CENTER_CM,
             ITEM6_HOLD_LEAVE_CENTER_CM)) &&
        (gHoldStaticBoostActive != 0U) &&
        (AbsFloat(command) < minimumMoveAngleDeg)) {
        command = BALL_OUTER_LOOP_SIGN *
                  SignFloat(positionError) *
                  minimumMoveAngleDeg;
    }

    /* Limit the feedback part first, then add the known car-motion force. */
    command = LimitFloat(
        command,
        feedbackMinimumDeg,
        feedbackMaximumDeg);

    gRodTargetCommandDeg = LimitFloat(
        command + gHoldFeedforwardDeg,
        -ACTIVE_HOLD_PARAMETER(
            HOLD_TRANSIENT_MAX_ANGLE_DEG,
            ITEM6_HOLD_TRANSIENT_MAX_ANGLE_DEG),
        ACTIVE_HOLD_PARAMETER(
            HOLD_TRANSIENT_MAX_ANGLE_DEG,
            ITEM6_HOLD_TRANSIENT_MAX_ANGLE_DEG));
}

static void SlewRodTarget(void)
{
    float difference =
        gRodTargetCommandDeg - gRodTargetDeg;
    float maximumSlew = ROD_TARGET_SLEW_DEG;

    if ((gTask3OpenPhase == TASK3_PHASE_CLOSED_TO_MINUS) &&
        ((gState == BALL_STATE_STATIC_GO_MINUS) ||
         (gState == BALL_STATE_STATIC_FINISHED))) {
        maximumSlew = TASK3_CLOSED_SLEW_DEG;
    } else if ((gState == BALL_STATE_HOLD) &&
        ((gHoldStopCaptureActive != 0U) ||
         (AbsFloat(gHoldFeedforwardDeg) > 0.50f))) {
        maximumSlew = ACTIVE_HOLD_PARAMETER(
            HOLD_TRANSIENT_SLEW_DEG,
            ITEM6_HOLD_TRANSIENT_SLEW_DEG);
    }

    difference = LimitFloat(
        difference,
        -maximumSlew,
        maximumSlew);
    gRodTargetDeg += difference;
}

static void UpdateAngleInnerLoop(void)
{
    float angleError = gRodTargetDeg - gRodAngleDeg;
    float speedCommand;

    if (AbsFloat(angleError) < ROD_ANGLE_DEADBAND_DEG) {
        angleError = 0.0f;
    }

    gRodIntegral +=
        angleError * ((float)CONTROL_PERIOD_MS / 1000.0f);
    gRodIntegral = LimitFloat(
        gRodIntegral,
        -ROD_INTEGRAL_LIMIT,
        ROD_INTEGRAL_LIMIT);

    speedCommand =
        ROD_ANGLE_KP * angleError +
        ROD_ANGLE_KI * gRodIntegral -
        ROD_ANGLE_KD * gRodRateDegS;

    RodStepper_SetSpeed(speedCommand);
}

void BallControl_Init(void)
{
    gState = BALL_STATE_DISABLED;
    gFault = BALL_FAULT_NONE;
    gHasZero = 0U;
    gLastControlMs = 0U;

    gBallPositionCm = 0.0f;
    gBallTargetCm = 0.0f;
    gBallSpeedCmS = 0.0f;
    gBallHoldSpeedCmS = 0.0f;
    gBallRawSpeedCmS = 0.0f;
    gLastFramePositionCm = 0.0f;
    gLastFrameSequence = 0xFFFFU;
    gLastFrameMs = 0U;
    gHavePreviousFrame = 0U;
    gBallOutsideFrames = 0U;

    gRodAngleDeg = 0.0f;
    gLastRodAngleDeg = 0.0f;
    gRodRateDegS = 0.0f;
    gRodTargetCommandDeg = 0.0f;
    gRodTargetDeg = 0.0f;
    gRodIntegral = 0.0f;
    gHoldCentered = 0U;
    gHoldUseItem6Parameters = 0U;
    gHoldFeedforwardDeg = 0.0f;
    gHoldPositionIntegralDeg = 0.0f;
    gHoldMotionReferencePositionCm = 0.0f;
    gHoldLastCorrectionSign = 1.0f;
    gHoldMotionWindowMs = 0U;
    gHoldMotionReady = 0U;
    gHoldStaticBoostActive = 0U;

    gStaticStableMs = 0U;
    ResetStaticFinalCapture();
    ResetStaticPulse();
    ResetTask3Controller();
    ResetHoldStopCapture();
}

void BallControl_Update(uint32_t now_ms)
{
    float measuredRodRate;
    uint8_t needsVision;
    uint8_t visionUpdated = 0U;

    if ((uint32_t)(now_ms - gLastControlMs) <
        CONTROL_PERIOD_MS) {
        return;
    }
    gLastControlMs = now_ms;

    gRodAngleDeg =
        (float)RodStepper_GetLogicalPosition() /
        ROD_STEPS_PER_DEGREE;
    measuredRodRate =
        (gRodAngleDeg - gLastRodAngleDeg) *
        (1000.0f / (float)CONTROL_PERIOD_MS);
    gRodRateDegS += ROD_RATE_FILTER_ALPHA *
                   (measuredRodRate - gRodRateDegS);
    gLastRodAngleDeg = gRodAngleDeg;

    if (gState == BALL_STATE_DISABLED) {
        RodStepper_Stop();
        RodStepper_Enable(0U);
        return;
    }

    if ((AbsFloat(gRodAngleDeg) > ROD_SAFE_ANGLE_DEG) &&
        (gState != BALL_STATE_FAULT)) {
        SetFault(BALL_FAULT_ANGLE_LIMIT);
    }
    if ((RodStepper_IsAtSoftLimit() != 0U) &&
        (gState != BALL_STATE_FAULT)) {
        SetFault(BALL_FAULT_STEPPER_LIMIT);
    }

    needsVision = (uint8_t)(
        (gState == BALL_STATE_HOLD) ||
        (gState == BALL_STATE_STATIC_GO_PLUS) ||
        (gState == BALL_STATE_STATIC_GO_MINUS) ||
        (gState == BALL_STATE_STATIC_FINISHED));

    if (needsVision != 0U) {
        if (BallVision_IsFresh(
                now_ms, VISION_TIMEOUT_MS) == 0U) {
            SetFault(BALL_FAULT_VISION_LOST);
        } else {
            visionUpdated =
                UpdateVisionMeasurement(now_ms);

            if ((visionUpdated == 0U) &&
                (gHavePreviousFrame != 0U) &&
                ((uint32_t)(now_ms - gLastFrameMs) >
                 VISION_STATIONARY_REFRESH_MS)) {
                gBallSpeedCmS *= BALL_SPEED_STALE_DECAY;
                gBallHoldSpeedCmS *=
                    ACTIVE_HOLD_PARAMETER(
                        HOLD_SPEED_FAST_STALE_DECAY,
                        ITEM6_HOLD_SPEED_FAST_STALE_DECAY);
                gTask3ClosedSpeedCmS *=
                    TASK3_CLOSED_SPEED_STALE_DECAY;
                gBallRawSpeedCmS = 0.0f;
                if (AbsFloat(gBallSpeedCmS) < 0.05f) {
                    gBallSpeedCmS = 0.0f;
                }
                if (AbsFloat(gBallHoldSpeedCmS) < 0.05f) {
                    gBallHoldSpeedCmS = 0.0f;
                }
            }

            if (visionUpdated != 0U) {
                if (AbsFloat(gBallPositionCm) >
                    BALL_SAFE_POSITION_CM) {
                    if (gBallOutsideFrames < 255U) {
                        gBallOutsideFrames++;
                    }
                    if (gBallOutsideFrames >=
                        BALL_OUTSIDE_CONFIRM_FRAMES) {
                        SetFault(BALL_FAULT_BALL_OUTSIDE);
                    }
                } else {
                    gBallOutsideFrames = 0U;
                }
            }
        }
    }

    if ((gState == BALL_STATE_STATIC_GO_PLUS) ||
        (gState == BALL_STATE_STATIC_GO_MINUS)) {
        UpdateTask3SequenceState();
    }

    if ((gState == BALL_STATE_STATIC_GO_PLUS) ||
        (gState == BALL_STATE_STATIC_GO_MINUS) ||
        (gState == BALL_STATE_STATIC_FINISHED)) {
        UpdateTask3SingleController();
    } else if (gState == BALL_STATE_HOLD) {
        UpdateGeneralPositionControl();
    } else {
        gRodTargetCommandDeg = 0.0f;
    }

    SlewRodTarget();
    UpdateAngleInnerLoop();
}

void BallControl_SetCurrentRodAsZero(void)
{
    RodStepper_Stop();
    RodStepper_ResetLogicalPosition();
    gRodAngleDeg = 0.0f;
    gLastRodAngleDeg = 0.0f;
    gRodTargetDeg = 0.0f;
    gRodTargetCommandDeg = 0.0f;
    gRodRateDegS = 0.0f;
    gRodIntegral = 0.0f;
    gBallOutsideFrames = 0U;
    gHoldUseItem6Parameters = 0U;
    gHasZero = 1U;
    gFault = BALL_FAULT_NONE;
    gState = BALL_STATE_DISABLED;
    ResetStaticFinalCapture();
    ResetStaticPulse();
    ResetTask3Controller();
    ResetHoldStopCapture();
}

uint8_t BallControl_HasZero(void)
{
    return gHasZero;
}

uint8_t BallControl_StartLevel(void)
{
    if (StartCommon() == 0U) {
        return 0U;
    }
    gHoldUseItem6Parameters = 0U;
    gBallTargetCm = 0.0f;
    gRodTargetCommandDeg = 0.0f;
    gState = BALL_STATE_LEVEL;
    return 1U;
}

static uint8_t StartHoldWithProfile(
    float target_cm, uint8_t useItem6Parameters)
{
    if (StartCommon() == 0U) {
        return 0U;
    }
    gHoldUseItem6Parameters =
        (useItem6Parameters != 0U) ? 1U : 0U;
    gBallTargetCm =
        LimitFloat(target_cm, -10.0f, 10.0f);
    gHoldCentered = 0U;
    gHoldFeedforwardDeg = 0.0f;
    gHoldPositionIntegralDeg = 0.0f;
    gHoldMotionReferencePositionCm = 0.0f;
    gHoldLastCorrectionSign = 1.0f;
    gHoldMotionWindowMs = 0U;
    gHoldMotionReady = 0U;
    gHoldStaticBoostActive = 0U;
    gState = BALL_STATE_HOLD;
    return 1U;
}

uint8_t BallControl_StartHold(float target_cm)
{
    return StartHoldWithProfile(target_cm, 0U);
}

uint8_t BallControl_StartItem6Hold(float target_cm)
{
    return StartHoldWithProfile(target_cm, 1U);
}

void BallControl_UseItem6HoldParameters(void)
{
    if (gHoldUseItem6Parameters != 0U) {
        return;
    }

    gHoldUseItem6Parameters = 1U;
    if (gState != BALL_STATE_HOLD) {
        return;
    }

    gHoldCentered = 0U;
    gHoldPositionIntegralDeg = 0.0f;
    gHoldMotionReferencePositionCm = gBallPositionCm;
    gHoldLastCorrectionSign = 1.0f;
    gHoldMotionWindowMs = 0U;
    gHoldMotionReady = 0U;
    gHoldStaticBoostActive = 0U;
    ResetHoldStopCapture();
}

void BallControl_SetHoldTarget(float target_cm)
{
    if (gState != BALL_STATE_HOLD) {
        return;
    }

    gBallTargetCm = LimitFloat(
        target_cm, ITEM6_TARGET_MIN_CM, ITEM6_TARGET_MAX_CM);
    gHoldCentered = 0U;
    gHoldPositionIntegralDeg = 0.0f;
    gHoldMotionReferencePositionCm = 0.0f;
    gHoldLastCorrectionSign = 1.0f;
    gHoldMotionWindowMs = 0U;
    gHoldMotionReady = 0U;
    gHoldStaticBoostActive = 0U;
    ResetHoldStopCapture();
}

void BallControl_SetHoldFeedforwardDeg(float angle_deg)
{
    float maximumFeedforwardDeg = ACTIVE_HOLD_PARAMETER(
        HOLD_CAR_FEEDFORWARD_MAX_DEG,
        ITEM6_HOLD_CAR_FEEDFORWARD_MAX_DEG);

    gHoldFeedforwardDeg = LimitFloat(
        angle_deg,
        -maximumFeedforwardDeg,
        maximumFeedforwardDeg);
}

void BallControl_StartStopCapture(void)
{
    if (gState != BALL_STATE_HOLD) {
        return;
    }

    gHoldStopCaptureActive = 1U;
    gHoldStopVehicleStopped = 0U;
    gHoldStopCaptureMs = 0U;
    gHoldStopStableMs = 0U;
    gHoldCentered = 0U;
    gHoldPositionIntegralDeg = 0.0f;
    gHoldMotionReferencePositionCm = gBallPositionCm;
    gHoldMotionWindowMs = 0U;
    gHoldMotionReady = 0U;
    gHoldStaticBoostActive = 0U;
}

void BallControl_SetStopCaptureVehicleStopped(uint8_t stopped)
{
    if (gHoldStopCaptureActive == 0U) {
        return;
    }

    gHoldStopVehicleStopped =
        (stopped != 0U) ? 1U : 0U;
    if (gHoldStopVehicleStopped == 0U) {
        gHoldStopStableMs = 0U;
    }
}

void BallControl_CancelStopCapture(void)
{
    ResetHoldStopCapture();
    gHoldCentered = 0U;
    gHoldPositionIntegralDeg = 0.0f;
    gHoldMotionReferencePositionCm = gBallPositionCm;
    gHoldMotionWindowMs = 0U;
    gHoldMotionReady = 0U;
    gHoldStaticBoostActive = 0U;
}

uint8_t BallControl_IsStopCaptureActive(void)
{
    return gHoldStopCaptureActive;
}

uint8_t BallControl_StartStaticSequence(void)
{
    if (StartCommon() == 0U) {
        return 0U;
    }
    gHoldUseItem6Parameters = 0U;
    gBallTargetCm = TASK3_OPEN_SWITCH_CM;
    gState = BALL_STATE_STATIC_GO_PLUS;
    ResetStaticFinalCapture();
    ResetStaticPulse();
    ResetTask3Controller();
    return 1U;
}

void BallControl_Stop(void)
{
    gState = BALL_STATE_DISABLED;
    gFault = BALL_FAULT_NONE;
    gRodTargetCommandDeg = 0.0f;
    gRodTargetDeg = 0.0f;
    gRodIntegral = 0.0f;
    gBallOutsideFrames = 0U;
    gHoldUseItem6Parameters = 0U;
    gHoldFeedforwardDeg = 0.0f;
    gHoldPositionIntegralDeg = 0.0f;
    gHoldMotionReferencePositionCm = 0.0f;
    gHoldLastCorrectionSign = 1.0f;
    gHoldMotionWindowMs = 0U;
    gHoldMotionReady = 0U;
    gHoldStaticBoostActive = 0U;
    ResetHoldStopCapture();
    gHasZero = 0U;
    ResetStaticFinalCapture();
    ResetStaticPulse();
    ResetTask3Controller();
    RodStepper_Stop();
    RodStepper_Enable(0U);
}

void BallControl_ClearFault(void)
{
    if (gState == BALL_STATE_FAULT) {
        BallControl_Stop();
    }
}

uint8_t BallControl_IsRunning(void)
{
    return (uint8_t)(
        (gState == BALL_STATE_LEVEL) ||
        (gState == BALL_STATE_HOLD) ||
        (gState == BALL_STATE_STATIC_GO_PLUS) ||
        (gState == BALL_STATE_STATIC_GO_MINUS));
}

uint8_t BallControl_IsFinished(void)
{
    return (uint8_t)(
        gState == BALL_STATE_STATIC_FINISHED);
}

uint8_t BallControl_HasFault(void)
{
    return (uint8_t)(gState == BALL_STATE_FAULT);
}

BallControlState BallControl_GetState(void)
{
    return gState;
}

BallFault BallControl_GetFault(void)
{
    return gFault;
}

float BallControl_GetBallPositionCm(void)
{
    return gBallPositionCm;
}

float BallControl_GetBallTargetCm(void)
{
    return gBallTargetCm;
}

float BallControl_GetRodAngleDeg(void)
{
    return gRodAngleDeg;
}

float BallControl_GetRodTargetDeg(void)
{
    return gRodTargetDeg;
}

float BallControl_GetBallSpeedCmS(void)
{
    return gBallSpeedCmS;
}

float BallControl_GetHoldSpeedCmS(void)
{
    return gBallHoldSpeedCmS;
}

float BallControl_GetStepperSpeed(void)
{
    return RodStepper_GetCommandSpeed();
}
