#ifndef BALL_CONTROL_H_
#define BALL_CONTROL_H_

#include <stdint.h>

typedef enum {
    BALL_STATE_DISABLED = 0,
    BALL_STATE_LEVEL,
    BALL_STATE_HOLD,
    BALL_STATE_STATIC_GO_PLUS,
    BALL_STATE_STATIC_GO_MINUS,
    BALL_STATE_STATIC_FINISHED,
    BALL_STATE_FAULT
} BallControlState;

typedef enum {
    BALL_FAULT_NONE = 0,
    BALL_FAULT_NO_ZERO,
    BALL_FAULT_VISION_LOST,
    BALL_FAULT_ANGLE_LIMIT,
    BALL_FAULT_BALL_OUTSIDE,
    BALL_FAULT_STEPPER_LIMIT
} BallFault;

void BallControl_Init(void);
void BallControl_Update(uint32_t now_ms);

void BallControl_SetCurrentRodAsZero(void);
uint8_t BallControl_HasZero(void);

uint8_t BallControl_StartLevel(void);
uint8_t BallControl_StartHold(float target_cm);
uint8_t BallControl_StartItem6Hold(float target_cm);
void BallControl_UseItem6HoldParameters(void);
void BallControl_SetHoldTarget(float target_cm);
uint8_t BallControl_StartStaticSequence(void);
void BallControl_SetHoldFeedforwardDeg(float angle_deg);
void BallControl_StartStopCapture(void);
void BallControl_SetStopCaptureVehicleStopped(uint8_t stopped);
void BallControl_CancelStopCapture(void);
uint8_t BallControl_IsStopCaptureActive(void);
uint8_t BallControl_IsStopReverseConfirmed(void);
void BallControl_Stop(void);
void BallControl_ClearFault(void);

uint8_t BallControl_IsRunning(void);
uint8_t BallControl_IsFinished(void);
uint8_t BallControl_HasFault(void);
BallControlState BallControl_GetState(void);
BallFault BallControl_GetFault(void);

float BallControl_GetBallPositionCm(void);
float BallControl_GetBallTargetCm(void);
/* 摆杆角度由S3清零后的步进脉冲数估算，不是传感器实测值。 */
float BallControl_GetRodAngleDeg(void);
float BallControl_GetRodTargetDeg(void);
float BallControl_GetBallSpeedCmS(void);
float BallControl_GetHoldSpeedCmS(void);
float BallControl_GetStepperSpeed(void);

#endif
