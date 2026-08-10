#ifndef H_LINE_TRACKING_CONTROL_H_
#define H_LINE_TRACKING_CONTROL_H_

#include <stdint.h>

typedef enum {
    CAR_STATE_IDLE = 0,
    CAR_STATE_WAIT_LINE,
    CAR_STATE_LEAVE_START_MARKER,
    CAR_STATE_TRACK,
    CAR_STATE_FINISH_DELAY,
    CAR_STATE_FINISH_ADVANCE,
    CAR_STATE_FINISHED,
    CAR_STATE_FAULT
} CarControlState;

typedef enum {
    CAR_TEST_ITEM_2 = 2,
    CAR_TEST_ITEM_4 = 4,
    CAR_TEST_ITEM_5 = 5,
    CAR_TEST_ITEM_6 = 6
} CarTestMode;

void CarControl_Init(void);
void CarControl_Start(CarTestMode mode);
void CarControl_Update(void);
void CarControl_Abort(void);
void CarControl_On10msTick(void);

uint8_t CarControl_IsRunning(void);
uint8_t CarControl_IsFinished(void);
uint8_t CarControl_HasFault(void);
uint8_t CarControl_IsSoftStopping(void);
uint16_t CarControl_GetSoftStopProgressPermille(void);
uint8_t CarControl_GetCompletedLaps(void);
CarTestMode CarControl_GetTestMode(void);
uint8_t CarControl_GetSensorLevelMask(void);
uint8_t CarControl_GetBlackMask(void);
uint32_t CarControl_GetLeftEncoderSpeed(void);
uint32_t CarControl_GetRightEncoderSpeed(void);
uint32_t CarControl_GetElapsed10ms(void);
uint32_t CarControl_GetTick10ms(void);
CarControlState CarControl_GetState(void);

#endif
