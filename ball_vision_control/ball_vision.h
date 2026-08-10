#ifndef BALL_VISION_H_
#define BALL_VISION_H_

#include <stdint.h>

typedef struct {
    uint16_t sequence;
    uint8_t valid;
    int16_t x_tenth_mm;
    uint8_t confidence;
    uint32_t receive_ms;
} BallVisionFrame;

void BallVision_Init(void);
void BallVision_Update(uint32_t now_ms);
uint8_t BallVision_GetFrame(BallVisionFrame *frame);
uint8_t BallVision_IsFresh(uint32_t now_ms, uint32_t timeout_ms);
float BallVision_GetPositionCm(void);
uint8_t BallVision_HasTarget(void);
float BallVision_GetTargetCm(void);
uint32_t BallVision_GetTargetVersion(void);
uint32_t BallVision_GetGoodFrameCount(void);
uint32_t BallVision_GetBadFrameCount(void);

#endif
