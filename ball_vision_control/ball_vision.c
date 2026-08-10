#include "ball_vision.h"
#include "ball_config.h"

#define RX_LINE_SIZE  48U

static volatile char gRxLine[RX_LINE_SIZE];
static volatile uint8_t gRxLength;
static volatile uint8_t gLineReady;

static BallVisionFrame gFrame;
static uint32_t gGoodFrames;
static uint32_t gBadFrames;
static uint8_t gTargetValid;
static int16_t gTargetTenthMm;
static uint32_t gTargetVersion;
static uint16_t gNextSequence;
static uint32_t gLastMeasurementMs;

static void ReceiveByte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if ((gRxLength != 0U) && (gLineReady == 0U)) {
            gRxLine[gRxLength] = '\0';
            gLineReady = 1U;
        }
        return;
    }

    if (gLineReady != 0U) {
        return;
    }

    if (gRxLength < (RX_LINE_SIZE - 1U)) {
        gRxLine[gRxLength++] = (char)byte;
    } else {
        gRxLength = 0U;
    }
}

static void DrainUart(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(BALL_UART_INST)) {
        ReceiveByte(DL_UART_Main_receiveData(BALL_UART_INST));
    }
}

/* 把最多两位小数转换为放大100倍的整数。 */
static uint8_t ParseFixedHundredths(
    const char **cursor, int32_t *value)
{
    int32_t integerPart = 0;
    int32_t fractionPart = 0;
    int32_t sign = 1;
    uint8_t integerDigits = 0U;
    uint8_t fractionDigits = 0U;
    const char *p = *cursor;

    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        integerPart = integerPart * 10 + (int32_t)(*p - '0');
        if (integerPart > 10000) {
            return 0U;
        }
        integerDigits++;
        p++;
    }

    if (integerDigits == 0U) {
        return 0U;
    }

    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            if (fractionDigits >= 2U) {
                return 0U;
            }
            fractionPart =
                fractionPart * 10 + (int32_t)(*p - '0');
            fractionDigits++;
            p++;
        }
        if (fractionDigits == 0U) {
            return 0U;
        }
    }

    if (fractionDigits == 1U) {
        fractionPart *= 10;
    }

    *cursor = p;
    *value = sign * (integerPart * 100 + fractionPart);
    return 1U;
}

/*
 * 队友视觉代码的数据格式：
 *   B,3.25,0.86\r\n   有效球位置，单位cm
 *   B,X,0.00\r\n      当前没有识别到球
 *
 * 位置保留两位小数。0.01 cm正好等于0.1 mm，因此转换后的
 * 百分之一厘米整数可以直接存入x_tenth_mm。
 */
static uint8_t ParseFrame(const char *line, BallVisionFrame *frame)
{
    const char *p;
    int32_t positionHundredthsCm;
    int32_t confidenceHundredths;

    if ((line[0] != 'B') || (line[1] != ',')) {
        return 0U;
    }

    p = &line[2];
    if (*p == 'X') {
        p++;
        if ((*p++ != ',') ||
            (ParseFixedHundredths(
                &p, &confidenceHundredths) == 0U) ||
            (*p != '\0') ||
            (confidenceHundredths < 0) ||
            (confidenceHundredths > 100)) {
            return 0U;
        }
        frame->valid = 0U;
        frame->x_tenth_mm = 0;
        frame->confidence = 0U;
        return 1U;
    }

    if ((ParseFixedHundredths(
            &p, &positionHundredthsCm) == 0U) ||
        (*p++ != ',') ||
        (ParseFixedHundredths(
            &p, &confidenceHundredths) == 0U) ||
        (*p != '\0')) {
        return 0U;
    }

    if ((positionHundredthsCm < -2000) ||
        (positionHundredthsCm > 2000) ||
        (confidenceHundredths < 0) ||
        (confidenceHundredths > 100)) {
        return 0U;
    }

    frame->valid = 1U;
    frame->x_tenth_mm = (int16_t)positionHundredthsCm;
    frame->confidence = (uint8_t)confidenceHundredths;
    return 1U;
}

/* Teammate touch-screen format: T,target_cm\r\n */
static uint8_t ParseTargetFrame(const char *line, int16_t *targetTenthMm)
{
    const char *p;
    int32_t targetHundredthsCm;
    int32_t minimum = (int32_t)(ITEM6_TARGET_MIN_CM * 100.0f);
    int32_t maximum = (int32_t)(ITEM6_TARGET_MAX_CM * 100.0f);

    if ((line[0] != 'T') || (line[1] != ',')) {
        return 0U;
    }

    p = &line[2];
    if ((ParseFixedHundredths(
            &p, &targetHundredthsCm) == 0U) ||
        (*p != '\0') ||
        (targetHundredthsCm < -2000) ||
        (targetHundredthsCm > 2000)) {
        return 0U;
    }

    if (targetHundredthsCm < minimum) {
        targetHundredthsCm = minimum;
    } else if (targetHundredthsCm > maximum) {
        targetHundredthsCm = maximum;
    }
    *targetTenthMm = (int16_t)targetHundredthsCm;
    return 1U;
}

void BallVision_Init(void)
{
    gRxLength = 0U;
    gLineReady = 0U;
    gFrame.sequence = 0U;
    gFrame.valid = 0U;
    gFrame.x_tenth_mm = 0;
    gFrame.confidence = 0U;
    gFrame.receive_ms = 0U;
    gGoodFrames = 0U;
    gBadFrames = 0U;
    gTargetValid = 0U;
    gTargetTenthMm = 0;
    gTargetVersion = 0U;
    gNextSequence = 0U;
    gLastMeasurementMs = 0U;

    DL_UART_Main_enableInterrupt(
        BALL_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(BALL_UART_IRQN);
    NVIC_EnableIRQ(BALL_UART_IRQN);
}

void BallVision_Update(uint32_t now_ms)
{
    BallVisionFrame parsed;
    int16_t parsedTarget;
    char localLine[RX_LINE_SIZE];
    uint8_t i;

    DrainUart();
    if (gLineReady == 0U) {
        return;
    }

    __disable_irq();
    for (i = 0U; i <= gRxLength; i++) {
        localLine[i] = gRxLine[i];
    }
    gRxLength = 0U;
    gLineReady = 0U;
    __enable_irq();

    if (ParseFrame(localLine, &parsed) != 0U) {
        gGoodFrames++;
        if (parsed.valid != 0U) {
            parsed.receive_ms = now_ms;

            /*
             * K230在两次YOLO推理之间会重复发送同一坐标。
             * 相同坐标只刷新超时，不制造一帧假的“零速度”测量。
             */
            if ((gFrame.valid == 0U) ||
                (parsed.x_tenth_mm != gFrame.x_tenth_mm) ||
                ((uint32_t)(now_ms - gLastMeasurementMs) >=
                 VISION_STATIONARY_REFRESH_MS)) {
                gNextSequence++;
                gLastMeasurementMs = now_ms;
            }
            parsed.sequence = gNextSequence;
            gFrame = parsed;
        }
    } else if (ParseTargetFrame(localLine, &parsedTarget) != 0U) {
        gGoodFrames++;
        if ((gTargetValid == 0U) ||
            (parsedTarget != gTargetTenthMm)) {
            gTargetTenthMm = parsedTarget;
            gTargetVersion++;
        }
        gTargetValid = 1U;
    } else {
        gBadFrames++;
    }
}

uint8_t BallVision_GetFrame(BallVisionFrame *frame)
{
    if (frame == 0) {
        return 0U;
    }
    *frame = gFrame;
    return 1U;
}

uint8_t BallVision_IsFresh(uint32_t now_ms, uint32_t timeout_ms)
{
    return (uint8_t)((gFrame.valid != 0U) &&
                     ((uint32_t)(now_ms - gFrame.receive_ms) <= timeout_ms));
}

float BallVision_GetPositionCm(void)
{
    /* 0.1 mm -> cm，除以100。 */
    return (float)gFrame.x_tenth_mm / 100.0f;
}

uint8_t BallVision_HasTarget(void)
{
    return gTargetValid;
}

float BallVision_GetTargetCm(void)
{
    return (float)gTargetTenthMm / 100.0f;
}

uint32_t BallVision_GetTargetVersion(void)
{
    return gTargetVersion;
}

uint32_t BallVision_GetGoodFrameCount(void)
{
    return gGoodFrames;
}

uint32_t BallVision_GetBadFrameCount(void)
{
    return gBadFrames;
}

void BALL_UART_ISR(void)
{
    if (DL_UART_Main_getPendingInterrupt(BALL_UART_INST) ==
        DL_UART_MAIN_IIDX_RX) {
        DrainUart();
    }
}
