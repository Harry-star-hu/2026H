#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "Delay.h"
#include "JY61P.h"

/* 这些变量由 UART 中断写入、主循环读取，因此都要加 volatile。 */
uint8_t RollL, RollH, PitchL, PitchH, YawL, YawH, VL, VH, SUM;
volatile float Pitch = 0.0f;
volatile float Roll = 0.0f;
volatile float Yaw = 0.0f;

/* OLED 最下一行显示这两个数，用来定位串口问题。 */
volatile uint32_t JY61P_RxByteCount = 0U;
volatile uint32_t JY61P_ValidAngleFrameCount = 0U;

#define WAIT_HEADER1 0U
#define WAIT_HEADER2 1U
#define RECEIVE_DATA 2U

static uint8_t rx_state = WAIT_HEADER1;
static uint8_t received_data[9];
static uint8_t data_index = 0U;

void Serial_JY61P_Zero_Yaw(void)
{
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0xFFU);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x69U);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x88U);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0xB5U);
    Delay_ms(100U);

    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0xFFU);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x01U);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x04U);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x00U);
    Delay_ms(100U);

    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0xFFU);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0xAAU);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x00U);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x00U);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, 0x00U);
}

/* 把一个 UART 字节送入 JY61P 解帧器。
 * 中断接收和轮询接收共用此函数，保证两种方式的处理完全相同。 */
static void jy61p_process_byte(uint8_t uart_data)
{
    ++JY61P_RxByteCount;

    switch (rx_state) {
    case WAIT_HEADER1:
        if (uart_data == 0x55U) {
            rx_state = WAIT_HEADER2;
        }
        break;

    case WAIT_HEADER2:
        if (uart_data == 0x53U) {
            rx_state = RECEIVE_DATA;
            data_index = 0U;
        } else if (uart_data == 0x55U) {
            /* 连续收到 0x55 时，不丢掉可能的新帧头。 */
            rx_state = WAIT_HEADER2;
        } else {
            rx_state = WAIT_HEADER1;
        }
        break;

    case RECEIVE_DATA:
        received_data[data_index++] = uart_data;
        if (data_index >= 9U) {
            uint8_t calculated_sum;

            RollL = received_data[0];
            RollH = received_data[1];
            PitchL = received_data[2];
            PitchH = received_data[3];
            YawL = received_data[4];
            YawH = received_data[5];
            VL = received_data[6];
            VH = received_data[7];
            SUM = received_data[8];

            calculated_sum = (uint8_t)(0x55U + 0x53U + RollL + RollH +
                                       PitchL + PitchH + YawL + YawH + VL + VH);

            if (calculated_sum == SUM) {
                /* JY61P 的角度数据本来就是 int16_t 有符号数。 */
                Roll  = (float)(int16_t)(((uint16_t)RollH  << 8) | RollL)  * 180.0f / 32768.0f;
                Pitch = (float)(int16_t)(((uint16_t)PitchH << 8) | PitchL) * 180.0f / 32768.0f;
                Yaw   = (float)(int16_t)(((uint16_t)YawH   << 8) | YawL)   * 180.0f / 32768.0f;
                ++JY61P_ValidAngleFrameCount;
            }

            rx_state = WAIT_HEADER1;
        }
        break;

    default:
        rx_state = WAIT_HEADER1;
        break;
    }
}

/* JY61P 的 0x53 角度帧：55 53 RollL RollH PitchL PitchH YawL YawH VL VH SUM */
void UART_JY61P_INST_IRQHandler(void)
{
    jy61p_process_byte(DL_UART_Main_receiveData(UART_JY61P_INST));
}

/* 轮询兜底：即使 UART 接收中断没有被正确打开，也会读取 PA22 到来的字节。 */
void JY61P_Poll(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_JY61P_INST)) {
        jy61p_process_byte(DL_UART_Main_receiveData(UART_JY61P_INST));
    }
}
