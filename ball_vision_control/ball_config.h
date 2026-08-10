#ifndef BALL_CONFIG_H_
#define BALL_CONFIG_H_

#include "ti_msp_dl_config.h"

/*
 * 只需要在这个文件里修改引脚名和控制参数。
 * 下列名字需要与 SysConfig 生成的 ti_msp_dl_config.h 一致。
 */

/* 摆杆步进电机：STEP、DIR、EN。 */
#define ROD_STEP_PORT                GPIO_GIMBAL_PITCH_STEP_PORT
#define ROD_STEP_PIN                 GPIO_GIMBAL_PITCH_STEP_PIN_6_PIN
#define ROD_DIR_PORT                 GPIO_GIMBAL_PITCH_DIR_PORT
#define ROD_DIR_PIN                  GPIO_GIMBAL_PITCH_DIR_PIN_7_PIN
#define ROD_EN_PORT                  GPIO_GIMBAL_PITCH_EN_PORT
#define ROD_EN_PIN                   GPIO_GIMBAL_PITCH_EN_PIN_10_PIN

/* 20 kHz 周期中断，用软件产生 STEP 脉冲。 */
#define ROD_TIMER_INST               TIMER_GIMBAL_INST
#define ROD_TIMER_IRQN               TIMER_GIMBAL_INST_INT_IRQN
#define ROD_TIMER_ISR                TIMER_GIMBAL_INST_IRQHandler
#define ROD_TIMER_HZ                 20000U

/*
 * 视觉板使用UART2发送，MSPM0的SysConfig实例名称仍是UART_1。
 * 只接收视觉板TX；两块板必须共地。串口配置为115200、8N1。
 */
#define BALL_UART_INST  UART_0_INST
#define BALL_UART_IRQN  UART_0_INST_INT_IRQN
#define BALL_UART_ISR   UART_0_INST_IRQHandler

/* ---------- 步进电机 ---------- */
#define ROD_ENABLE_ACTIVE_LOW        1U
#define ROD_POSITIVE_DIR_HIGH        1U
/* 正速度使摆杆右端上升；相反就把符号改为 -1。 */
#define ROD_MOTOR_SIGN               (-1)
#define ROD_MAX_STEP_RATE            800.0f

/*
 * 软件角度只由S3水平零点之后的脉冲数估算，不再使用WDD35D4。
 * 默认35.6 step/°按“200步电机、1/16细分、机构约4:1”估算。
 * 必须根据实际驱动器细分和连杆几何标定；首次测试不要放球。
 */
#define ROD_STEPS_PER_DEGREE         56.0f
#define ROD_SOFT_LIMIT_STEPS         250L

/* ---------- 串级控制参数 ---------- */
/* 外环：视觉钢球位置(cm) -> 摆杆目标角度(°)。 */
#define BALL_POSITION_KP             0.30f
#define BALL_POSITION_KD             0.45f
/* 球偏右时应抬高右端；若第一次测试越控越远，改成 -1.0f。 */
#define BALL_OUTER_LOOP_SIGN         1.0f
#define BALL_SPEED_FILTER_ALPHA      0.25f
#define BALL_SPEED_STALE_DECAY       0.80f
#define ROD_TARGET_MAX_DEG           2.5f
#define ROD_TARGET_SLEW_DEG          0.12f

/*
 * Task 4/5/6 moving-car hold controller.
 * Keep this separate from Task 3: the open-loop Task 3 parameters below are
 * not changed.  The smaller angle limit and the speed term brake the ball
 * before it crosses O instead of driving it from end to end.
 */
#define HOLD_POSITION_KP             0.15f
#define HOLD_SPEED_KD                0.65f
#define HOLD_CENTER_POSITION_KP      0.12f
#define HOLD_CENTER_SPEED_KD         0.70f
#define HOLD_CENTER_FEEDBACK_MAX_DEG 1.20f
#define HOLD_SPEED_FAST_FILTER_ALPHA 0.60f
#define HOLD_SPEED_FAST_STALE_DECAY  0.65f
#define HOLD_TRANSIENT_SLEW_DEG       0.20f
#define HOLD_STATIC_I_RAMP_DEG_S     0.60f
#define HOLD_POSITION_I_LIMIT_DEG    1.20f
/*
 * 脱困只能在钢球确实静止时介入。
 * 观察窗口内无论向哪个方向移动，只要位移达到阈值都判定为“正在运动”，
 * 立即清除脱困角，避免钢球越过 O 点后被错误地继续加速。
 */
#define HOLD_MOTION_CONFIRM_CM       0.12f
#define HOLD_STILL_WINDOW_MS         350U
#define HOLD_MAX_ANGLE_DEG           1.60f
/* 机构正、负方向静摩擦不同：只提高 OLED 中正 T 的脱困角。 */
#define HOLD_NEGATIVE_MIN_MOVE_ANGLE_DEG 1.20f
#define HOLD_POSITIVE_MIN_MOVE_ANGLE_DEG 1.70f
#define HOLD_POSITIVE_STATIC_MAX_DEG     1.90f
#define HOLD_ENTER_CENTER_CM         0.55f
#define HOLD_LEAVE_CENTER_CM         0.90f
#define HOLD_CENTER_SPEED_CM_S       0.80f
#define HOLD_STILL_SPEED_CM_S        0.45f
#define HOLD_SPEED_LIMIT_CM_S        5.00f

/* Vehicle longitudinal-motion compensation shared by Task 4/5/6. */
#define HOLD_TRANSIENT_MAX_ANGLE_DEG    3.50f
#define HOLD_CAR_START_FF_DEG         (-3.40f)
#define HOLD_CAR_CRUISE_FF_DEG        (-0.05f)
#define HOLD_CAR_START_FF_MS            700U
#define HOLD_CAR_FEEDFORWARD_MAX_DEG    3.50f
#define HOLD_CAR_SPEED_FILTER_ALPHA     0.30f
#define HOLD_CAR_ACCEL_FILTER_ALPHA     0.25f
#define HOLD_CAR_ACCEL_GAIN_DEG         0.00f
#define HOLD_CAR_ACCEL_DEADBAND         0.03f

/*
 * Stop-capture controller for Tasks 4/5/6.  It is enabled when the vehicle
 * enters its PWM soft-stop and remains active after wheel PWM reaches zero.
 * The stronger speed term catches forward ball inertia before position error
 * grows large; normal driving and curve parameters are not changed.
 */
#define HOLD_STOP_POSITION_KP            0.28f
#define HOLD_STOP_SPEED_KD               1.50f
#define HOLD_STOP_MAX_ANGLE_DEG          3.50f
#define HOLD_STOP_FIXED_BACKWARD_ANGLE_DEG 3.50f
#define HOLD_STOP_RELEASE_ERROR_CM        5.00f
#define HOLD_STOP_RAW_REVERSE_SPEED_CM_S 0.15f
#define HOLD_STOP_STABLE_ERROR_CM        0.35f
#define HOLD_STOP_STABLE_SPEED_CM_S      0.15f
#define HOLD_STOP_STABLE_MS               400U
#define HOLD_STOP_MAX_TIME_MS            5000U

/*
 * Task 6 independent ball-hold parameters.
 *
 * These defaults intentionally match Tasks 4/5.  Tune only this ITEM6_HOLD_*
 * block when the ball moves as the Task 6 vehicle starts; Tasks 4/5 will not
 * change.  Keep ITEM6_HOLD_TRANSIENT_MAX_ANGLE_DEG at least as large as
 * ITEM6_HOLD_CAR_FEEDFORWARD_MAX_DEG.
 */
#define ITEM6_HOLD_POSITION_KP                 0.15f
#define ITEM6_HOLD_SPEED_KD                    0.65f
#define ITEM6_HOLD_CENTER_POSITION_KP          0.12f
#define ITEM6_HOLD_CENTER_SPEED_KD             0.70f
#define ITEM6_HOLD_CENTER_FEEDBACK_MAX_DEG     1.20f
#define ITEM6_HOLD_SPEED_FAST_FILTER_ALPHA     0.60f
#define ITEM6_HOLD_SPEED_FAST_STALE_DECAY      0.65f
#define ITEM6_HOLD_TRANSIENT_SLEW_DEG          0.20f
#define ITEM6_HOLD_STATIC_I_RAMP_DEG_S         0.60f
#define ITEM6_HOLD_POSITION_I_LIMIT_DEG        1.40f
#define ITEM6_HOLD_MOTION_CONFIRM_CM           0.20f
#define ITEM6_HOLD_STILL_WINDOW_MS              300U
#define ITEM6_HOLD_MAX_ANGLE_DEG               1.60f
#define ITEM6_HOLD_NEGATIVE_MIN_MOVE_ANGLE_DEG 1.35f
#define ITEM6_HOLD_POSITIVE_MIN_MOVE_ANGLE_DEG 1.70f
#define ITEM6_HOLD_POSITIVE_STATIC_MAX_DEG     1.90f
#define ITEM6_HOLD_ENTER_CENTER_CM             0.55f
#define ITEM6_HOLD_LEAVE_CENTER_CM             0.90f
#define ITEM6_HOLD_CENTER_SPEED_CM_S           0.80f
#define ITEM6_HOLD_SPEED_LIMIT_CM_S            5.00f
#define ITEM6_HOLD_TRANSIENT_MAX_ANGLE_DEG     3.50f

/* Task 6 vehicle-motion feedforward. */
#define ITEM6_HOLD_CAR_START_FF_DEG           (-3.40f)
#define ITEM6_HOLD_CAR_CRUISE_FF_DEG          (-0.20f)
#define ITEM6_HOLD_CAR_START_FF_MS             1000U
#define ITEM6_HOLD_CAR_FEEDFORWARD_MAX_DEG     3.50f
#define ITEM6_HOLD_CAR_SPEED_FILTER_ALPHA      0.30f
#define ITEM6_HOLD_CAR_ACCEL_FILTER_ALPHA      0.25f
#define ITEM6_HOLD_CAR_ACCEL_GAIN_DEG          0.00f
#define ITEM6_HOLD_CAR_ACCEL_DEADBAND          0.03f

/* Task 6 independent wheel-stop ball capture. */
#define ITEM6_HOLD_STOP_POSITION_KP             0.28f
#define ITEM6_HOLD_STOP_SPEED_KD                1.50f
#define ITEM6_HOLD_STOP_MAX_ANGLE_DEG           3.50f
#define ITEM6_HOLD_STOP_FIXED_BACKWARD_ANGLE_DEG 3.50f
#define ITEM6_HOLD_STOP_RELEASE_ERROR_CM        5.00f
#define ITEM6_HOLD_STOP_RAW_REVERSE_SPEED_CM_S  0.15f
#define ITEM6_HOLD_STOP_STABLE_ERROR_CM         0.35f
#define ITEM6_HOLD_STOP_STABLE_SPEED_CM_S       0.15f
#define ITEM6_HOLD_STOP_STABLE_MS                400U
#define ITEM6_HOLD_STOP_MAX_TIME_MS             5000U

/*
 * Task 3 staged motion:
 *   0 -> +5 cm: fixed negative tilt, then a gentler approach.
 *   +5 -> -5 cm: fixed positive tilt, then closed-loop braking/holding.
 * These parameters only affect BallControl_StartStaticSequence().
 */
#define STATIC_PLUS_SWITCH_CM        4.20f
#define STATIC_POSITION_DEADBAND_CM  0.30f
#define STATIC_MOVING_SPEED_CM_S     0.40f
#define STATIC_MOVE_CONFIRM_CM       0.25f
#define STATIC_PROGRESS_CONFIRM_CM   0.10f
#define STATIC_NO_PROGRESS_MS        300U

/* Short breakaway pulse followed by a level observation interval. */
#define STATIC_KICK_START_DEG        1.40f
#define STATIC_KICK_STEP_DEG         0.30f
#define STATIC_KICK_MAX_DEG          2.50f
#define STATIC_KICK_NEAR_MAX_DEG     2.50f
#define STATIC_KICK_NEAR_DISTANCE_CM 1.50f
#define STATIC_KICK_TIME_MS          260U
#define STATIC_KICK_NEAR_TIME_MS     120U
#define STATIC_COAST_TIME_MS         140U

/* Motion and predictive braking after the ball starts rolling. */
#define STATIC_RUN_ANGLE_DEG         0.50f
#define STATIC_RETURN_START_DEG      1.80f
#define STATIC_RETURN_RUN_ANGLE_DEG  0.45f
#define STATIC_BRAKE_BASE_ANGLE_DEG  0.80f
#define STATIC_BRAKE_SPEED_GAIN      0.12f
#define STATIC_BRAKE_MAX_ANGLE_DEG   2.20f
#define STATIC_STOP_DISTANCE_BASE_CM 0.45f
#define STATIC_STOP_DISTANCE_GAIN_S  0.32f
#define STATIC_RETURN_STOP_BASE_CM   0.45f
#define STATIC_RETURN_STOP_GAIN_S    0.35f

/* Endpoint capture: enter earlier at higher speed, then use full PD braking. */
#define ENDPOINT_CAPTURE_BASE_CM      1.80f
#define ENDPOINT_CAPTURE_SPEED_GAIN_S 0.50f
#define ENDPOINT_CAPTURE_MAX_CM       4.00f
#define ENDPOINT_POSITION_KP          0.40f
#define ENDPOINT_SPEED_KD             0.70f
#define ENDPOINT_MAX_ANGLE_DEG        2.20f
#define ENDPOINT_PLUS_STABLE_MS       200U

/*
 * Task 3 hybrid control:
 *   0 -> +5 cm: one directly tunable open-loop pipe angle.
 *   +5 -> -5 cm: independent position/speed closed loop.
 */
#define TASK3_OPEN_SWITCH_CM              0.60f
#define TASK3_OPEN_TILT_DEG               2.00f

#define TASK3_CLOSED_TARGET_CM           (-7.80f)

/*
 * Task 3 independent closed loop.  Initial values match Tasks 4/5/6, but
 * changing any TASK3_CLOSED_* value below does not affect those tasks.
 */
#define TASK3_CLOSED_POSITION_KP               0.33f
#define TASK3_CLOSED_SPEED_KD                  0.30f

#define TASK3_CLOSED_CENTER_POSITION_KP        0.12f
#define TASK3_CLOSED_CENTER_SPEED_KD           0.50f
#define TASK3_CLOSED_CENTER_MAX_ANGLE_DEG      1.20f
#define TASK3_CLOSED_SPEED_FILTER_ALPHA        0.80f
#define TASK3_CLOSED_SPEED_STALE_DECAY         0.40f
#define TASK3_CLOSED_SPEED_LIMIT_CM_S          5.00f
#define TASK3_CLOSED_SLEW_DEG                  0.16f

#define TASK3_CLOSED_STATIC_I_RAMP_DEG_S       0.60f
#define TASK3_CLOSED_I_LIMIT_DEG               1.60f
#define TASK3_CLOSED_MOTION_CONFIRM_CM         0.12f
#define TASK3_CLOSED_STILL_WINDOW_MS            350U
#define TASK3_CLOSED_MAX_ANGLE_DEG              1.20f
#define TASK3_CLOSED_NEGATIVE_MIN_MOVE_DEG      1.20f
#define TASK3_CLOSED_POSITIVE_MIN_MOVE_DEG      1.80f
#define TASK3_CLOSED_POSITIVE_STATIC_MAX_DEG    2.50f
#define TASK3_CLOSED_ENTER_CENTER_CM            0.25f
#define TASK3_CLOSED_LEAVE_CENTER_CM            0.35f
#define TASK3_CLOSED_CENTER_SPEED_CM_S          0.80f
#define TASK3_CLOSED_COMMAND_MAX_ANGLE_DEG      3.50f

/*
 * Task 3 endpoint acceptance.  The ball must first reach/cross -5 cm.  Only
 * after that first crossing may control capture inside +/-0.85 cm and keep
 * the pipe horizontal inside the allowed +/-1.00 cm band.
 */
#define TASK3_CLOSED_ACCEPT_ERROR_CM       0.85f
#define TASK3_CLOSED_ACCEPT_RELEASE_CM     1.00f
#define TASK3_CLOSED_ACCEPT_SPEED_CM_S     0.30f

#define TASK3_CLOSED_FINISH_ERROR_CM       1.00f
#define TASK3_CLOSED_FINISH_SPEED_CM_S     0.60f
#define TASK3_CLOSED_FINISH_TIME_MS         500U

#define STATIC_FINISH_ERROR_CM       0.30f
#define STATIC_FINISH_SPEED_CM_S     0.50f
#define STATIC_FINISH_TIME_MS        800U

/* 内环：软件估算摆杆角度(°) -> 步进速度(step/s)。 */
#define ROD_ANGLE_KP                 300.0f
#define ROD_ANGLE_KI                 0.0f
#define ROD_ANGLE_KD                 1.5f
#define ROD_RATE_FILTER_ALPHA        0.20f
#define ROD_ANGLE_DEADBAND_DEG       0.06f
#define ROD_INTEGRAL_LIMIT           2.0f

/* 安全参数。 */
#define ROD_SAFE_ANGLE_DEG           7.0f
#define BALL_SAFE_POSITION_CM        11.5f
#define BALL_OUTSIDE_CONFIRM_FRAMES  3U
#define VISION_TIMEOUT_MS            1000U
#define VISION_STATIONARY_REFRESH_MS 100U
#define CONTROL_PERIOD_MS            10U

/* Task 6 live target sent by the vision touch screen. */
#define ITEM6_TARGET_MIN_CM          (-10.0f)
#define ITEM6_TARGET_MAX_CM            10.0f
#define ITEM6_READY_ERROR_CM            0.50f
#define ITEM6_READY_SPEED_CM_S          0.50f
#define ITEM6_READY_STABLE_MS          500U

/* 0: Task 4 runs the car and ball hold controller together. */
#define BALL_HOLD_BENCH_TEST          0U



#endif