#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_ball_control.h"
#include "app_chassis_control.h"
#include "app_gyro_control.h"
#include "app_line_follow.h"
#include "app_task_mobile_ball.h"
#include "app_task_static_ball.h"
#include "interrupt.h"

/* Hardware substitutes; ball control and task code are linked unchanged. */
static uint32_t now_ms = 1000U;
static uint16_t servo_position;
static float planned_accel = 10.0f;

uint32_t Get_Tick(void) { return now_ms; }
void DVE_HTS25L_Init(void) {}
bool DVE_HTS25L_SetPosition(uint16_t pos, uint16_t time_ms)
{
    (void)time_ms;
    servo_position = pos;
    return true;
}
uint16_t DVE_HTS25L_GetCommandPosition(void) { return servo_position; }
bool APP_GyroControl_GetLatestAccel(APP_GyroAccelData_t *data)
{
    memset(data, 0, sizeof(*data));
    return false;
}
void APP_ChassisControl_Stop(void) {}
void APP_ChassisControl_Start(void) {}
void APP_ChassisControl_SetBallPriorityMode(bool enable) { (void)enable; }
void APP_ChassisControl_SetTargetBaseSpeed(float speed) { (void)speed; }
void APP_ChassisControl_RequestSoftStop(void) {}
bool APP_ChassisControl_IsSoftStopped(void) { return true; }
float APP_ChassisControl_GetPlannedAccelCmps2(void) { return planned_accel; }
void APP_ChassisControl_GetState(APP_ChassisControlState_t *state)
{
    memset(state, 0, sizeof(*state));
    state->is_running = true;
    state->planned_base_speed_cm_s = 24.0f;
}
void APP_LineFollow_SetErrorFilterAlpha(float alpha) { (void)alpha; }
void APP_LineFollow_GetState(APP_LineFollowState_t *state)
{
    memset(state, 0, sizeof(*state));
}

static APP_BallControlStatus_t status(void)
{
    APP_BallControlStatus_t ball;
    APP_BallControl_GetStatus(&ball);
    return ball;
}

static void check_gains(float kv, float ka, bool enabled)
{
    APP_BallControlStatus_t ball = status();
    assert(fabsf(ball.motion_velocity_gain - kv) < 0.00001f);
    assert(fabsf(ball.motion_acceleration_gain - ka) < 0.00001f);
    assert(ball.motion_compensation_enabled == enabled);
}

static void measure(float x_cm)
{
    APP_K230BallFrame_t frame;
    now_ms += 33U;
    frame.x_cm = x_cm;
    frame.tick_ms = now_ms;
    APP_BallControl_UpdateFromVision(&frame);
}

static void check_disabled_output(void)
{
    APP_BallControlStatus_t ball = status();
    assert(ball.estimated_velocity_cm_s > 0.0f);
    assert(ball.velocity_feedback_deg == 0.0f);
    assert(ball.vehicle_accel_feedforward_deg == 0.0f);
}

int main(void)
{
    APP_BallControl_Init();
    APP_TaskStaticBall_Init();
    APP_TaskMobileBall_Init();
    check_gains(0.5f, 0.205f, true);

    /* Static control keeps tuned gains but suppresses their actual output. */
    APP_BallControl_SetMotionCompensation(0.45f, 0.123f);
    APP_TaskStaticBall_Start();
    check_gains(0.45f, 0.123f, false);
    APP_BallControl_StartHold(0.0f);
    measure(0.0f);
    measure(0.1f);
    check_disabled_output();

    /* Serial-style tuning during static mode must not implicitly enable it. */
    APP_BallControl_SetMotionCompensation(0.65f, 0.15f);
    check_gains(0.65f, 0.15f, false);
    measure(0.2f);
    check_disabled_output();
    APP_TaskStaticBall_Stop();
    check_gains(0.65f, 0.15f, true);

    for (unsigned mode = 0U; mode < 3U; ++mode) {
        /* Direct task switching also works without an intermediate stop. */
        APP_TaskStaticBall_Start();
        check_gains(0.65f, 0.15f, false);
        APP_TaskMobileBall_Start((APP_MobileMode_t)mode, mode == 2U ? 5.0f : 0.0f);
        check_gains(0.65f, 0.15f, true);
        measure(0.0f);
        APP_TaskMobileBall_Task(now_ms); // Valid vision enters position hold.
        measure(0.0f);
        measure(0.1f);
        APP_BallControlStatus_t ball = status();
        assert(ball.estimated_velocity_cm_s > 0.0f);
        assert(fabsf(ball.velocity_feedback_deg +
                     0.65f * ball.estimated_velocity_cm_s) < 0.00001f);
        assert(fabsf(ball.vehicle_accel_feedforward_deg +
                     0.15f * planned_accel) < 0.00001f);
        APP_TaskMobileBall_Stop();
    }

    /* Deliberately tuning both gains to zero must remain valid across tasks. */
    APP_BallControl_SetMotionCompensation(0.0f, 0.0f);
    APP_TaskStaticBall_Start();
    APP_TaskMobileBall_Start(APP_MOBILE_MODE_AB_CENTER, 0.0f);
    check_gains(0.0f, 0.0f, true);

    puts("PASS: stored gains survive static/mobile switches; output gating works in all three mobile modes");
    return 0;
}
