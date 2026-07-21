#include "control.h"

#define CONTROL_PERIOD_MS          10U
#define LINE_STABLE_MS             40U
#define ARC_EXIT_STABLE_MS         60U
#define STRAIGHT_MIN_MS           400U
#define ARC_MIN_MS                700U
#define ARC_MIN_YAW_DEG         135.0f
#define SENSOR_TIMEOUT_MS         500U
#define STRAIGHT_SPEED_CMD       53.0f
#define HEADING_KP                1.20f
#define HEADING_KD                0.80f
#define HEADING_TURN_LIMIT       28.0f
#define MIN_WHEEL_CMD            20.0f
#define MAX_WHEEL_CMD            90.0f

/* These are the original Yahboom H-task test corrections. They are kept in
 * one place so track-side tuning is straightforward. */
static const float s_first_diagonal_deg[4] = {-35.0f, -31.5f, -29.0f, -30.3f};
static const float s_second_diagonal_deg[4] = {37.4f, 38.3f, 37.0f, 37.3f};

int encoder_odometry_flag = 0;
volatile int odometry_sum = 0;
volatile int RGB_BEEP_flag = 0;

static uint8_t s_mode = 0;
static uint8_t s_lap = 1;
static CompetitionState s_state = COMP_STATE_IDLE;
static uint32_t s_mission_start_ms = 0;
static uint32_t s_state_start_ms = 0;
static uint32_t s_last_control_ms = 0;
static uint32_t s_black_stable_ms = 0;
static uint32_t s_white_stable_ms = 0;
static uint32_t s_signal_end_ms = 0;
static uint8_t s_signal_active = 0;
static uint8_t s_seen_white = 0;
static float s_target_yaw = 0.0f;
static float s_arc_start_yaw = 0.0f;
static float s_heading_error_last = 0.0f;

static float Wrap360(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float SignedAngleError(float current, float target)
{
    float error = Wrap360(current) - Wrap360(target);
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return error;
}

static float AbsoluteAngleTravel(float current, float start)
{
    float value = SignedAngleError(current, start);
    return (value < 0.0f) ? -value : value;
}

static float ClampFloat(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void SignalStart(void)
{
    Buzzer_open_state();
    Control_RGB_ALL(Yellow_RGB);
    s_signal_active = 1U;
    s_signal_end_ms = Get_Time() + 150U;
}

static void SignalPoint(void)
{
    Buzzer_open_state();
    Control_RGB_ALL(Cyan_RGB);
    s_signal_active = 1U;
    s_signal_end_ms = Get_Time() + 100U;
}

static void SignalFinish(void)
{
    Buzzer_open_state();
    Control_RGB_ALL(Green_RGB);
    s_signal_active = 1U;
    s_signal_end_ms = Get_Time() + 300U;
}

static void SignalError(void)
{
    Buzzer_open_state();
    Control_RGB_ALL(Red_RGB);
    s_signal_active = 1U;
    s_signal_end_ms = Get_Time() + 500U;
}

static void SignalUpdate(uint32_t now)
{
    if ((s_signal_active != 0U) && ((int32_t)(now - s_signal_end_ms) >= 0)) {
        Buzzer_close_state();
        if (s_state != COMP_STATE_FINISHED && s_state != COMP_STATE_ERROR) {
            Control_RGB_ALL(OFF);
        }
        s_signal_active = 0U;
    }
}

static void EnterState(CompetitionState state)
{
    s_state = state;
    s_state_start_ms = Get_Time();
    s_black_stable_ms = 0U;
    s_white_stable_ms = 0U;
    s_seen_white = 0U;
    s_heading_error_last = 0.0f;
    odometry_sum = 0;
    encoder_odometry_flag = 1;
    if ((state == COMP_STATE_ARC_FIRST) || (state == COMP_STATE_ARC_SECOND)) {
        s_arc_start_yaw = calibratedYaw;
    }
}

static void StopMission(bool error)
{
    Motor_Stop(1);
    encoder_odometry_flag = 0;
    s_state = error ? COMP_STATE_ERROR : COMP_STATE_FINISHED;
    if (error) {
        SignalError();
    } else {
        SignalFinish();
    }
}

static void UpdateLineStability(bool black, uint32_t elapsed_ms)
{
    uint32_t step = (elapsed_ms > 50U) ? 50U : elapsed_ms;
    if (black) {
        s_black_stable_ms += step;
        s_white_stable_ms = 0U;
    } else {
        s_white_stable_ms += step;
        s_black_stable_ms = 0U;
        if (s_white_stable_ms >= LINE_STABLE_MS) {
            s_seen_white = 1U;
        }
    }
}

static void DriveHeading(float target_yaw)
{
    float error = SignedAngleError(calibratedYaw, target_yaw);
    float turn = HEADING_KP * error + HEADING_KD * (error - s_heading_error_last);
    float left;
    float right;

    s_heading_error_last = error;
    turn = ClampFloat(turn, -HEADING_TURN_LIMIT, HEADING_TURN_LIMIT);
    left = ClampFloat(STRAIGHT_SPEED_CMD - turn, MIN_WHEEL_CMD, MAX_WHEEL_CMD);
    right = ClampFloat(STRAIGHT_SPEED_CMD + turn, MIN_WHEEL_CMD, MAX_WHEEL_CMD);
    Set_PID_Motor(left, right, 0.0f);
}

static void OnStraightLineReached(void)
{
    SignalPoint();

    if ((s_mode == 1U) && (s_state == COMP_STATE_STRAIGHT_FIRST)) {
        StopMission(false);
        return;
    }

    if (s_state == COMP_STATE_STRAIGHT_FIRST) {
        EnterState(COMP_STATE_ARC_FIRST);
    } else {
        EnterState(COMP_STATE_ARC_SECOND);
    }
}

static void OnArcExitReached(void)
{
    SignalPoint();

    if (s_state == COMP_STATE_ARC_FIRST) {
        if (s_mode == 2U) {
            s_target_yaw = Wrap360(calibratedYaw);
        } else {
            s_target_yaw = Wrap360(calibratedYaw +
                s_second_diagonal_deg[s_lap - 1U]);
        }
        EnterState(COMP_STATE_STRAIGHT_SECOND);
        return;
    }

    if ((s_mode == 4U) && (s_lap < 4U)) {
        s_lap++;
        s_target_yaw = Wrap360(calibratedYaw +
            s_first_diagonal_deg[s_lap - 1U]);
        EnterState(COMP_STATE_STRAIGHT_FIRST);
    } else {
        StopMission(false);
    }
}

uint8_t switch_mode(void)
{
    uint8_t selected = 1U;

    OLED_Clear();
    OLED_ShowString(0, 0, "Select Mode:", 8, 1);
    OLED_ShowNum(88, 0, selected, 1, 16, 1);
    OLED_ShowString(0, 20, "K1:+  K2:OK", 8, 1);
    OLED_Refresh();

    for (;;) {
        uint8_t key;
        Scheduler_Run();
        key = KEY_Scan();
        if (key == KEY1_PRES) {
            selected++;
            if (selected > 4U) selected = 1U;
            OLED_ShowNum(88, 0, selected, 1, 16, 1);
            OLED_Refresh();
        } else if (key == KEY2_PRES) {
            OLED_ShowString(88, 20, "OK", 8, 1);
            OLED_Refresh();
            delay_ms(250);
            OLED_Clear();
            return selected;
        }
    }
}

void Competition_Init(uint8_t mode)
{
    s_mode = (mode >= 1U && mode <= 4U) ? mode : 1U;
    s_lap = 1U;
    s_mission_start_ms = Get_Time();
    s_last_control_ms = s_mission_start_ms;
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop(1);

    if ((s_mode == 1U) || (s_mode == 2U)) {
        s_target_yaw = Wrap360(calibratedYaw);
    } else {
        s_target_yaw = Wrap360(calibratedYaw + s_first_diagonal_deg[0]);
    }

    EnterState(COMP_STATE_STRAIGHT_FIRST);
    SignalStart();
}

void Competition_Update(void)
{
    uint32_t now = Get_Time();
    uint32_t elapsed = now - s_last_control_ms;
    bool black;

    SignalUpdate(now);
    IR_DATA();

    if ((s_state == COMP_STATE_IDLE) ||
        (s_state == COMP_STATE_FINISHED) ||
        (s_state == COMP_STATE_ERROR)) {
        return;
    }
    if (elapsed < CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now;

    if (IR_LastFrameAgeMs() > SENSOR_TIMEOUT_MS) {
        StopMission(true);
        return;
    }

    if (((s_mode == 4U) && ((now - s_mission_start_ms) > 240000U)) ||
        ((s_mode != 4U) && ((now - s_mission_start_ms) > 90000U))) {
        StopMission(true);
        return;
    }

    black = (LineCheck() == BLACK_IR);
    UpdateLineStability(black, elapsed);

    if ((s_state == COMP_STATE_STRAIGHT_FIRST) ||
        (s_state == COMP_STATE_STRAIGHT_SECOND)) {
        DriveHeading(s_target_yaw);
        if ((s_seen_white != 0U) &&
            (s_black_stable_ms >= LINE_STABLE_MS) &&
            ((now - s_state_start_ms) >= STRAIGHT_MIN_MS)) {
            OnStraightLineReached();
        }
        return;
    }

    Line_Tracke(1);
    if ((s_white_stable_ms >= ARC_EXIT_STABLE_MS) &&
        ((now - s_state_start_ms) >= ARC_MIN_MS) &&
        (AbsoluteAngleTravel(calibratedYaw, s_arc_start_yaw) >= ARC_MIN_YAW_DEG)) {
        OnArcExitReached();
    }
}

bool Competition_IsFinished(void)
{
    return (s_state == COMP_STATE_FINISHED);
}

uint8_t Competition_GetLap(void)
{
    return s_lap;
}

CompetitionState Competition_GetState(void)
{
    return s_state;
}
