#include "control.h"

#define CONTROL_PERIOD_MS          10U
#define LINE_STABLE_MS             40U
#define ARC_EXIT_STABLE_MS         30U
#define STRAIGHT_MIN_MS           400U
#define ARC_MIN_MS                700U
#define ARC_MIN_YAW_DEG         135.0f
#define SENSOR_TIMEOUT_MS         500U
#define STRAIGHT_SPEED_CMD       53.0f
#define HEADING_KP                1.35f
#define HEADING_RATE_KD           0.025f
#define HEADING_RATE_FILTER       0.25f
#define HEADING_CORRECT_START     0.90f
#define HEADING_CORRECT_STOP      0.30f
#define HEADING_TURN_LIMIT       12.0f
#define HEADING_TURN_SLEW         1.50f
#define MIN_WHEEL_CMD            35.0f
#define MAX_WHEEL_CMD            90.0f
#define PRETURN_TOLERANCE_DEG     2.0f
#define PRETURN_STABLE_MS         80U
#define PRETURN_TIMEOUT_MS      2500U
#define PRETURN_SLOW_ZONE_DEG     8.0f
#define PRETURN_SLOW_CMD         42.0f
#define PRETURN_FAST_CMD         52.0f
#define MODE_MENU_GUARD_MS       2000U

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
static uint8_t s_preturn_active = 0;
static uint8_t s_heading_correction_active = 0;
static uint32_t s_preturn_stable_ms = 0;
static float s_initial_yaw = 0.0f;
static float s_target_yaw = 0.0f;
static float s_arc_start_yaw = 0.0f;
static float s_heading_error_last = 0.0f;
static float s_heading_error_rate = 0.0f;
static float s_heading_turn_command = 0.0f;

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

static float SlewToward(float current, float target, float step)
{
    if (target > (current + step)) return current + step;
    if (target < (current - step)) return current - step;
    return target;
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
    s_heading_error_rate = 0.0f;
    s_heading_turn_command = 0.0f;
    s_heading_correction_active = 0U;
    odometry_sum = 0;
    encoder_odometry_flag = 1;
    if ((state == COMP_STATE_ARC_FIRST) || (state == COMP_STATE_ARC_SECOND)) {
        s_arc_start_yaw = calibratedYaw;
        Line_Track_Reset();
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

static void DriveHeading(float target_yaw, uint32_t elapsed_ms)
{
    float error = SignedAngleError(calibratedYaw, target_yaw);
    float abs_error = (error < 0.0f) ? -error : error;
    float raw_error_rate;
    float requested_turn = 0.0f;
    float left;
    float right;

    if (elapsed_ms == 0U) {
        elapsed_ms = CONTROL_PERIOD_MS;
    } else if (elapsed_ms > 50U) {
        elapsed_ms = 50U;
    }

    raw_error_rate = SignedAngleError(error, s_heading_error_last) *
                     1000.0f / (float)elapsed_ms;
    s_heading_error_rate += HEADING_RATE_FILTER *
        (raw_error_rate - s_heading_error_rate);
    s_heading_error_last = error;

    if ((s_heading_correction_active == 0U) &&
        (abs_error >= HEADING_CORRECT_START)) {
        s_heading_correction_active = 1U;
    } else if ((s_heading_correction_active != 0U) &&
               (abs_error <= HEADING_CORRECT_STOP)) {
        s_heading_correction_active = 0U;
    }

    if (s_heading_correction_active != 0U) {
        requested_turn = HEADING_KP * error +
                         HEADING_RATE_KD * s_heading_error_rate;
        requested_turn = ClampFloat(requested_turn,
            -HEADING_TURN_LIMIT, HEADING_TURN_LIMIT);
    }

    /*
     * Never jump directly from a left correction to a right correction.
     * First unwind to zero; this removes the time-consuming left/right hunt.
     */
    if (((requested_turn > 0.0f) && (s_heading_turn_command < 0.0f)) ||
        ((requested_turn < 0.0f) && (s_heading_turn_command > 0.0f))) {
        requested_turn = 0.0f;
    }
    s_heading_turn_command = SlewToward(
        s_heading_turn_command, requested_turn, HEADING_TURN_SLEW);

    left = ClampFloat(STRAIGHT_SPEED_CMD - s_heading_turn_command,
                      MIN_WHEEL_CMD, MAX_WHEEL_CMD);
    right = ClampFloat(STRAIGHT_SPEED_CMD + s_heading_turn_command,
                       MIN_WHEEL_CMD, MAX_WHEEL_CMD);
    Set_PID_Motor(left, right, 0.0f);
}

static bool PreTurnToStraight(uint32_t elapsed_ms)
{
    float error;
    float abs_error;
    float turn;
    uint32_t step = (elapsed_ms > 50U) ? 50U : elapsed_ms;

    error = SignedAngleError(calibratedYaw, s_target_yaw);
    abs_error = (error < 0.0f) ? -error : error;

    if (abs_error <= PRETURN_TOLERANCE_DEG) {
        Motor_Stop(1);
        s_preturn_stable_ms += step;
        return (s_preturn_stable_ms >= PRETURN_STABLE_MS);
    }

    s_preturn_stable_ms = 0U;
    turn = (abs_error <= PRETURN_SLOW_ZONE_DEG) ?
           PRETURN_SLOW_CMD : PRETURN_FAST_CMD;
    if (error < 0.0f) {
        turn = -turn;
    }
    Set_PID_Motor(-turn, turn, 0.0f);
    return false;
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
            /*
             * The arc has already reversed the chassis. Lock the actual exit
             * heading and start the reverse straight immediately instead of
             * spending several seconds on a second in-place alignment.
             */
            s_target_yaw = Wrap360(calibratedYaw);
            s_preturn_active = 0U;
        } else {
            s_target_yaw = Wrap360(calibratedYaw +
                s_second_diagonal_deg[s_lap - 1U]);
            s_preturn_active = 1U;
            s_preturn_stable_ms = 0U;
        }
        EnterState(COMP_STATE_STRAIGHT_SECOND);
        return;
    }

    if ((s_mode == 4U) && (s_lap < 4U)) {
        s_lap++;
        s_target_yaw = Wrap360(calibratedYaw +
            s_first_diagonal_deg[s_lap - 1U]);
        s_preturn_active = 1U;
        s_preturn_stable_ms = 0U;
        EnterState(COMP_STATE_STRAIGHT_FIRST);
    } else {
        StopMission(false);
    }
}

uint8_t switch_mode(void)
{
    uint8_t selected = 1U;
    uint32_t menu_guard_start;

    OLED_Clear();
    OLED_ShowString(0, 0, "Select Mode:", 8, 1);
    OLED_ShowNum(88, 0, selected, 1, 16, 1);
    OLED_ShowString(0, 20, "K1/K3:+ K2:OK", 8, 1);
    OLED_Refresh();

    /* Keep the menu visible for at least two seconds. Each key has its own
     * one-shot state so a startup level on one input cannot lock the others. */
    menu_guard_start = Get_Time();
    while ((Get_Time() - menu_guard_start) < MODE_MENU_GUARD_MS) {
        Scheduler_Run();
        (void)Key1_State(KEY_MODE_ONE_TIME);
        (void)Key2_State(KEY_MODE_ONE_TIME);
        (void)Key3_State(KEY_MODE_ONE_TIME);
    }

    for (;;) {
        uint8_t next_k1;
        uint8_t next_k3;
        uint8_t confirm_k2;

        Scheduler_Run();
        next_k1 = Key1_State(KEY_MODE_ONE_TIME);
        next_k3 = Key3_State(KEY_MODE_ONE_TIME);
        confirm_k2 = Key2_State(KEY_MODE_ONE_TIME);

        if ((next_k1 == KEY_PRESS) || (next_k3 == KEY_PRESS)) {
            selected++;
            if (selected > 4U) selected = 1U;
            OLED_ShowNum(88, 0, selected, 1, 16, 1);
            OLED_Refresh();
            delay_ms(20);
        } else if (confirm_k2 == KEY_PRESS) {
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
    s_preturn_active = 0U;
    s_preturn_stable_ms = 0U;
    PID_Clear_Motor(MAX_MOTOR);
    Motor_Stop(1);

    /* Re-zero the fused heading at each mission start. */
    AngleOffsetCalc();
    s_initial_yaw = Wrap360(calibratedYaw);
    if ((s_mode == 1U) || (s_mode == 2U)) {
        s_target_yaw = s_initial_yaw;
    } else {
        s_target_yaw = Wrap360(calibratedYaw + s_first_diagonal_deg[0]);
        s_preturn_active = 1U;
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
        if (s_preturn_active != 0U) {
            if ((now - s_state_start_ms) >= PRETURN_TIMEOUT_MS) {
                StopMission(true);
            } else if (PreTurnToStraight(elapsed)) {
                CompetitionState straight_state = s_state;
                s_preturn_active = 0U;
                PID_Clear_Motor(MAX_MOTOR);
                EnterState(straight_state);
                s_heading_error_last =
                    SignedAngleError(calibratedYaw, s_target_yaw);
            }
            return;
        }

        DriveHeading(s_target_yaw, elapsed);
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
