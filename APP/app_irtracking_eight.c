#include "app_irtracking_eight.h"

/*
 * Sensor order viewed from the front of the car:
 * x1 (far left) ... x8 (far right).
 * Digital value 0 means black line, 1 means white background.
 */
u8 x1, x2, x3, x4, x5, x6, x7, x8;
int pid_output_IRR = 0;
u8 trun_flag = 0;
int8_t err = 0;

#define ARC_PID_KP             1.70f
#define ARC_PID_KD             0.65f
#define ARC_PID_TURN_LIMIT    36.0f
#define ARC_PID_BASE_SPEED    48.0f
#define ARC_LOST_ERROR        20.0f
#define ARC_MIN_WHEEL_SPEED   12.0f
#define ARC_MAX_WHEEL_SPEED   85.0f
#define ARC_STEERING_POLARITY  1.0f
#define ARC_COMMAND_SLEW       6.0f
#define ARC_LOST_COAST_CYCLES  4U

typedef struct {
    float abs_error;
    float base_speed;
    float differential_ratio;
} ArcLookupPoint;

/*
 * Continuous version of the table shown in the reference image.
 * Centered line: high speed and little differential.
 * Line near an edge: lower speed and a much larger wheel-speed difference.
 */
static const ArcLookupPoint s_arc_table[] = {
    { 0.0f, 70.0f, 0.00f },
    { 5.0f, 68.0f, 0.08f },
    {10.0f, 63.0f, 0.18f },
    {15.0f, 56.0f, 0.30f },
    {20.0f, 48.0f, 0.46f },
};

static const int8_t s_sensor_weights[IR_Num] =
    {-20, -15, -10, -5, 5, 10, 15, 20};

static float s_last_line_error = 0.0f;
static float s_pid_error_last = 0.0f;
static float s_last_left_command = 48.0f;
static float s_last_right_command = 48.0f;
static uint8_t s_line_lost_cycles = 0U;

static float ClampFloat(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float SlewToward(float current, float target, float step)
{
    if (target > (current + step)) return current + step;
    if (target < (current - step)) return current - step;
    return target;
}

void Copy_HD_Data(void)
{
    x1 = IR_Data_number[0];
    x2 = IR_Data_number[1];
    x3 = IR_Data_number[2];
    x4 = IR_Data_number[3];
    x5 = IR_Data_number[4];
    x6 = IR_Data_number[5];
    x7 = IR_Data_number[6];
    x8 = IR_Data_number[7];
}

int LineCheck(void)
{
    uint8_t i;

    IR_DATA();
    Copy_HD_Data();
    for (i = 0U; i < IR_Num; i++) {
        if (IR_Data_number[i] == 0U) {
            return BLACK_IR;
        }
    }
    return WHITE_IR;
}

/*
 * Returns a weighted line error and whether the pattern is suitable for the
 * primary lookup controller. A normal line should activate a contiguous group
 * of no more than four sensors.
 */
static uint8_t ReadLinePosition(float *line_error, uint8_t *active_count)
{
    int16_t weighted_sum = 0;
    int8_t first_active = -1;
    int8_t last_active = -1;
    uint8_t count = 0U;
    uint8_t i;
    uint8_t contiguous = 1U;

    for (i = 0U; i < IR_Num; i++) {
        if (IR_Data_number[i] == 0U) {
            weighted_sum += s_sensor_weights[i];
            if (first_active < 0) {
                first_active = (int8_t)i;
            }
            last_active = (int8_t)i;
            count++;
        }
    }

    *active_count = count;
    if (count == 0U) {
        *line_error = s_last_line_error;
        return 0U;
    }

    *line_error = (float)weighted_sum / (float)count;
    s_last_line_error = *line_error;

    for (i = (uint8_t)first_active; i <= (uint8_t)last_active; i++) {
        if (IR_Data_number[i] != 0U) {
            contiguous = 0U;
            break;
        }
    }

    return ((contiguous != 0U) && (count <= 4U)) ? 1U : 0U;
}

static void InterpolateArcTable(float abs_error,
                                float *base_speed,
                                float *ratio)
{
    uint8_t i;
    uint8_t count =
        (uint8_t)(sizeof(s_arc_table) / sizeof(s_arc_table[0]));

    abs_error = ClampFloat(abs_error, 0.0f,
                           s_arc_table[count - 1U].abs_error);
    for (i = 1U; i < count; i++) {
        if (abs_error <= s_arc_table[i].abs_error) {
            const ArcLookupPoint *low = &s_arc_table[i - 1U];
            const ArcLookupPoint *high = &s_arc_table[i];
            float span = high->abs_error - low->abs_error;
            float t = (abs_error - low->abs_error) / span;

            *base_speed =
                low->base_speed + t * (high->base_speed - low->base_speed);
            *ratio = low->differential_ratio +
                t * (high->differential_ratio - low->differential_ratio);
            return;
        }
    }

    *base_speed = s_arc_table[count - 1U].base_speed;
    *ratio = s_arc_table[count - 1U].differential_ratio;
}

/*
 * Backup line PD. It is used only for lost, overly wide, or non-contiguous
 * patterns; normal cornering is driven by the lookup table above.
 */
float Track_PID(int8_t actual_value)
{
    float line_error = (float)actual_value;
    float derivative = line_error - s_pid_error_last;
    float output =
        ARC_PID_KP * line_error + ARC_PID_KD * derivative;

    s_pid_error_last = line_error;
    return ClampFloat(output, -ARC_PID_TURN_LIMIT,
                      ARC_PID_TURN_LIMIT);
}

void Line_Track_Reset(void)
{
    s_last_line_error = 0.0f;
    s_pid_error_last = 0.0f;
    s_last_left_command = ARC_PID_BASE_SPEED;
    s_last_right_command = ARC_PID_BASE_SPEED;
    s_line_lost_cycles = 0U;
    pid_output_IRR = 0;
    err = 0;
}

void Line_Tracke(int state)
{
    float line_error;
    float base_speed;
    float ratio;
    float turn;
    float left;
    float right;
    uint8_t active_count;
    uint8_t table_pattern_valid;

    IR_DATA();
    Copy_HD_Data();
    if (state != 1) {
        return;
    }

    table_pattern_valid = ReadLinePosition(&line_error, &active_count);
    err = (int8_t)line_error;

    if (table_pattern_valid != 0U) {
        s_line_lost_cycles = 0U;
        InterpolateArcTable(AbsFloat(line_error), &base_speed, &ratio);
        turn = base_speed * ratio;
        if (line_error < 0.0f) {
            turn = -turn;
        }
        turn *= ARC_STEERING_POLARITY;
        left = base_speed + turn;
        right = base_speed - turn;
        s_pid_error_last = line_error;
    } else {
        /*
         * If all sensors are white, keep searching in the last known
         * direction. The state machine still decides whether this is the
         * legitimate end of the arc.
         */
        if (active_count == 0U) {
            s_line_lost_cycles++;
            if (s_line_lost_cycles <= ARC_LOST_COAST_CYCLES) {
                /*
                 * All-white commonly means the real arc exit. Unwind most of
                 * the previous differential before the state machine confirms
                 * the exit, instead of applying a final maximum turn.
                 */
                float average =
                    (s_last_left_command + s_last_right_command) * 0.5f;
                float difference =
                    (s_last_left_command - s_last_right_command) * 0.20f;
                left = average + difference;
                right = average - difference;
                goto apply_output;
            }
            if (line_error > 0.0f) {
                line_error = ARC_LOST_ERROR;
            } else if (line_error < 0.0f) {
                line_error = -ARC_LOST_ERROR;
            } else {
                /* No direction history: hold the previous differential. */
                Set_PID_Motor(s_last_left_command,
                              s_last_right_command, 0.0f);
                return;
            }
        } else {
            s_line_lost_cycles = 0U;
        }

        turn = Track_PID((int8_t)line_error);
        pid_output_IRR = (int)turn;
        left = ARC_PID_BASE_SPEED + turn;
        right = ARC_PID_BASE_SPEED - turn;
    }

apply_output:
    left = ClampFloat(left, ARC_MIN_WHEEL_SPEED, ARC_MAX_WHEEL_SPEED);
    right = ClampFloat(right, ARC_MIN_WHEEL_SPEED, ARC_MAX_WHEEL_SPEED);
    left = SlewToward(s_last_left_command, left, ARC_COMMAND_SLEW);
    right = SlewToward(s_last_right_command, right, ARC_COMMAND_SLEW);
    s_last_left_command = left;
    s_last_right_command = right;
    Set_PID_Motor(left, right, 0.0f);
}
