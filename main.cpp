/*
 *
 * FEH ROBOT SP24 TEAM C2
 * Team Members:
 * - Griffin Bohm   (bohm.26@buckeyemail.osu.edu)
 * - Brian Jia      (jia.659@buckeyemail.osu.edu)
 * - Sam Patterson  (patterson.1368@buckeyemail.osu.edu)
 * - Drew Straub    (straub.135@buckeyemail.osu.edu)
 *
 * Code by Brian Jia (jia.659@buckeyemail.osu.edu)
 *
 * Unless otherwise specified, all lengths in this code are in inches,
 * and all angles are in radians.
 *
 * IF YOU ARE NOT A MEMBER OF FEH ROBOT SP24 TEAM C2 (THE TEAM), DO NOT USE THIS CODE.
 * THE TEAM HOLDS EXCLUSIVE COPYRIGHT OVER THIS CODE AND USING IT FOR YOUR OWN
 * ROBOT CONSTITUTES COPYRIGHT INFRINGEMENT AND ACADEMIC MISCONDUCT.
 *
 */

#include "CMSIS/MK60D10.h"
#include <FastLCD.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <FEHRCS.h>
#include <cmath>
#include <typeinfo>
#include <FEHBattery.h>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
}

using namespace std;

/*
 * Namespace forward declarations and uses.
 */
namespace util {};
namespace robot_control {};
namespace tasks {};

using namespace util;
using namespace robot_control;
using namespace tasks;

/*
 * Proteus constants.
 */
int SystemCoreClock = 110 * 1000000; // yeah we overclocked the proteus
constexpr int LCD_WIDTH = 320;
constexpr int LCD_HEIGHT = 240;

#define Sleep vTaskDelay

/*
 * Code that won't be modified often during testing is stuffed into the "util" namespace.
 */
namespace util {
    constexpr float rad(float deg) {
        return (float) (deg * (M_PI / 180));
    }

    constexpr float deg(float rad) {
        return (float) (rad * (180 / M_PI));
    }

    [[noreturn]]
    void poweroff() {
        GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(13));
        while (true) {}
    }

    template<int m>
    struct Vec {
        float vec[m];

        Vec add(Vec &b) const {
            Vec<m> sum{};

            for (int i = 0; i < m; i++) {
                sum.vec[i] = vec[i] + b.vec[i];
            }

            return sum;
        }

        float dist(Vec<m> &b) const {
            static_assert(m >= 2);
            return sqrt(pow(b.vec[0] - vec[0], 2) + pow(b.vec[1] - vec[1], 2));
        }
    };

    struct CycTimer {
        uint32_t value{};
        uint32_t last_lap_cyc{};

        void begin() {
            // Enable debugging functions including cycle counter
            // 24th bit is TRCENA
            DEMCR |= (0b1 << 24);
            // Enable the cycle counter itself
            // first bit is CYCCNTENA
            DWT_CTRL |= 0b1;

            value = DWT_CYCCNT;
        }

        uint32_t lap() {
            uint32_t current_cyc = DWT_CYCCNT;
            uint32_t lap_cyc = current_cyc - value;
            value = current_cyc;
            last_lap_cyc = lap_cyc;
            return lap_cyc;
        }
    };

    enum {
        Clear = 0,
        White,
        Gray,
        Red,
        Yellow,
    };

    struct PIController {
        float sample_time;
        float error{};
        float kP, kI;
        float I{}, max_I;
        float control_effort{};

        explicit PIController(float sample_rate, float kP, float kI, float max_I) :
                sample_time(1 / sample_rate), kP(kP), kI(kI), max_I(max_I) {}

        float process(float setpoint, float process_variable) {
            error = setpoint - process_variable;

            I += error * sample_time;
            I = fmaxf(-max_I, fminf(max_I, I));

            control_effort =
                    kP * error +
                    kI * I;

            return control_effort;
        }

        void reset() {
            I = 0;
        }
    };
}

/*
 * Robot control constants.
 */
constexpr int CONTROL_LOOP_HZ = 100;
constexpr float IGWAN_COUNTS_PER_REV = 636;
constexpr float TRACK_WIDTH = 8.40;
constexpr float WHEEL_DIA = 2.5;
constexpr float INCHES_PER_REV = WHEEL_DIA * M_PI;
constexpr float INCHES_PER_COUNT = (float) (INCHES_PER_REV / IGWAN_COUNTS_PER_REV);
float DRIVE_MOTOR_MAX_VOLTAGE = 11; // TODO: igwans are actually rated at 12

/*
 * Robot control configuration.
 */
constexpr float DRIVE_MIN_PERCENT = 7.5;
constexpr float TURN_MIN_PERCENT = 7.5;
constexpr float STOPPED_I = 10;
constexpr float STOPPED_I_ACCUMULATE = 3;
constexpr float STOPPED_I_HIGHPASS = 0.999;
constexpr float START_LIGHT_THRESHOLD_VOLTAGE = 1;
constexpr int WAIT_FOR_LIGHT_CONFIDENT_MS = 50;
constexpr int TICKET_LIGHT_AVERAGING_MS = 50;
constexpr int SWITCH_CONFIDENT_TICKS = 20;

constexpr Vec<2> INITIAL_POS{0, 0};
constexpr float INITIAL_ANGLE = rad(-45);

/*
 * Diagnostics/visualization configuration.
 */
constexpr int DIAGNOSTICS_HZ = 10;
constexpr float FORCE_START_HOLD_SEC = 0.5;
constexpr float FORCE_START_TOTAL_SEC = 1;

/*
 * Port configuration.
 */
constexpr auto DRIVE_MOTOR_L_PORT = FEHMotor::Motor0;
constexpr auto DRIVE_MOTOR_R_PORT = FEHMotor::Motor1;
constexpr auto CDS_CELL_RED_PIN = FEHIO::FEHIOPin::P1_0;
constexpr auto CDS_CELL_BLUE_PIN = FEHIO::FEHIOPin::P1_1;
constexpr auto L_BUMP_SWITCH_PIN = FEHIO::FEHIOPin::P1_5;
constexpr auto R_BUMP_SWITCH_PIN = FEHIO::FEHIOPin::P1_6;
constexpr auto BUTTON_BUMP_SWITCH_PIN = FEHIO::FEHIOPin::P1_7;
constexpr auto ENCODER_L_PIN_0 = FEHIO::FEHIOPin::P3_0;
constexpr auto ENCODER_L_PIN_1 = FEHIO::FEHIOPin::P3_1;
constexpr auto ENCODER_R_PIN_0 = FEHIO::FEHIOPin::P3_2;
constexpr auto ENCODER_R_PIN_1 = FEHIO::FEHIOPin::P3_3;
constexpr float DRIVE_INCHES_PER_COUNT_L = INCHES_PER_COUNT;
constexpr float DRIVE_INCHES_PER_COUNT_R = INCHES_PER_COUNT;

constexpr auto FUEL_SERVO_PORT = FEHServo::FEHServoPort::Servo0;
constexpr auto DUMPTRUCK_SERVO_PORT = FEHServo::FEHServoPort::Servo1;
constexpr auto PASSPORT_SERVO_PORT = FEHServo::FEHServoPort::Servo2;

constexpr auto FUEL_SERVO_MIN = 650;
constexpr auto FUEL_SERVO_MAX = 2400;
constexpr auto DUMPTRUCK_SERVO_MIN = 500;
constexpr auto DUMPTRUCK_SERVO_MAX = 2500;
constexpr auto PASSPORT_SERVO_MIN = 500;
constexpr auto PASSPORT_SERVO_MAX = 2500;

/*
 * Main robot control code.
 */
namespace robot_control {
    enum TicketLightColor : int {
        TICKET_LIGHT_NONE,
        TICKET_LIGHT_RED,
        TICKET_LIGHT_BLUE
    };

    struct Robot {
        /*
          * Because my modified DigitalEncoder code obtains a pointer to itself using the “this” keyword so that the port
          * IRQs can update the object state, we MUST avoid using a copy of the object and use the original object
          * that we instantiated. I couldn't get this working properly with C++ move semantics, so I'm instantiating the
          * DigitalEncoder objects inside the Drivetrain constructor instead. It's working now.
         */

        FEHMotor ml{DRIVE_MOTOR_L_PORT, DRIVE_MOTOR_MAX_VOLTAGE};
        FEHMotor mr{DRIVE_MOTOR_R_PORT, DRIVE_MOTOR_MAX_VOLTAGE};
        DigitalEncoder el{ENCODER_L_PIN_0, ENCODER_L_PIN_1};
        DigitalEncoder er{ENCODER_R_PIN_0, ENCODER_R_PIN_1};
        DigitalInputPin button_bump_switch{BUTTON_BUMP_SWITCH_PIN};
        DigitalInputPin l_bump_switch{L_BUMP_SWITCH_PIN};
        DigitalInputPin r_bump_switch{R_BUMP_SWITCH_PIN};
        AnalogInputPin cds_red{CDS_CELL_RED_PIN};
        AnalogInputPin cds_blue{CDS_CELL_BLUE_PIN};
        FEHServo fuel_servo{FUEL_SERVO_PORT};
        FEHServo dumptruck_servo{DUMPTRUCK_SERVO_PORT};
        FEHServo passport_servo{PASSPORT_SERVO_PORT};

        // Start Light / Ticket Light variables
        TicketLightColor ticket_light_color = TICKET_LIGHT_NONE;

        volatile bool force_start{};

        float target_speed = 50;
        float drive_slew_rate = 200; // Percent per m/s
        float turn_slew_rate = 400; // Percent per radian/s

        /* START DEBUG VARIABLES */
        volatile float cds_red_value{};
        volatile float cds_blue_value{};

        float d_battery_voltage{};
        float d_pct_l{};
        float d_pct_r{};
        float d_diff{};
        int32_t total_counts_l{}, total_counts_r{};

        float R{};
        /* END DEBUG VARIABLES */

        // Position is in inches
        Vec<2> pos{};

        // Angle in radians
        float angle{};
        float target_angle{};

        int32_t counts_l{}, counts_r{};
        int32_t task_counts_l{}, task_counts_r{};

        Robot() {
            pos = INITIAL_POS;
            angle = INITIAL_ANGLE;
            target_angle = INITIAL_ANGLE;

            fuel_servo.SetMax(FUEL_SERVO_MAX);
            fuel_servo.SetMin(FUEL_SERVO_MIN);
            dumptruck_servo.SetMax(DUMPTRUCK_SERVO_MAX);
            dumptruck_servo.SetMin(DUMPTRUCK_SERVO_MIN);
            passport_servo.SetMax(PASSPORT_SERVO_MAX);
            passport_servo.SetMin(PASSPORT_SERVO_MIN);
        }

        void process_odometry() {
            counts_l = el.Counts();
            counts_r = -er.Counts();
            total_counts_l += counts_l;
            total_counts_r += counts_r;
            task_counts_l += counts_l;
            task_counts_r += counts_r;

            el.ResetCounts();
            er.ResetCounts();

            float arclength_l = DRIVE_INCHES_PER_COUNT_L * (float) counts_l;
            float arclength_r = DRIVE_INCHES_PER_COUNT_R * (float) counts_r;

            float arclength_inner;
            if (abs(arclength_l) > abs(arclength_r)) {
                arclength_inner = arclength_r;
            } else {
                arclength_inner = arclength_l;
            }

            float dAngle = (arclength_l - arclength_r) / TRACK_WIDTH;
            if (dAngle != 0) {
                float radius_inner = fabsf(arclength_inner / dAngle);

                // Let R be the distance from the arc center to the point between the wheels
                R = radius_inner + TRACK_WIDTH / 2;

                float dx = R * (cos(angle + dAngle) - cos(angle));
                float dy = -R * (sin(angle + dAngle) - sin(angle));

                if (abs(arclength_l) > abs(arclength_r)) {
                    dx *= -1;
                    dy *= -1;
                }

                pos.vec[0] += dx;
                pos.vec[1] += dy;
                angle += dAngle;
            } else {
                pos.vec[0] += arclength_l * sin(angle);
                pos.vec[1] += arclength_l * cos(angle);
            }
        }

        void motor_power(float pct_l, float pct_r) {
            // A lower battery voltage will result in a HIGHER power supplied to compensate the voltage drop.
            d_battery_voltage = Battery.Voltage();
            float voltage_compensation = 11.7f / d_battery_voltage;

            pct_l *= voltage_compensation;
            pct_r *= voltage_compensation;

            pct_l *= 9.0f / DRIVE_MOTOR_MAX_VOLTAGE;
            pct_r *= 9.0f / DRIVE_MOTOR_MAX_VOLTAGE;

            d_pct_l = pct_l;
            d_pct_r = pct_r;

            ml.SetPercent(pct_l);
            mr.SetPercent(pct_r);
        }

        static float slew(float rate, float min, float max, float dist_from_start, float dist_to_end) {
            float slewed_start = sqrtf(rate * fabs(dist_from_start));
            float slewed_end = sqrtf(rate * fabs(dist_to_end));
            return fmin(max, fmin(slewed_start, slewed_end) + min);
        }

        struct unstuck {
            uint32_t last_encoder_l_tick_at = 0;
            uint32_t last_encoder_r_tick_at = 0;
            uint32_t tick_count = 0;
            float stopped_i = 0;

            float process(Robot &r, float turn_l_mul, float turn_r_mul) {
                if (r.counts_l != 0) {
                    last_encoder_l_tick_at = tick_count;
                }
                if (r.counts_r != 0) {
                    last_encoder_r_tick_at = tick_count;
                }

                if (turn_l_mul > 0.2) {
                    if (last_encoder_l_tick_at + 25 < tick_count) {
                        stopped_i += STOPPED_I_ACCUMULATE / CONTROL_LOOP_HZ;
                    }
                }
                if (turn_r_mul > 0.2) {
                    if (last_encoder_r_tick_at + 25 < tick_count) {
                        stopped_i += STOPPED_I_ACCUMULATE / CONTROL_LOOP_HZ;
                    }
                }

                tick_count++;
                stopped_i *= STOPPED_I_HIGHPASS;
                return stopped_i * STOPPED_I;
            }

        };

        void execute_straight(float inches, bool until_switch, int timeout_ms) {
            int switch_pressed_ticks = 0;

            Vec<2> pos0 = pos;
            unstuck unstuck;
            PIController angle_controller(CONTROL_LOOP_HZ, 200, 100, 100);
            TickType_t time = xTaskGetTickCount();
            TickType_t start_time = xTaskGetTickCount();

            while (true) {
                if (timeout_ms > 0) {
                    if (xTaskGetTickCount() - start_time > timeout_ms) {
                        motor_power(0, 0);
                        return;
                    }
                }

                if (until_switch) {
                    if (!l_bump_switch.Value() && !r_bump_switch.Value()) {
                        switch_pressed_ticks++;
                    } else if (!button_bump_switch.Value()) {
                        switch_pressed_ticks++;
                    } else {
                        switch_pressed_ticks = 0;
                    }

                    if (switch_pressed_ticks >= SWITCH_CONFIDENT_TICKS) {
                        motor_power(0, 0);
                        return;
                    }
                }

                float control_effort = angle_controller.process(target_angle, angle);

                float dist = pos.dist(pos0);
                float dist_remain = fabs(inches) - fabs(dist);
                float power_pct = slew(
                        drive_slew_rate,
                        DRIVE_MIN_PERCENT,
                        target_speed,
                        dist,
                        dist_remain
                );

                power_pct += unstuck.process(*this, 1, 1);

                if (inches < 0) {
                    motor_power(-power_pct + control_effort, -power_pct - control_effort);
                } else {
                    motor_power(power_pct + control_effort, power_pct - control_effort);
                }

                if (fabs(dist) > fabsf(inches)) {
                    motor_power(0, 0);
                    return;
                }

                vTaskDelayUntil(&time, 1000 / CONTROL_LOOP_HZ);
            }
        }

        void execute_turn(float degree, float turn_l_mul, float turn_r_mul) {
            target_angle = (float) rad(degree);
            float turn_start_angle = angle;
            bool turning_right = target_angle > angle;

            task_counts_l = 0;
            task_counts_r = 0;
            unstuck unstuck;
            PIController pi(CONTROL_LOOP_HZ, 0.5, 1, 30);
            TickType_t time = xTaskGetTickCount();

            while (true) {
                float angle_turned_so_far = fabs(angle - turn_start_angle);
                float angle_remain = fabs(target_angle - angle);
                float power_pct = slew(
                        turn_slew_rate,
                        TURN_MIN_PERCENT,
                        target_speed,
                        angle_turned_so_far,
                        angle_remain
                );

                power_pct += unstuck.process(*this, turn_l_mul, turn_r_mul);

                // make sure wheel ratios are correct using PI controller
                float l_effort, r_effort;
                // if regular (i.e. not pivot) turn
                if (turn_l_mul != 0 && turn_r_mul != 0) {
                    // diff > 0 - left wheel is ahead of right
                    // diff < 0 - right wheel is ahead of left
                    float diff = abs((float)task_counts_l / turn_l_mul) - abs((float)task_counts_r / turn_r_mul);
                    float effort = pi.process(0, diff);

                    // when left is ahead, diff > 0, and so PI produces a negative effort
                    l_effort = effort;
                    r_effort = -effort;
                } else {
                    if (turn_l_mul == 0) {
                        l_effort = pi.process(0, (float)task_counts_l);
                        r_effort = 0;
                    } else {
                        l_effort = 0;
                        r_effort = pi.process(0, (float)task_counts_r);
                    }
                }

                l_effort = 0;
                r_effort = 0;

                float new_pct_l, new_pct_r;
                if (turning_right) {
                    new_pct_l = power_pct + l_effort;
                    new_pct_r = -power_pct + r_effort;

                    if (angle >= target_angle) {
                        motor_power(0, 0);
                        return;
                    }
                } else {
                    new_pct_l = -power_pct + l_effort;
                    new_pct_r = power_pct + r_effort;

                    if (angle <= target_angle) {
                        motor_power(0, 0);
                        return;
                    }
                }

                new_pct_l *= turn_l_mul;
                new_pct_r *= turn_r_mul;

                motor_power(new_pct_l, new_pct_r);

                vTaskDelayUntil(&time, 1000 / CONTROL_LOOP_HZ);
            }
        }
    } robot;
}

/*
 * Task running/handling code.
 */
namespace tasks {
    void WaitForStartLight() {
        robot.ml.Stop();
        robot.mr.Stop();

        // wait some time until we're confident that the light has started
        int ms_been_confident_for = 0;
        while (ms_been_confident_for < WAIT_FOR_LIGHT_CONFIDENT_MS) {
            robot.cds_red_value = robot.cds_red.Value();
            robot.cds_blue_value = robot.cds_blue.Value();

            // robot control loop will update cds_red_value
            if (robot.cds_red_value < START_LIGHT_THRESHOLD_VOLTAGE) {
                ms_been_confident_for++;
            } else {
                ms_been_confident_for = 0;
            }

            if (robot.force_start) {
                break;
            }

            Sleep(1);
        }
    }

    void CaptureTicketLight() {
        robot.ml.Stop();
        robot.mr.Stop();

        float cds_red_value_avg = 0;
        float cds_blue_value_avg = 0;

        for (int i = 0; i < TICKET_LIGHT_AVERAGING_MS; i++) {
            robot.cds_red_value = robot.cds_red.Value();
            robot.cds_blue_value = robot.cds_blue.Value();

            cds_red_value_avg += robot.cds_red_value;
            cds_blue_value_avg += robot.cds_blue_value;

            Sleep(1);
        }

        cds_red_value_avg /= TICKET_LIGHT_AVERAGING_MS;
        cds_blue_value_avg /= TICKET_LIGHT_AVERAGING_MS;

        cds_red_value_avg *= 1.15; // Correction factor to make red and blue equal.

        if (cds_red_value_avg < cds_blue_value_avg) {
            // CDS cell red receiving more light...
            robot.ticket_light_color = TICKET_LIGHT_RED;
        } else {
            robot.ticket_light_color = TICKET_LIGHT_BLUE;
        }
    }

    void Straight(float inches) {
        robot.execute_straight(inches, false, 0);
    }

    void StraightTimeout(float inches, int timeout_ms) {
        robot.execute_straight(inches, false, timeout_ms);
    }

    void StraightUntilSwitch(float inches) {
        robot.execute_straight(inches, true, 0);
    }

    void StraightUntilSwitchTimeout(float inches, int timeout_ms) {
        robot.execute_straight(inches, true, timeout_ms);
    }

    void ResetFacing(float degree) {
        robot.angle = rad(degree);
    }

    void Speed(float percent) {
        robot.target_speed = percent;
    }

    void TurnSlewRate(float rate) {
        robot.turn_slew_rate = rate;
    }

    void DriveSlewRate(float rate) {
        robot.drive_slew_rate = rate;
    }

    void Turn(float degree) {
        robot.execute_turn(degree, 1, 1);
    }

    void Pivot(float degree, float turn_wheel_bias) {
        float turn_l_mul = 1, turn_r_mul = 1;
        if (turn_wheel_bias < 0) {
            turn_r_mul = 1 - fabs(turn_wheel_bias);
            turn_l_mul = fabs(turn_wheel_bias) + 1;
        } else if (turn_wheel_bias > 0) {
            turn_l_mul = 1 - fabs(turn_wheel_bias);
            turn_r_mul = fabs(turn_wheel_bias) + 1;
        }

        robot.execute_turn(degree, turn_l_mul, turn_r_mul);
    }

    void Arc(float degree, float turn_l_mul, float turn_r_mul) {
        robot.execute_turn(degree, turn_l_mul, turn_r_mul);
    }

    void PivotLeft(float degree) {
        // right wheel turning only = pivoting on left
        robot.execute_turn(degree, 0, 2);
    }

    void PivotRight(float degree) {
        // left wheel turning only = pivoting on right
        robot.execute_turn(degree, 2, 0);
    }

    void FuelServo(float degree) {
        robot.fuel_servo.SetDegree(degree);
    }

    void DumptruckServo(float degree) {
        robot.dumptruck_servo.SetDegree(degree);
    }

    void PassportServo(float degree) {
        robot.passport_servo.SetDegree(degree);
    }
}

/*
 * Visualization/debugging code.
 */
namespace visualization {
    template<typename T>
    void log(const char *label, T value) {
        FastLCD::Write(label);
        FastLCD::Write(": ");
        FastLCD::WriteLine(value);
    }

    void print_diagnostics() {
        static bool prev_touching = false;
        static bool display_testing_screen = true;
        static float holding_sec = 0;

        int x, y;
        bool touching = LCD.Touch(&x, &y);
        if (touching && !prev_touching) {
            if (x < LCD_WIDTH / 2) {
                display_testing_screen = !display_testing_screen;
            }
        }
        prev_touching = touching;

        switch (robot.ticket_light_color) {
            case TICKET_LIGHT_RED:
                FastLCD::SetPaletteColor(Clear, RED);
                break;
            case TICKET_LIGHT_BLUE:
                FastLCD::SetPaletteColor(Clear, BLUE);
                break;
            case TICKET_LIGHT_NONE:
                FastLCD::SetPaletteColor(Clear, BLACK);
                break;
        }

        FastLCD::Clear();

        FastLCD::SetFontPaletteIndex(White);

        log("Time (s)", (int)(xTaskGetTickCount() / 1000));

        log("CDS  Red", robot.cds_red_value);
        log("CDS Blue", robot.cds_blue_value);

        log("FuelLever", RCS.GetCorrectLever());
        log("Angle", deg(robot.angle));
        log("TargetAngle", deg(robot.target_angle));

        log("Battery%", (int)round((robot.d_battery_voltage / 11.7) * 100));
        log("MotorL%", robot.d_pct_l);
        log("MotorR%", robot.d_pct_r);
        log("Diff", robot.d_diff);

        log("LBump", robot.r_bump_switch.Value());
        log("RBump", robot.r_bump_switch.Value());
        log("FBump", robot.button_bump_switch.Value());

        if (holding_sec < FORCE_START_HOLD_SEC) {
            if (touching && x >= LCD_WIDTH / 2) {
                holding_sec += 1.0 / DIAGNOSTICS_HZ;
            } else {
                holding_sec = 0;
            }
        } else {
            holding_sec += 1.0 / DIAGNOSTICS_HZ;

            if (holding_sec > FORCE_START_TOTAL_SEC) {
                robot.force_start = true;
                holding_sec = 0;
            }
        }

        if (holding_sec > 0) {
            float progress = holding_sec / FORCE_START_TOTAL_SEC;
            auto progress_bar_width = (int) (progress * LCD_WIDTH);
            FastLCD::SetFontPaletteIndex(3);
            FastLCD::FillRectangle(0, 0, progress_bar_width, 32);
        }

        FastLCD::DrawScreen();
    }
}

void robot_path_task() {
    const int TS = 800;
    const int DS = 600;

    FuelServo(180);
    DumptruckServo(180);
    PassportServo(90);

    TurnSlewRate(TS);
    Speed(90);

    /*
     * PATH START!
     */

    // Wait for start light to turn on.
    WaitForStartLight();

    // Start button
    DriveSlewRate(1000); // fast acceleration for start button
        StraightTimeout(-2, 500);
    DriveSlewRate(550);    // a bit slower for getting to levers

    // Forward // TODO: 18.5 cuts it close to the lever assembly
    Straight(19.5);
    DriveSlewRate(DS);     // regular acceleration for rest of course

    // Turn left toward fuel levers
    Turn(-90);

    // Levers are 3.5 inches apart
    float leverMinus = float(2 - RCS.GetCorrectLever()) * 3.5f;

    // Go toward fuel levers
    Straight(8 - leverMinus); // This will vary based on which fuel lever to use.

    // Turn arm toward fuel levers
    Turn(0);
    Straight(-3.5); // Back into position for the fuel lever flipping

    // Fuel lever flip sequence
    FuelServo(72);
    Sleep(250);
    FuelServo(135);
    Straight(3);
    Sleep(5000);
    FuelServo(45);
    Straight(-3);
    FuelServo(90);
    Sleep(250);
    FuelServo(45);
    Straight(3);
    FuelServo(180);

    // Turn right
    Turn(90);

    // Square with the left wall
    StraightUntilSwitch(-(8.0f + leverMinus) - 2);
    ResetFacing(90);

    // Face up ramp
    Straight(0.5);
    PivotLeft(0);

    // Drive up ramp
    Straight(25);

    // Go to luggage drop
    Pivot(180, -0.8);
    StraightUntilSwitch(9);
    ResetFacing(180);

    // Drop the luggage
    DumptruckServo(90);
    Sleep(500);
    DumptruckServo(180);

    // Go to ticket light
    Straight(-15);
    Turn(135);

    // Square with ticket light wall
    StraightUntilSwitch(-15);
    ResetFacing(135);
    Straight(2);
    CaptureTicketLight();

    if (robot.ticket_light_color == TICKET_LIGHT_BLUE) {
        Straight(6);
        Turn(90);
        Straight(8.75);
        PivotRight(0);
        StraightUntilSwitchTimeout(7, 2000);

        // Pivot to get into position for the center button
        PivotRight(-45);
        PivotLeft(0);
    } else {
        Straight(10);
        Pivot(0, 0.75);
        StraightUntilSwitchTimeout(4, 2000);

        // Pivot to get into position for the center button
        PivotLeft(45);
        PivotRight(0);
    }

    StraightUntilSwitchTimeout(8, 2000);
    Straight(-1.1);

    // Dumptruck in place
    DumptruckServo(30);
    Sleep(1000);
    DumptruckServo(75);

    // Get into place for passport
    PassportServo(170);
    PivotRight(-50);
    DumptruckServo(0);
    PivotLeft(0);
    PivotRight(25);

    // Passport arm up
    PassportServo(70);
    Sleep(1000);
    PassportServo(90);

    // use the dumptruck to hit the passport down
    PivotRight(30);
    PivotLeft(180);

    // Go toward ramp to go to end button
    Straight(7);
    DumptruckServo(180);
    PivotLeft(90);

    // Square with right side wall and turn
    StraightUntilSwitch(15);
    ResetFacing(90);
    Straight(-1);

    Turn(180);

    // Go down right side ramp and hit the end button
    DriveSlewRate(600); // FULL SPEED AHEAD!
    Speed(100);
    Straight(50);
}

[[noreturn]]
void odometry_task() {
    static TickType_t time = 0;
    while (true) {
        robot.process_odometry();
        vTaskDelayUntil(&time, 1000 / CONTROL_LOOP_HZ);
    }
}

[[noreturn]]
void diagnostics_task() {
    static TickType_t time = 0;
    while (true) {
        visualization::print_diagnostics();
        vTaskDelayUntil(&time, 1000 / DIAGNOSTICS_HZ);
    }
}

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                              StackType_t **ppxIdleTaskStackBuffer,
                                              uint32_t *puxIdleTaskStackSize) {

    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

extern "C" void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                               StackType_t **ppxTimerTaskStackBuffer,
                                               uint32_t *puxTimerTaskStackSize) {

    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *puxTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

int main() {

    /*
     * Initialize Robot Communication System (RCS).
     */

    RCS.InitializeTouchMenu("C2N8hFpMW");

    /*
     * Assign colors to palette numbers.
     */
    FastLCD::SetPaletteColor(Clear, BLACK);
    FastLCD::SetPaletteColor(White, WHITE);
    FastLCD::SetPaletteColor(Gray, GRAY);
    FastLCD::SetPaletteColor(Red, RED);
    FastLCD::SetPaletteColor(Yellow, YELLOW);

    /*
     * Start tasks.
     */
    const int STACK_SIZE = 1024;
    static StaticTask_t diagnostics_tcb;
    static StackType_t diagnostics_stack[STACK_SIZE];
    static StaticTask_t robot_path_tcb;
    static StackType_t robot_path_stack[STACK_SIZE];
    static StaticTask_t odometry_tcb;
    static StackType_t odometry_stack[STACK_SIZE];

    xTaskCreateStatic(
            TaskFunction_t(diagnostics_task),
            "diagnostics",
            STACK_SIZE,
            nullptr,
            0,
            diagnostics_stack,
            &diagnostics_tcb
    );

    xTaskCreateStatic(
            TaskFunction_t(robot_path_task),
            "robot_path",
            STACK_SIZE,
            nullptr,
            4,
            robot_path_stack,
            &robot_path_tcb
    );

    xTaskCreateStatic(
            TaskFunction_t(odometry_task),
            "odometry",
            STACK_SIZE,
            nullptr,
            5,
            odometry_stack,
            &odometry_tcb
    );

    vTaskStartScheduler();
}