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
 * Utility type definitions.
 */
using Radian = float;
using Volt = float;
using Degree = float;
using Microsecond = float;
using Second = float;
using Inch = float;
using Hertz = int;

/*
 * Proteus constants.
 */
constexpr int BSP_BUS_DIV = 2; // Declared in startup_mk60d10.cpp
constexpr Hertz TICK_RATE = 100;
constexpr Hertz PROTEUS_SYSTEM_HZ = 88000000.0;
constexpr int LCD_WIDTH = 320;
constexpr int LCD_HEIGHT = 240;

/*
 * Code that won't be modified often during testing is stuffed into the "util" namespace.
 */
namespace util {
    constexpr Radian rad(Degree deg) {
        return (float) (deg * (M_PI / 180));
    }

    constexpr Degree deg(Radian rad) {
        return (float) (rad * (180 / M_PI));
    }

    constexpr uint32_t cyc(const double sec) {
        return (uint32_t) (sec * PROTEUS_SYSTEM_HZ);
    }

    constexpr uint32_t ticks(const double sec) {
        return (uint32_t) (sec * TICK_RATE);
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

    template<int m, int n>
    struct Mat {
        static_assert(m == n);
        float mat[m * n];

        Vec<m> multiply(Vec<m> &b) const {
            Vec<m> result{};

            int mat_pos = 0;
            for (int r = 0; r < m; r++) {
                for (int c = 0; c < n; c++) {
                    result.vec[r] += mat[mat_pos++] * b.vec[c];
                }
            }

            return result;
        }

        static Mat<3, 3> RotationTranslation(float angle, float dx, float dy) {
            float cos = cosf(angle);
            float sin = sinf(angle);

            return Mat<3, 3>{
                    cos, -sin, dx,
                    sin, cos, dy,
                    0, 0, 1
            };
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

    // NXP Freescale K60 manual: Chapter 40: Periodic Interrupt Timer (PIT)
    template<int pit_num, int irq_priority>
    void init_PIT(uint32_t cyc_interval) {
        // PIT0 is being used by FEHIO AnalogEncoder and AnalogInputPin, cannot use
        static_assert(pit_num != 0);
        static_assert(pit_num < 4);

        // Set priority
        NVIC_SetPriority((IRQn) (PIT0_IRQn + pit_num), irq_priority);

        // Enable PIT clock
        PIT_BASE_PTR->MCR = 0;
        // Set up PIT interval
        PIT_BASE_PTR->CHANNEL[pit_num].LDVAL = cyc_interval / BSP_BUS_DIV;
        // Start PIT and enable PIT interrupts
        PIT_BASE_PTR->CHANNEL[pit_num].TCTRL = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

        // Enable interrupt in NVIC
        NVIC_EnableIRQ((IRQn) (PIT0_IRQn + pit_num));
    }

    template<int pit_num>
    void clear_PIT_irq_flag() {
        PIT_BASE_PTR->CHANNEL[pit_num].TFLG = 1;
    }

    enum {
        Clear = 0,
        White,
        Gray,
        Red,
        Yellow,
    } PaletteColors;

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
constexpr int SYSTICK_INTERVAL_CYCLES = cyc(1.0 / TICK_RATE);
constexpr float IGWAN_COUNTS_PER_REV = 636;
constexpr Inch TRACK_WIDTH = 8.276;
constexpr Inch WHEEL_DIA = 2.5;
constexpr Inch INCHES_PER_REV = WHEEL_DIA * M_PI;
constexpr Inch INCHES_PER_COUNT = (float) (INCHES_PER_REV / IGWAN_COUNTS_PER_REV);
constexpr Volt DRIVE_MOTOR_MAX_VOLTAGE = 9.0;

/*
 * Robot control configuration.
 */
constexpr float DRIVE_SLEW_RATE = 200; // Percent per m/s
constexpr float TURN_SLEW_RATE = 200; // Percent per radian/s
constexpr float DRIVE_MIN_PERCENT = 5;
constexpr float TURN_MIN_PERCENT = 5;
constexpr float STOPPED_I = 10;
constexpr float STOPPED_I_ACCUMULATE = 3;
constexpr float STOPPED_I_HIGHPASS = 0.999;
constexpr float START_LIGHT_THRESHOLD_VOLTAGE = 0.5;
constexpr float TICKET_LIGHT_THRESHOLD_VOLTAGE = 2.2;
constexpr float RED_LIGHT_VOLTAGE = 0.35;
constexpr float BLUE_LIGHT_VOLTAGE = 1.5;
constexpr int WAIT_FOR_LIGHT_CONFIRM_TICKS = ticks(0.5);

constexpr Vec<2> INITIAL_POS{0, 0};
constexpr Radian INITIAL_ANGLE = rad(-45);

/*
 * Diagnostics/visualization configuration.
 */
constexpr Hertz DIAGNOSTICS_HZ = 5;
constexpr Microsecond TICK_INTERVAL_MICROSECONDS = (1.0 / TICK_RATE) * 1000000;
constexpr Second FORCE_START_HOLD_SEC = 0.5;
constexpr Second FORCE_START_TOTAL_SEC = 1;

/*
 * Port configuration.
 */
constexpr auto DRIVE_MOTOR_L_PORT = FEHMotor::Motor0;
constexpr auto DRIVE_MOTOR_R_PORT = FEHMotor::Motor1;
constexpr auto ENCODER_L_PIN_0 = FEHIO::FEHIOPin::P3_0;
constexpr auto ENCODER_L_PIN_1 = FEHIO::FEHIOPin::P3_1;
constexpr auto ENCODER_R_PIN_0 = FEHIO::FEHIOPin::P0_0;
constexpr auto ENCODER_R_PIN_1 = FEHIO::FEHIOPin::P0_1;
constexpr auto LIGHT_SENSOR_PIN = FEHIO::FEHIOPin::P2_0;
constexpr auto BUMP_SWITCH_PIN = FEHIO::FEHIOPin::P1_7;
constexpr Inch DRIVE_INCHES_PER_COUNT_L = INCHES_PER_COUNT;
constexpr Inch DRIVE_INCHES_PER_COUNT_R = INCHES_PER_COUNT;

constexpr auto LEVER_SERVO_PORT = FEHServo::FEHServoPort::Servo0;
constexpr auto LEVER_SERVO_MIN = 650;
constexpr auto LEVER_SERVO_MAX = 2300;

/*
 * Main robot control code.
 */
namespace robot_control {
    enum TicketLightColor : int {
        TICKET_LIGHT_NONE,
        TICKET_LIGHT_RED,
        TICKET_LIGHT_BLUE
    };

    enum class ControlMode {
        INIT,
        TURNING,
        STRAIGHT,
        STRAIGHT_UNTIL_SWITCH,
        WAIT_FOR_START_LIGHT,
        WAIT_FOR_TICKET_LIGHT
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
        DigitalInputPin bump_switch{BUMP_SWITCH_PIN};
        AnalogInputPin light_sensor{LIGHT_SENSOR_PIN};
        FEHServo lever_servo{LEVER_SERVO_PORT};

        ControlMode control_mode = ControlMode::INIT;

        // Start Light / Ticket Light variables
        bool force_start{};
        Volt light_sensor_value{};
        int last_nonconfident_wait_for_light_tick{};
        float light_sensor_average_value{};
        TicketLightColor ticket_light_color = TICKET_LIGHT_NONE;

        int tick_count{}, task_tick_count{};

        // Drivetrain variables
        float pct_l{}, pct_r{};
        float target_pct{};
        float slewed_pct{};
        float dist_remain{};
        Inch target_dist{};
        int total_counts_l{}, total_counts_r{};
        PIController angle_controller = PIController(TICK_RATE, 100, 50, 30);

        // Position is in inches
        Vec<2> pos{}, pos0{};
        // Angle in radians
        Radian angle{};
        Radian target_angle{};
        Radian turn_start_angle{};
        bool turning_right{};
        float R{};

        volatile bool task_running{};

        int last_encoder_l_tick_at = 0;
        int last_encoder_r_tick_at = 0;
        float stopped_i{};

        Robot() {
            pos = INITIAL_POS;
            angle = INITIAL_ANGLE;
            target_angle = INITIAL_ANGLE;

            lever_servo.SetMax(LEVER_SERVO_MAX);
            lever_servo.SetMin(LEVER_SERVO_MIN);
        }

        void task_finished() {
            ml.Stop();
            mr.Stop();

            // Set main function priority to higher than encoders so the task list can run.
            __set_BASEPRI(1);
            task_running = false;
        }

        void process_odometry() {
            auto counts_l = el.Counts();
            auto counts_r = -er.Counts();

            total_counts_l += counts_l;
            total_counts_r += counts_r;

            if (counts_l != 0) {
                last_encoder_l_tick_at = tick_count;
            }
            if (counts_r != 0) {
                last_encoder_r_tick_at = tick_count;
            }

            if (last_encoder_l_tick_at + ticks(0.025) < tick_count) {
                stopped_i += STOPPED_I_ACCUMULATE / TICK_RATE;
            }
            if (last_encoder_r_tick_at + ticks(0.025) < tick_count) {
                stopped_i += STOPPED_I_ACCUMULATE / TICK_RATE;
            }
            stopped_i *= STOPPED_I_HIGHPASS;

            el.ResetCounts();
            er.ResetCounts();

            Inch arclength_l = DRIVE_INCHES_PER_COUNT_L * (float) counts_l;
            Inch arclength_r = DRIVE_INCHES_PER_COUNT_R * (float) counts_r;

            Inch arclength_inner;
            if (abs(arclength_l) > abs(arclength_r)) {
                arclength_inner = arclength_r;
            } else {
                arclength_inner = arclength_l;
            }

            Radian dAngle = (arclength_l - arclength_r) / TRACK_WIDTH;
            if (dAngle != 0) {
                Inch radius_inner = fabsf(arclength_inner / dAngle);

                // Let R be the distance from the arc center to the point between the wheels
                R = radius_inner + TRACK_WIDTH / 2;

                Inch dx = R * (cos(angle + dAngle) - cos(angle));
                Inch dy = -R * (sin(angle + dAngle) - sin(angle));

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

        void motor_power(float new_pct_l, float new_pct_r) {
            pct_l = new_pct_l;
            pct_r = new_pct_r;

            // A lower battery voltage will result in a HIGHER power supplied to compensate the voltage drop.
            float voltage_compensation = 11.7f / Battery.Voltage();

            pct_l *= voltage_compensation;
            pct_r *= voltage_compensation;

            ml.SetPercent(pct_l);
            mr.SetPercent(pct_r);
        }

        static float slew(float rate, float min, float max, float dist_from_start, float dist_to_end) {
            float slewed_start = sqrtf(rate * fabs(dist_from_start));
            float slewed_end = sqrtf(rate * fabs(dist_to_end));
            return fmin(max, fmin(slewed_start, slewed_end) + min);
        }

        void tick() {
            tick_count++;
            task_tick_count++;

            /*
             * GET ANALOG INPUT DATA
             */

            light_sensor_value = light_sensor.Value();

            /*
             * ODOMETRY
             */

            process_odometry();

            /*
             * DRIVETRAIN MOTOR MANAGEMENT
             */

            float control_effort;
            switch (control_mode) {
                case ControlMode::WAIT_FOR_TICKET_LIGHT:
                case ControlMode::WAIT_FOR_START_LIGHT:
                    ml.Stop();
                    mr.Stop();

                    float threshold_voltage;
                    if (control_mode == ControlMode::WAIT_FOR_START_LIGHT) {
                        threshold_voltage = START_LIGHT_THRESHOLD_VOLTAGE;
                    } else {
                        threshold_voltage = TICKET_LIGHT_THRESHOLD_VOLTAGE;
                    }

                    light_sensor_average_value += light_sensor_value;
                    if (light_sensor_value >= threshold_voltage) {
                        last_nonconfident_wait_for_light_tick = tick_count;
                        light_sensor_average_value = 0;
                    }

                    if (control_mode == ControlMode::WAIT_FOR_START_LIGHT && force_start) {
                        task_finished();
                        return;
                    }

                    // wait some time until we're confident that the light has started
                    if (last_nonconfident_wait_for_light_tick + WAIT_FOR_LIGHT_CONFIRM_TICKS < tick_count) {
                        light_sensor_average_value /= WAIT_FOR_LIGHT_CONFIRM_TICKS;

                        if (control_mode == ControlMode::WAIT_FOR_TICKET_LIGHT) {
                            float red_dist = abs(RED_LIGHT_VOLTAGE - light_sensor_average_value);
                            float blue_dist = abs(BLUE_LIGHT_VOLTAGE - light_sensor_average_value);

                            if (red_dist < blue_dist) {
                                ticket_light_color = TICKET_LIGHT_RED;
                            } else {
                                ticket_light_color = TICKET_LIGHT_BLUE;
                            }
                        }

                        task_finished();
                    }
                    return;
                case ControlMode::TURNING: {
                    Radian angle_turned_so_far = fabs(angle - turn_start_angle);
                    Radian angle_remain = fabs(target_angle - angle);
                    slewed_pct = slew(
                            TURN_SLEW_RATE,
                            TURN_MIN_PERCENT,
                            target_pct,
                            angle_turned_so_far,
                            angle_remain
                    );
                    float power_pct = slewed_pct + stopped_i * STOPPED_I;

                    if (turning_right) {
                        motor_power(power_pct, -power_pct);
                        if (angle >= target_angle) {
                            task_finished();
                        }
                    } else {
                        motor_power(-power_pct, power_pct);
                        if (angle <= target_angle) {
                            task_finished();
                        }
                    }
                    return;
                }
                case ControlMode::STRAIGHT_UNTIL_SWITCH:
                    if (!bump_switch.Value()) {
                        task_finished();
                        return;
                    }
                case ControlMode::STRAIGHT: {
                    control_effort = angle_controller.process(target_angle, angle);

                    float dist = pos.dist(pos0);
                    dist_remain = fabs(target_dist) - fabs(dist);
                    slewed_pct = slew(
                            DRIVE_SLEW_RATE,
                            DRIVE_MIN_PERCENT,
                            target_pct,
                            dist,
                            dist_remain
                    );
                    float power_pct = slewed_pct + stopped_i * STOPPED_I;

                    if (target_dist < 0) {
                        motor_power(-power_pct + control_effort, -power_pct - control_effort);
                    } else {
                        motor_power(power_pct + control_effort, power_pct - control_effort);
                    }

                    if (fabs(dist) > fabsf(target_dist)) {
                        task_finished();
                    }
                    return;
                }
                default:
                    return;
            }
        }
    } robot;

    void stop_robot_control_loop() {
        robot.motor_power(0, 0);
        SysTick_BASE_PTR->CSR = 0;
    }

    CycTimer tick_timer{};
    int tick_cycles{};
    // This is called every (1 / TICK_RATE) seconds by the SysTick timer
    extern "C" void SysTick_Handler(void) {
        tick_timer.begin();
        robot.tick();
        tick_cycles = (int) tick_timer.lap();
    }

}

/*
 * Task running/handling code.
 */
namespace tasks {
    int spinwait_iters{};

    void wait_for_task_to_finish() {
        spinwait_iters = 0;
        robot.task_running = true;
        // Lower main() priority back to normal.
        __set_BASEPRI(0);
        while (robot.task_running) {
            spinwait_iters++;
        }
    }

    void WaitForStartLight() {
        robot.control_mode = ControlMode::WAIT_FOR_START_LIGHT;
        robot.light_sensor_average_value = 0;
        robot.last_nonconfident_wait_for_light_tick = robot.tick_count;
        wait_for_task_to_finish();
    }

    void WaitForTicketLight(int timeout_ms) {
        robot.control_mode = ControlMode::WAIT_FOR_TICKET_LIGHT;
        robot.light_sensor_average_value = 0;
        robot.last_nonconfident_wait_for_light_tick = robot.tick_count;
        wait_for_task_to_finish();
    }

    void Straight_prepare(Inch inches) {
        robot.pos0 = robot.pos;
        robot.target_dist = inches;
        robot.angle_controller.reset();
        robot.control_mode = ControlMode::STRAIGHT;
        robot.stopped_i = 0;
    }

    void Straight(Inch inches) {
        Straight_prepare(inches);
        wait_for_task_to_finish();
    }

    void StraightUntilSwitch(Inch inches) {
        Straight_prepare(inches);
        robot.control_mode = ControlMode::STRAIGHT_UNTIL_SWITCH;
        wait_for_task_to_finish();
    }

    void Speed(float percent) {
        robot.target_pct = percent;
    }

    void Turn(Degree angle) {
        robot.target_angle = (float) rad(angle);
        robot.turn_start_angle = robot.angle;
        robot.turning_right = robot.target_angle > robot.angle;
        robot.control_mode = ControlMode::TURNING;
        robot.stopped_i = 0;
        wait_for_task_to_finish();
    }

    void LeverServo(float degree) {
        robot.lever_servo.SetDegree(degree);
    }

    void Position4Bar(Degree target_angle) {
        // TODO
        wait_for_task_to_finish();
    }

    void Delay(int ms) {
        // TODO
        wait_for_task_to_finish();
    }
}

/*
 * Visualization/debugging code.
 */
namespace visualization {
    [[nodiscard]]
    const char *control_mode_string() {
        switch (robot.control_mode) {
            case ControlMode::INIT:
                return "Init";
            case ControlMode::STRAIGHT:
                return "Forward";
            case ControlMode::STRAIGHT_UNTIL_SWITCH:
                return "FwdTilSwitch";
            case ControlMode::TURNING:
                return "Turning";
            case ControlMode::WAIT_FOR_START_LIGHT:
                return "WaitStartLight";
            case ControlMode::WAIT_FOR_TICKET_LIGHT:
                return "WaitTicketLight";
        }
        return "?????";
    }

    const char *nesw[4] = {"N", "W", "S", "E"};

    Vec<3> nesw_poss[] = {
            Vec<3>{0, 80, 1},
            Vec<3>{80, 0, 1},
            Vec<3>{0, -80, 1},
            Vec<3>{-80, 0, 1},
    };

    Vec<3> arrow_vtx[] = {
            Vec<3>{-10, 0, 1},
            Vec<3>{10, 0, 1},
            Vec<3>{10, 45, 1},
            Vec<3>{14, 45, 1},
            Vec<3>{0, 64, 1},
            Vec<3>{-14, 45, 1},
            Vec<3>{-10, 45, 1},
    };

    const int arrow_vtx_len = sizeof(arrow_vtx) / sizeof(Vec<3>);

    void draw_vtx_list(Vec<3> vtx_list[], int len, Mat<3, 3> mat, bool thick) {
        Vec<3> vtx1 = mat.multiply(vtx_list[len - 1]);
        for (int i = 0; i < len; i++) {
            Vec<3> vtx2 = mat.multiply(vtx_list[i]);
            FastLCD::DrawLine(
                    (int) vtx1.vec[0],
                    (int) vtx1.vec[1],
                    (int) vtx2.vec[0],
                    (int) vtx2.vec[1], thick);
            vtx1 = vtx2;
        }
    }

    void draw_compass(Mat<3, 3> mat) {
        FastLCD::SetFontPaletteIndex(White);
        for (int i = 0; i < 4; i++) {
            Vec<3> offs{-12.0 / 2, -17.0 / 2, 1};
            Vec<3> pos = mat.multiply(nesw_poss[i]).add(offs);
            FastLCD::WriteAt(nesw[i], (int) pos.vec[0], (int) pos.vec[1]);
        }

        FastLCD::DrawCircle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 64);
        FastLCD::WriteAt(deg(robot.angle), LCD_WIDTH / 2 - 42, 17);

        FastLCD::SetFontPaletteIndex(Yellow);
        draw_vtx_list(arrow_vtx, arrow_vtx_len, mat, true);
    }

    CycTimer visualization_timer;
    CycTimer draw_timer;

    template<typename T>
    void log(const char *label, T value) {
        FastLCD::Write(label);
        FastLCD::Write(": ");
        FastLCD::WriteLine(value);
    }

    extern "C" void PIT1_IRQHandler(void) {
        visualization_timer.begin();

        clear_PIT_irq_flag<1>();

        static bool prev_touching = false;
        static bool display_compass = true;
        static float holding_sec = 0;

        int x, y;
        bool touching = LCD.Touch(&x, &y);
        if (touching && !prev_touching) {
            if (x < LCD_WIDTH / 2) {
                display_compass = !display_compass;
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
        if (display_compass) {
            Mat mat = Mat<3, 3>::RotationTranslation(-robot.angle + rad(180), LCD_WIDTH / 2.0, LCD_HEIGHT / 2.0);
            draw_compass(mat);
        } else {
            FastLCD::SetFontPaletteIndex(White);
//			FastLCD::Write("Tick CPU usage: ");
//			FastLCD::Write((tick_cycles * 100) / (int) cyc(1.0 / TICK_RATE) + 1); // add 1% safety margin
//			FastLCD::WriteLine("%");
//			log("Tick count", (int) robot.tick_count);

//            FastLCD::Write("X/Yin: ");
//            FastLCD::Write(robot.pos.vec[0]);
//            FastLCD::Write(" ");
//            FastLCD::WriteLine(robot.pos.vec[1]);

//            log("Turn radius: ", robot.R);

            log("Angle", deg(robot.angle));
            log("TargetAngle", deg(robot.target_angle));

            log("L Motor Counts", robot.total_counts_l);
            log("R Motor Counts", robot.total_counts_r);

            log("ControlMode", control_mode_string());

            log("ControlEffort", robot.angle_controller.control_effort);
            log("Error", robot.angle_controller.error);
            log("I", robot.angle_controller.I);
            log("CDS Value", robot.light_sensor_value);
            log("Average CDS V", robot.light_sensor_average_value);

            log("Dist", robot.pos.dist(robot.pos0));
            log("TargetDist", robot.target_dist);
            log("DistRemain", robot.dist_remain);
            log("Slewed%", robot.slewed_pct);
//            log("VT", (int) visualization_timer.last_lap_cyc);
//            log("DT", (int) draw_timer.last_lap_cyc);
        }

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

        draw_timer.begin();
        FastLCD::DrawScreen();
        draw_timer.lap();

        visualization_timer.lap();
    }
}

/*
 * EXCEPTION PRIORITY LEVELS - FROM HIGHEST TO LOWEST
 *
 * The K60 has priority levels ranging from 0 to 15, where 0 is highest and 15 is lowest.
 * When an exception becomes pending and another exception with the same priority is running,
 * the pending exception waits for the running exception to end.
 *
 * 0 - PORT{A,B,C,D,E} - DigitalEncoder IRQs
 * 1 - SysTick - Robot Control Loop
 * 15 - PIT1 - Diagnostics/visualization printing
 */

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
     * Set main function priority.
     */
    __set_BASEPRI(1);

    /*
     * Begin the diagnostics printing timer at the lowest possible priority (15).
     * Calls PIT1_IRQHandler() every 0.05 seconds.
     */
    init_PIT<1, 15>(cyc(1.0 / DIAGNOSTICS_HZ));

    /*
     * Initialize the robot control loop by setting up the SysTick timer.
     * Calls SysTick_Handler() at TICK_RATE hz.
     *
     * Priority is 1 because DigitalEncoder PORT IRQs need higher priority.
     */
    SysTick_Config(SYSTICK_INTERVAL_CYCLES);
    NVIC_SetPriority(SysTick_IRQn, 1);

    /*
     * Robot Tasks.
     */

    LeverServo(180);

    Speed(25);

    // Wait for start light to turn on.
    WaitForStartLight();

    // Forward
    Straight(17.811);

    // Turn left toward fuel levers
    Turn(-90);

    // Go toward fuel levers
    Straight(8); // This will vary based on which fuel lever to use.

    // Turn arm toward fuel levers
    Turn(0);
    Straight(-4); // Back into position for the fuel lever flipping

    // Fuel lever flip sequence
    LeverServo(70);
    Sleep(250);
    LeverServo(135);
    Straight(5);
    Sleep(3000);
    LeverServo(45);
    Straight(-5);
    LeverServo(90);
    Sleep(250);
    LeverServo(45);
    Straight(-5);
    LeverServo(180);

    /*
    // Go up ramp.
    Straight(3.92100);
    Turn(45);
    Straight(6);
    Turn(0);
    Straight(29);

    // Turn toward kiosk
    Turn(-45);
    StraightUntilSwitch(28);

    // Go back to line up with ticket light
    Straight(-4.4);
    WaitForTicketLight(3000);

    Straight(-4.832636);
    Turn(-90);

    if (robot.ticket_light_color == TICKET_LIGHT_BLUE) {
        Straight(-2);
        Turn(0);
        StraightUntilSwitch(5.75);

        Straight(-5.844 + 2);

        Turn(-40);

        Straight(-20.672 + 2.5);
    } else if (robot.ticket_light_color == TICKET_LIGHT_RED) {
        Straight(-3 - 2.5);
        Turn(0);
        StraightUntilSwitch(5.75);

        Straight(-12.007 + 2);

        Turn(-40);

        Straight(-13.041 + 1);
    }

    Turn(0);
    Straight(-30);

    // Get correct lever from the RCS
    int correct_lever = RCS.GetCorrectLever();
     */

    stop_robot_control_loop();

    return 0;
}