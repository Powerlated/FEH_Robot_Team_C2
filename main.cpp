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
 * Proteus constants.
 */
constexpr int BSP_BUS_DIV = 2; // Declared in startup_mk60d10.cpp
constexpr int TICK_RATE = 100;
constexpr int PROTEUS_SYSTEM_HZ = 88000000.0;
constexpr int LCD_WIDTH = 320;
constexpr int LCD_HEIGHT = 240;

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

    constexpr uint32_t cyc(const double sec) {
        return (uint32_t)(sec * PROTEUS_SYSTEM_HZ);
    }

    constexpr uint32_t ticks(const double sec) {
        return (uint32_t)(sec * TICK_RATE);
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

    // NXP Freescale K60 manual: Chapter 40: Periodic Interrupt Timer (PIT)
    template<int pit_num, int irq_priority>
    void init_PIT(uint32_t cyc_interval) {
        // PIT0 is being used by FEHIO AnalogEncoder and AnalogInputPin, cannot use
        static_assert(pit_num != 0);
        static_assert(pit_num < 4);

        // Set priority
        NVIC_SetPriority((IRQn)(PIT0_IRQn + pit_num), irq_priority);

        // Enable PIT clock
        PIT_BASE_PTR->MCR = 0;
        // Set up PIT interval
        PIT_BASE_PTR->CHANNEL[pit_num].LDVAL = cyc_interval / BSP_BUS_DIV;
        // Start PIT and enable PIT interrupts
        PIT_BASE_PTR->CHANNEL[pit_num].TCTRL = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

        // Enable interrupt in NVIC
        NVIC_EnableIRQ((IRQn)(PIT0_IRQn + pit_num));
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
constexpr int SYSTICK_INTERVAL_CYCLES = cyc(1.0 / TICK_RATE);
constexpr float IGWAN_COUNTS_PER_REV = 636;
constexpr float TRACK_WIDTH = 8.35;
constexpr float WHEEL_DIA = 2.5;
constexpr float INCHES_PER_REV = WHEEL_DIA * M_PI;
constexpr float INCHES_PER_COUNT = (float) (INCHES_PER_REV / IGWAN_COUNTS_PER_REV);
constexpr float DRIVE_MOTOR_MAX_VOLTAGE = 9.0;

/*
 * Robot control configuration.
 */
constexpr float DRIVE_MIN_PERCENT = 10;
constexpr float TURN_MIN_PERCENT = 10;
constexpr float STOPPED_I = 10;
constexpr float STOPPED_I_ACCUMULATE = 3;
constexpr float STOPPED_I_HIGHPASS = 0.999;
constexpr float START_LIGHT_THRESHOLD_VOLTAGE = 1;
constexpr int WAIT_FOR_LIGHT_CONFIDENT_MS = 500;
constexpr int TICKET_LIGHT_AVERAGING_MS = 100;
constexpr int SWITCH_CONFIDENT_TICKS = 50;

constexpr Vec<2> INITIAL_POS{0, 0};
constexpr float INITIAL_ANGLE = rad(-45);

/*
 * Diagnostics/visualization configuration.
 */
constexpr int DIAGNOSTICS_HZ = 5;
constexpr float TICK_INTERVAL_MICROSECONDS = (1.0 / TICK_RATE) * 1000000;
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

    enum class ControlMode {
        INIT,
        TURN,
        STRAIGHT,
        STRAIGHT_UNTIL_SWITCH,
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

        ControlMode control_mode = ControlMode::INIT;

        // Start Light / Ticket Light variables
        TicketLightColor ticket_light_color = TICKET_LIGHT_NONE;

        int tick_count{}, task_tick_count{};
        volatile bool task_running{};
        volatile bool force_start{};

        int switch_pressed_ticks{};

        float target_speed = 50;
        float drive_slew_rate = 200; // Percent per m/s
        float turn_slew_rate = 400; // Percent per radian/s

        /* START DEBUG VARIABLES */
        volatile float cds_red_value{};
        volatile float cds_blue_value{};

        float pct_l{}, pct_r{};
        float slewed_pct{};
        float dist_remain{};
        float target_dist{};
        int total_counts_l{}, total_counts_r{};

        float R{};
        /* END DEBUG VARIABLES */

        PIController angle_controller = PIController(TICK_RATE, 100, 50, 30);

        // Position is in inches
        Vec<2> pos{}, pos0{};

        /* START TURN VARIABLES */
        // Angle in radians
        float angle{};
        float target_angle{};
        float turn_start_angle{};
        bool turning_right{};
        float turn_wheel_bias{};
        /* END TURN VARIABLES */

        /* START UNSTUCK VARIABLES */
        int last_encoder_l_tick_at = 0;
        int last_encoder_r_tick_at = 0;
        float stopped_i{};
        /* END UNSTUCK VARIABLES */

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

            if (turn_wheel_bias < 0.8) {  
                if (last_encoder_l_tick_at + ticks(0.025) < tick_count) {
                    stopped_i += STOPPED_I_ACCUMULATE / TICK_RATE;
                }
            }
            if (turn_wheel_bias > -0.8) {
                if (last_encoder_r_tick_at + ticks(0.025) < tick_count) {
                    stopped_i += STOPPED_I_ACCUMULATE / TICK_RATE;
                }
            }
            stopped_i *= STOPPED_I_HIGHPASS;

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
             * ODOMETRY
             */

            process_odometry();

            /*
             * DRIVETRAIN MOTOR MANAGEMENT
             */

            float control_effort;
            switch (control_mode) {
                case ControlMode::TURN: {
                    float angle_turned_so_far = fabs(angle - turn_start_angle);
                    float angle_remain = fabs(target_angle - angle);
                    slewed_pct = slew(
                            turn_slew_rate,
                            TURN_MIN_PERCENT,
                            target_speed,
                            angle_turned_so_far,
                            angle_remain
                    );
                    float power_pct = slewed_pct + stopped_i * STOPPED_I;

                    float new_pct_l, new_pct_r;
                    if (turning_right) {
                        new_pct_l = power_pct;
                        new_pct_r = -power_pct;

                        if (angle >= target_angle) {
                            task_finished();
                        }
                    } else {
                        new_pct_l = -power_pct;
                        new_pct_r = power_pct;

                        if (angle <= target_angle) {
                            task_finished();
                        }
                    }

                    if (turn_wheel_bias < 0) {
                        new_pct_r *= 1 - fabs(turn_wheel_bias);
                        new_pct_l *= fabs(turn_wheel_bias) + 1;
                    } else if (turn_wheel_bias > 0) {
                        new_pct_l *= 1 - fabs(turn_wheel_bias);
                        new_pct_r *= fabs(turn_wheel_bias) + 1;
                    }

                    motor_power(new_pct_l, new_pct_r);
                    return;
                }
                case ControlMode::STRAIGHT_UNTIL_SWITCH:
                    if (!l_bump_switch.Value() && !r_bump_switch.Value()) {
                        switch_pressed_ticks++;
                    } else if (!button_bump_switch.Value()) {
                        switch_pressed_ticks++;
                    } else {
                        switch_pressed_ticks = 0;
                    }

                    if (switch_pressed_ticks >= SWITCH_CONFIDENT_TICKS) {
                        switch_pressed_ticks = 0;
                        task_finished();
                        return;
                    }
                case ControlMode::STRAIGHT: {
                    control_effort = angle_controller.process(target_angle, angle);

                    float dist = pos.dist(pos0);
                    dist_remain = fabs(target_dist) - fabs(dist);
                    slewed_pct = slew(
                            drive_slew_rate,
                            DRIVE_MIN_PERCENT,
                            target_speed,
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

    void Straight_prepare(float inches) {
        robot.pos0 = robot.pos;
        robot.target_dist = inches;
        robot.angle_controller.reset();
        robot.control_mode = ControlMode::STRAIGHT;
        robot.stopped_i = 0;
    }

    void Straight(float inches) {
        Straight_prepare(inches);
        wait_for_task_to_finish();
    }

    void StraightUntilSwitch(float inches) {
        Straight_prepare(inches);
        robot.control_mode = ControlMode::STRAIGHT_UNTIL_SWITCH;
        wait_for_task_to_finish();
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

    void Turn_prepare(float degree, float turn_wheel_bias) {
        robot.turn_wheel_bias = turn_wheel_bias;
        robot.target_angle = (float) rad(degree);
        robot.turn_start_angle = robot.angle;
        robot.turning_right = robot.target_angle > robot.angle;
        robot.stopped_i = 0;
        robot.control_mode = ControlMode::TURN;
    }

    void Turn(float degree) {
        Turn_prepare(degree, 0);
        wait_for_task_to_finish();
    }

    void Pivot(float degree, float turn_wheel_bias) {
        Turn_prepare(degree, turn_wheel_bias);
        wait_for_task_to_finish();
    }

    void PivotLeft(float degree) {
        // Bias 1 = right wheel turning only = pivoting on left
        Turn_prepare(degree, 1);
        wait_for_task_to_finish();
    }

    void PivotRight(float degree) {
        // Bias -1 = left wheel turning only = pivoting on right
        Turn_prepare(degree, -1);
        wait_for_task_to_finish();
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
            case ControlMode::TURN:
                return "Turn";
        }
        return "?????";
    }

    template<typename T>
    void log(const char *label, T value) {
        FastLCD::Write(label);
        FastLCD::Write(": ");
        FastLCD::WriteLine(value);
    }

    extern "C" void PIT1_IRQHandler(void) {
        clear_PIT_irq_flag<1>();

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
//      FastLCD::Write("X/Yin: ");
//      FastLCD::Write(robot.pos.vec[0]);
//      FastLCD::Write(" ");
//      FastLCD::WriteLine(robot.pos.vec[1]);

//      log("Turn radius: ", robot.R);

        log("Angle", deg(robot.angle));
        log("TargetAngle", deg(robot.target_angle));

//      log("L Motor Counts", robot.total_counts_l);
//      log("R Motor Counts", robot.total_counts_r);

        log("ControlMode", control_mode_string());

//      log("ControlEffort", robot.angle_controller.control_effort);
//      log("Error", robot.angle_controller.error);
        log("I", robot.angle_controller.I);

        log("CDS  Red", robot.cds_red_value);
        log("CDS Blue", robot.cds_blue_value);

        log("Dist", robot.pos.dist(robot.pos0));
        log("TargetDist", robot.target_dist);

        log("FuelLever", RCS.GetCorrectLever());
//      log("DistRemain", robot.dist_remain);
//      log("Slewed%", robot.slewed_pct);


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

void init() {
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
}

int main() {
    init();

    FuelServo(180);
    DumptruckServo(180);
    PassportServo(90);

    DriveSlewRate(400);
    TurnSlewRate(600);
    Speed(90);

    /*
     * PATH START!
     */

    // Wait for start light to turn on.
    WaitForStartLight();
    // Forward
    Straight(18.5);

    // Turn left toward fuel levers
    Turn(-90);

    // Levers are 3.5 inches apart
    float leverMinus = float(2 - RCS.GetCorrectLever()) * 3.5f;

    // Go toward fuel levers
    Straight(8 - leverMinus); // This will vary based on which fuel lever to use.

    // Turn arm toward fuel levers
    Turn(0);
    Straight(-4.75); // Back into position for the fuel lever flipping

    // Fuel lever flip sequence
    FuelServo(72);
    Sleep(250);
    FuelServo(135);
    Straight(2);
    Sleep(5000);
    FuelServo(45);
    Straight(-2);
    FuelServo(90);
    Sleep(250);
    FuelServo(45);
    Straight(2);
    FuelServo(180);

    // Turn right
    Turn(90);

    // Square with the left wall
    StraightUntilSwitch(-(7.0f + leverMinus));
    ResetFacing(90);

    // Face up ramp
    Straight(0.5);
    PivotLeft(0);

    // Approach the ramp slowly and then go fast up toward it to avoid slippage
    Straight(2);
    Straight(23);

    // Go to luggage drop
    Pivot(180, -0.7);
    StraightUntilSwitch(6);
    ResetFacing(180);

    // Drop the luggage
    DumptruckServo(90);
    Sleep(500);
    DumptruckServo(180);

    // Ticket light
    Straight(-16.7);
    Turn(135);

    // TODO: Go to kiosk depending on light color. This is just for the blue light so far.
    StraightUntilSwitch(-16);
    ResetFacing(135);
    Straight(2);
    CaptureTicketLight();
    Straight(5);
    Turn(90);
    Straight(6);
    PivotRight(0);
    StraightUntilSwitch(4);

    PivotRight(-30);
    PivotLeft(0);

    Straight(2);

    // Press the high button
    DumptruckServo(0);
    Sleep(500);
    DumptruckServo(180);

    // TODO: Passport mech

    // TODO: Go down right side ramp and hit the end button
}