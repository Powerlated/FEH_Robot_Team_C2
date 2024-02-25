/* *
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
 * */

#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <cmath>
#include <Startup/MK60DZ10.h>
#include <array>

using namespace std;

constexpr int TICK_RATE = 5000;
constexpr float TICK_INTERVAL_MICROSECONDS = (1.0 / TICK_RATE) * 1000000;
constexpr float TRACK_WIDTH = 8.276;

constexpr double IGWAN_COUNTS_PER_REV = 318;
constexpr double WHEEL_DIA = 2.5;
constexpr double INCHES_PER_REV = WHEEL_DIA * M_PI;
constexpr auto INCHES_PER_COUNT = (float) (INCHES_PER_REV / IGWAN_COUNTS_PER_REV);

constexpr float PROTEUS_SYSTEM_HZ = 88000000.0;

constexpr auto DRIVE_MOTOR_MAX_VOLTAGE = 9.0;
constexpr auto DRIVE_MOTOR_L_PORT = FEHMotor::Motor0;
constexpr auto DRIVE_MOTOR_R_PORT = FEHMotor::Motor1;
constexpr auto ENCODER_L_PIN_0 = FEHIO::FEHIOPin::P0_0;
constexpr auto ENCODER_L_PIN_1 = FEHIO::FEHIOPin::P0_1;
constexpr auto ENCODER_R_PIN_0 = FEHIO::FEHIOPin::P0_2;
constexpr auto ENCODER_R_PIN_1 = FEHIO::FEHIOPin::P0_3;
constexpr auto DRIVE_INCHES_PER_COUNT_L = INCHES_PER_COUNT;
constexpr auto DRIVE_INCHES_PER_COUNT_R = INCHES_PER_COUNT;

// Declared in startup_mk60d10.cpp
constexpr int BSP_BUS_DIV = 2;

constexpr int LCD_WIDTH = 320;
constexpr int LCD_HEIGHT = 240;

constexpr int SERVO_MIN = 500;
constexpr int SERVO_MAX = 2388;

constexpr uint32_t cyc(const double sec) {
    return (uint32_t) (sec * PROTEUS_SYSTEM_HZ);
}

constexpr int SYSTICK_INTERVAL_CYCLES = cyc(1.0 / TICK_RATE);

[[noreturn]] void robot_control_loop() {
    // Make sure SYSTICK_INTERVAL_CYCLES fits into 24 bits for SYSTICK_RVR
    static_assert(!(SYSTICK_INTERVAL_CYCLES & 0xFF000000));
    SysTick_BASE_PTR->RVR = SYSTICK_INTERVAL_CYCLES;

    // Register SYST_CSR
    // CLKSOURCE - bit 2 - use processor clock
    // TICKINT - bit 1 - enable SysTick interrupt
    // ENABLE - bit 0 - enable Sysick
    SysTick_BASE_PTR->CSR |= 0b111;

    // Enable SysTick IRQ in NVIC
    NVIC_BASE_PTR->ISER[0] |= 1 << INT_SysTick;

    while (true);
}

constexpr double rad(double deg) {
    return deg * (M_PI / 180);
}

constexpr double deg(double rad) {
    return rad * (180 / M_PI);
}

struct CycTimer {
    uint32_t value = 0;

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
        return lap_cyc;
    }
};

struct PIController {
    float sample_rate, sample_time;
    float error{};
    float kP, kI;
    float I{}, max_I;
    float control_effort{};

    explicit PIController(float sample_rate, float kP, float kI, float max_I) :
            sample_rate(sample_rate), kP(kP), kI(kI), max_I(max_I) {
        this->sample_time = 1 / sample_rate;
    }

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

/**
 * Sam's Drivetrain Derivations to calculate velocity over time to drive a given distance with
 * Constant Acceleration and Deceleration:
 *
 * V_max = (-at+sqrt((a^2)(t^2) + 4da)) / 2
 */

enum class ControlMode {
    STOP,
    TURNING,
    FORWARD,
    WAIT_FOR_LIGHT
};

struct RobotTask {
    RobotTask *next_task{};

    RobotTask();

    virtual void execute() = 0;
};

struct Robot {
    FEHMotor ml{DRIVE_MOTOR_L_PORT, DRIVE_MOTOR_MAX_VOLTAGE};
    FEHMotor mr{DRIVE_MOTOR_R_PORT, DRIVE_MOTOR_MAX_VOLTAGE};
    DigitalEncoder el{ENCODER_L_PIN_0, ENCODER_L_PIN_1};
    DigitalEncoder er{ENCODER_R_PIN_0, ENCODER_R_PIN_1};
    DigitalInputPin
            front_l{FEHIO::FEHIOPin::P2_0},
            front_r{FEHIO::FEHIOPin::P2_1},
            back_l{FEHIO::FEHIOPin::P2_2},
            back_r{FEHIO::FEHIOPin::P2_3};
    AnalogInputPin colorSensor{FEHIO::FEHIOPin::P3_0};
    FEHServo servo{FEHServo::FEHServoPort::Servo0};

    RobotTask *current_task{};
    ControlMode control_mode = ControlMode::STOP;

    volatile int tick_count{}, task_tick_count{};

    float pct_l{}, pct_r{};
    float target_pct_l{}, target_pct_r{};
    float target_dist{};

    int total_counts_l{}, total_counts_r{};

    PIController angle_controller = PIController(TICK_RATE, 200, 50, 30);

    // Position is in inches
    float pos_x{}, pos_y{};
    float total_dist{};
    // Angle in radians
    float angle{};
    float target_angle{};

    /*
     * Because my modified DigitalEncoder code obtains a pointer to itself using the “this” keyword so that the port
     * IRQs can update the object state, we MUST avoid using a copy of the object and use the original object
     * that we instantiated. I couldn't get this working properly with C++ move semantics, so I'm instantiating the
     * DigitalEncoder objects inside the Drivetrain constructor instead. It's working now.
     */
    explicit Robot() {
        // The origin is the starting pad.
        // angle = 0 degrees is straight up toward the right ramp,
        // so the bot will be pointed toward -45 degrees when placed on the starting pad.
        pos_x = 0;
        pos_y = 0;
        angle = rad(-45);
    }

    [[nodiscard]] const char *control_mode_string() const {
        switch (control_mode) {
            case ControlMode::STOP:
                return "Stop";
            case ControlMode::FORWARD:
                return "Forward";
            case ControlMode::TURNING:
                return "Turning";
            case ControlMode::WAIT_FOR_LIGHT:
                return "Wait4Light";
        }
        return "";
    }

    void process_odometry() {
        auto counts_l = el.Counts();
        auto counts_r = er.Counts();

        total_counts_l += counts_l;
        total_counts_r += counts_r;
        el.ResetCounts();
        er.ResetCounts();

        float arclength_l = DRIVE_INCHES_PER_COUNT_L * (float) counts_l;
        float arclength_r = DRIVE_INCHES_PER_COUNT_R * (float) counts_r;

        float arclength_inner;
        if (arclength_l > arclength_r) {
            arclength_inner = arclength_r;
        } else {
            arclength_inner = arclength_l;
        }

        float dAngle = (arclength_l - arclength_r) / TRACK_WIDTH;
        float radius_inner = fabsf(arclength_inner / dAngle);

        // Let R be the distance from the arc center to the point between the wheels
        float R = radius_inner + TRACK_WIDTH / 2;

        float fwd_dist = R * sinf(dAngle);
        float dx = R * cosf(angle + dAngle) - cosf(angle);
        float dy = R * sinf(angle + dAngle) - sinf(angle);

        total_dist += fwd_dist;
        pos_x += dx;
        pos_y += dy;
        angle += dAngle;
    }

    void action_finished() const {
        current_task->execute();
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
            case ControlMode::WAIT_FOR_LIGHT:
                ml.Stop();
                mr.Stop();
                // TODO: Wait for light
                break;
            case ControlMode::TURNING:
                break;
            case ControlMode::FORWARD:
                control_effort = angle_controller.process(target_angle, angle);

                pct_l = target_pct_l + control_effort;
                pct_r = target_pct_r - control_effort;

                ml.SetPercent(-pct_l);
                mr.SetPercent(-pct_r);

                if (total_dist > target_dist) {
                    action_finished();
                }
                break;
            case ControlMode::STOP:
                pct_l = 0;
                pct_r = 0;

                ml.Stop();
                mr.Stop();
                action_finished();
                break;
            default:
                break;
        }
    }
} robot;

/*
 * In C++, a parent constructor is implicitly called by all inheriting constructors.
 */
RobotTask::RobotTask() {
    RobotTask **head_ptr = &robot.current_task;

    RobotTask *last = *head_ptr;
    // If the LL head is null, set this node to the LL head and return
    if (*head_ptr == nullptr) {
        *head_ptr = this;
        return;
    }

    // Traverse LL until we reach a task that doesn't have a next task
    while (last->next_task != nullptr) {
        last = last->next_task;
    }

    // Set this task as the next task of the last task
    last->next_task = this;
};

struct WaitForLight : RobotTask {
    // TODO: Implement WaitForLight timeout
    int timeout_ms;

    explicit WaitForLight(int timeout_ms) : timeout_ms(timeout_ms) {}

    void execute() override {
        robot.control_mode = ControlMode::WAIT_FOR_LIGHT;
    }
};

struct Straight : RobotTask {
    float percent, dist;

    explicit Straight(float percent, float dist) : percent(percent), dist(dist) {};

    void execute() override {
        robot.target_pct_l = percent;
        robot.target_pct_r = percent;
        robot.target_dist = dist;
        robot.target_angle = robot.angle;
        robot.angle_controller.reset();
        robot.control_mode = ControlMode::FORWARD;
    }
};

struct Stop : RobotTask {
    void execute() override {
        robot.control_mode = ControlMode::STOP;
    }
};

struct Turn : RobotTask {
    float turn_angle;

    explicit Turn(float turn_angle) : turn_angle(turn_angle) {};

    void execute() override {
        robot.target_angle = turn_angle;
        robot.control_mode = ControlMode::TURNING;
    }
};

/**
 * Formulas taken from RBJ's Audio EQ Cookbook:
 * https://www.w3.org/TR/audio-eq-cookbook/
 */
struct Biquad {
    float c0, c1, c2, c3, c4;
    float y1, y2;
    float x1, x2;

    constexpr static Biquad lpf(float sample_rate, float cutoff_freq, float q) {
        float w = 2 * (float) M_PI * (cutoff_freq / sample_rate);
        float a = sinf(w) / (2 * q);

        float b0 = (1 - cosf(w)) / 2;
        float b1 = 1 - cosf(w);
        float b2 = (1 - cosf(w)) / 2;

        float a0 = 1 + a;
        float a1 = -2 * cosf(w);
        float a2 = 1 - a;

        float c0 = b0 / a0;
        float c1 = b1 / a0;
        float c2 = b2 / a0;
        float c3 = a1 / a0;
        float c4 = a2 / a0;

        return Biquad{
                c0, c1, c2, c3, c4,
                0, 0,
                0, 0
        };
    }

    // Direct Form 1
    float process(const float in) {
        float result = c0 * in + c1 * x1 + c2 * x2 - c3 * y1 - c4 * y2;

        x2 = x1;
        x1 = in;

        y2 = y1;
        y1 = result;

        return result;
    }
};

// NXP Freescale K60 manual: Chapter 40: Periodic Interrupt Timer (PIT)
template<int pit_num, int irq_priority>
void init_PIT(uint32_t cyc_interval) {
    // PIT0 is being used by FEHIO AnalogEncoder and AnalogInputPin, cannot use
    static_assert(pit_num != 0);
    static_assert(pit_num < 4);

    // Set priority
    NVIC_BASE_PTR->IP[(INT_PIT0 - 16) + pit_num] = irq_priority;

    // Enable PIT clock
    PIT_BASE_PTR->MCR = 0;
    // Set up PIT interval
    PIT_BASE_PTR->CHANNEL[pit_num].LDVAL = cyc_interval / BSP_BUS_DIV;
    // Start PIT and enable PIT interrupts
    PIT_BASE_PTR->CHANNEL[pit_num].TCTRL = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

    // Enable interrupt in NVIC
    NVICISER2 |= 1 << ((INT_PIT0 + pit_num) % 16);
}

template<int pit_num>
void clear_PIT_irq_flag() {
    PIT_BASE_PTR->CHANNEL[pit_num].TFLG = 1;
}

CycTimer tick_timer{};
int tick_cycles{};
// This is called every (1 / TICK_RATE) seconds by the SysTick timer
extern "C" void SysTick_Handler(void) {
    tick_timer.begin();
    robot.tick();
    tick_cycles = (int) tick_timer.lap();
}

extern "C" void PIT1_IRQHandler(void) {
    clear_PIT_irq_flag<1>();

    LCD.Clear(BLACK);
    LCD.Write("Tick CPU usage: ");
    // Add 1% to give a safety margin
    LCD.Write((tick_cycles * 100) / (int) cyc(1.0 / TICK_RATE) + 1);
    LCD.WriteLine("%");
    LCD.Write("Tick count: ");
    LCD.WriteLine((int) robot.tick_count);

    LCD.Write("X (inches): ");
    LCD.WriteLine(robot.pos_x);
    LCD.Write("Y (inches): ");
    LCD.WriteLine(robot.pos_y);

    LCD.Write("Angle: ");
    LCD.WriteLine(deg(robot.angle));

    LCD.Write("L Motor Angle: ");
    LCD.WriteLine((robot.total_counts_l / IGWAN_COUNTS_PER_REV) * 360);
    LCD.Write("R Motor Angle: ");
    LCD.WriteLine((robot.total_counts_r / IGWAN_COUNTS_PER_REV) * 360);

    LCD.Write("ControlMode: ");
    LCD.WriteLine(robot.control_mode_string());

    LCD.Write("ControlEffort: ");
    LCD.WriteLine(robot.angle_controller.control_effort);
    LCD.Write("Error: ");
    LCD.WriteLine(robot.angle_controller.error);
    LCD.Write("I: ");
    LCD.WriteLine(robot.angle_controller.I);
}

void sleep(int ms) {
    // Make sure PIT3 IRQ flag isn't on
    clear_PIT_irq_flag<3>();

    auto cyc = (uint32_t) ((float) ms * (PROTEUS_SYSTEM_HZ / 1000));

    // Stop PIT3 just in case
    PIT_BASE_PTR->CHANNEL[3].TCTRL = 0;

    // Enable PIT clock
    PIT_BASE_PTR->MCR = 0;
    // Set up PIT3 timeout cycles
    PIT_BASE_PTR->CHANNEL[3].LDVAL = cyc / BSP_BUS_DIV;
    // Start PIT3
    PIT_BASE_PTR->CHANNEL[3].TCTRL = 0b11;

    // Wait for the PIT3 IRQ flag to show up
    while (!(PIT_BASE_PTR->CHANNEL[3].TFLG & 1));

    // Clear the PIT3 IRQ flag
    clear_PIT_irq_flag<3>();

    // Stop PIT3
    PIT_BASE_PTR->CHANNEL[3].TCTRL = 0;
}

int main() {
    /*
     * Queue up robot tasks. The RobotTask constructor will add these to the queue as they are declared.
     */
    WaitForLight(3000);
    Straight(50, -3);
    Straight(100, 6);
    Turn(45);
    Straight(100, 3);
    Turn(0);
    Stop();

    /*
     * Begin the diagnostics printing timer at the lowest possible priority (15).
     * Calls PIT1_IRQHandler() every 0.2 seconds.
     */
    init_PIT<1, 15>(cyc(0.2));

    /*
     * Initialize the robot control loop by setting up the SysTick timer.
     * Calls SysTick_Handler() at TICK_RATE hz.
     *
     * BECAUSE robot_control_loop() DOESN'T RETURN, main() DOESN'T RETURN.
     * main() MUST NOT RETURN BECAUSE THE QUEUED ROBOT TASKS MUST REMAIN IN THE STACK.
     */
    robot_control_loop();
}