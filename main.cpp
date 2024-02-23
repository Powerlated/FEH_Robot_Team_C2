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

void tick();

void drive();

constexpr float TICK_RATE = 1000;
constexpr float TICK_INTERVAL_MICROSECONDS = (1 / TICK_RATE) * 1000000;
constexpr float TRACK_WIDTH = 8.276;

constexpr float rad(float deg) {
    return deg * (M_PI / 180);
}

constexpr float deg(float rad) {
    return rad * (180 / M_PI);
}

enum class DriveMode {
    STOP,
    FORWARD
};

enum class ControlMode {
    STOP,
    MAINTAIN_ANGLE,
    DIRECT_CONTROL,
};

struct PIController {
    float sample_rate, sample_time;
    float I{}, max_I;
    float kP, kI;

    explicit PIController(float sample_rate, float kP, float kI, float max_I) :
            sample_rate(sample_rate), kP(kP), kI(kI), max_I(max_I) {
        this->sample_time = 1 / sample_rate;
    }

    float process(float setpoint, float process_variable) {
        float error = setpoint - process_variable;

        I += error * sample_time;
        I = fmaxf(-max_I, fminf(max_I, I));

        float control_effort =

                kP * error +
                kI * I;

        return control_effort;
    }

    void reset() {
        I = 0;
    }
};

struct Drivetrain {
    FEHMotor ml, mr;
    DigitalEncoder el, er;
    DriveMode drive_mode = DriveMode::STOP;
    ControlMode control_mode = ControlMode::STOP;

    float inches_per_tick_l{};
    float inches_per_tick_r{};

    float pct_l{}, pct_r{};
    float target_pct_l{}, target_pct_r{};
    float target_dist{};

    int totalcounts_l{}, totalcounts_r{};

    PIController angle_controller = PIController(TICK_RATE, 200, 50, 30);

    // Position is in inches
    float pos_x{}, pos_y{};
    float total_dist{};
    // Angle in radians
    float angle{};
    float angle_to_maintain{};

    explicit Drivetrain(FEHMotor &&ml, FEHMotor &&mr, DigitalEncoder &&el, DigitalEncoder &&er) :
            ml(ml), mr(mr), el(el), er(er) {
    }

    // Takes angle in radians
    void init_pos_and_angle(float _pos_x, float _pos_y, float _angle) {
        pos_x = _pos_x;
        pos_y = _pos_y;
        angle = _angle;
    }

    void set_inches_per_count(float left, float right) {
        inches_per_tick_l = left;
        inches_per_tick_r = right;
    }

    void tick() {
        auto counts_l = el.Counts();
        auto counts_r = er.Counts();

        totalcounts_l += counts_l;
        totalcounts_r += counts_r;
        el.ResetCounts();
        er.ResetCounts();

        float arclength_l = inches_per_tick_l * (float) counts_l;
        float arclength_r = inches_per_tick_r * (float) counts_r;

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

        switch (drive_mode) {
            case DriveMode::STOP:
                break;
            case DriveMode::FORWARD:
                if (total_dist > target_dist) {
                    stop();
                }
                break;
        }

        float control_effort;
        switch (control_mode) {
            case ControlMode::MAINTAIN_ANGLE:
                control_effort = angle_controller.process(angle_to_maintain, angle);

                pct_l = target_pct_l + control_effort;
                pct_r = target_pct_r - control_effort;
                break;
            case ControlMode::STOP:
                pct_l = 0;
                pct_r = 0;
                break;
            case ControlMode::DIRECT_CONTROL:
                pct_l = target_pct_l;
                pct_r = target_pct_r;
                break;
            default:
                break;
        }

        ml.SetPercent(-pct_l);
        mr.SetPercent(-pct_r);
    }

    void maintain_angle() {
        angle_to_maintain = angle;
        angle_controller.reset();

        control_mode = ControlMode::MAINTAIN_ANGLE;
    }

    void drive_in_straight_line(float percent, float dist) {
        target_pct_l = percent;
        target_pct_r = percent;
        target_dist = dist;
        maintain_angle();

        drive_mode = DriveMode::FORWARD;
    }

    void stop() {
        drive_mode = DriveMode::STOP;
        control_mode = ControlMode::STOP;
    }

    [[nodiscard]] const char *drive_mode_string() const {
        switch (drive_mode) {
            case DriveMode::STOP:
                return "Stop";
            case DriveMode::FORWARD:
                return "Forward";
        }
    }

    [[nodiscard]] const char *control_mode_string() const {
        switch (control_mode) {
            case ControlMode::STOP:
                return "Stop";
            case ControlMode::MAINTAIN_ANGLE:
                return "Maintain Angle";
            case ControlMode::DIRECT_CONTROL:
                return "Direct Control";
        }
    }
};

Drivetrain drivetrain(
        FEHMotor(FEHMotor::Motor0, 9.0),
        FEHMotor(FEHMotor::Motor1, 9.0),
        DigitalEncoder(FEHIO::FEHIOPin::P0_0, FEHIO::FEHIOPin::P0_1),
        DigitalEncoder(FEHIO::FEHIOPin::P0_2, FEHIO::FEHIOPin::P0_3)
);

DigitalInputPin
        front_l(FEHIO::FEHIOPin::P2_0),
        front_r(FEHIO::FEHIOPin::P2_1),
        back_l(FEHIO::FEHIOPin::P2_2),
        back_r(FEHIO::FEHIOPin::P2_3);

AnalogInputPin colorSensor(FEHIO::FEHIOPin::P3_0);

FEHServo servo(FEHServo::FEHServoPort::Servo0);

constexpr float PROTEUS_SYSTEM_HZ = 88000000.0;

// Declared in startup_mk60d10.cpp
const int BSP_BUS_DIV = 2;

const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;

const int SERVO_MIN = 500;
const int SERVO_MAX = 2388;

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

constexpr uint32_t cyc(const double sec) {
    return (uint32_t) (sec * PROTEUS_SYSTEM_HZ);
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

template<int pit_num>
void enable_PIT_irq() {
    // Enable interrupt in NVIC
    NVICISER2 |= 1 << ((INT_PIT0 + pit_num) % 16);
}

template<int pit_num>
void disable_PIT_irq() {
    // Disable interrupt in NVIC
    NVICISER2 &= ~(1 << ((INT_PIT0 + pit_num) % 16));
}

// NXP Freescale K60 manual: Chapter 40: Periodic Interrupt Timer (PIT)
template<int pit_num>
void init_PIT(uint32_t cyc_interval) {
    // PIT0 is being used by FEHIO AnalogEncoder and AnalogInputPin, cannot use
    static_assert(pit_num != 0);
    static_assert(pit_num < 4);

    // Enable PIT clock
    PIT_BASE_PTR->MCR = 0;
    // Set up PIT interval
    PIT_BASE_PTR->CHANNEL[pit_num].LDVAL = cyc_interval / BSP_BUS_DIV;
    // Start PIT and enable PIT interrupts
    PIT_BASE_PTR->CHANNEL[pit_num].TCTRL = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

    enable_PIT_irq<pit_num>();
}

template<int pit_num>
void clear_PIT_irq_flag() {
    PIT_BASE_PTR->CHANNEL[pit_num].TFLG = 1;
}

extern "C" void PIT1_IRQHandler(void) {
    clear_PIT_irq_flag<1>();
    tick();
}

int tick_microseconds;
CycTimer tick_timer;

// This is called every (1 / TICK_RATE) seconds by the PIT1 timer IRQ handler
void tick() {
    tick_timer.begin();

    drivetrain.tick();

    int ticks = (int) tick_timer.lap();
    tick_microseconds = (int) (((float) ticks / PROTEUS_SYSTEM_HZ) * 1000000);
}

const double IGWAN_COUNTS_PER_REV = 318;

extern "C" void PIT2_IRQHandler(void) {
    clear_PIT_irq_flag<2>();

    LCD.Clear(BLACK);
    LCD.WriteLine("Tick time (microseconds):");
    LCD.WriteLine(tick_microseconds);
    LCD.WriteLine("Tick budget remaining:");
    LCD.WriteLine((int) TICK_INTERVAL_MICROSECONDS - tick_microseconds);

    LCD.WriteLine("Angle:");
    LCD.WriteLine(deg(drivetrain.angle));

    LCD.WriteLine("Left Motor Angle:");
    LCD.WriteLine((drivetrain.totalcounts_l / IGWAN_COUNTS_PER_REV) * 360);
    LCD.WriteLine("Right Motor Angle:");
    LCD.WriteLine((drivetrain.totalcounts_r / IGWAN_COUNTS_PER_REV) * 360);

    LCD.WriteLine("Drive Mode:");
    LCD.WriteLine(drivetrain.drive_mode_string());

    LCD.WriteLine("Control Mode:");
    LCD.WriteLine(drivetrain.control_mode_string());
}

void sleep(int ms) {
    clear_PIT_irq_flag<3>();

    auto cyc = (uint32_t) ((float) ms * (PROTEUS_SYSTEM_HZ / 1000));

    // Enable PIT clock
    PIT_BASE_PTR->MCR = 0;
    // Set up PIT3 timeout cycles
    PIT_BASE_PTR->CHANNEL[3].LDVAL = cyc / BSP_BUS_DIV;
    // Start PIT3
    PIT_BASE_PTR->CHANNEL[3].TCTRL = 0b11;

    // Wait for PIT3 to expire
    while (!(PIT_BASE_PTR->CHANNEL[3].TFLG & 1));

    clear_PIT_irq_flag<3>();

    // Stop PIT3
    PIT_BASE_PTR->CHANNEL[3].TCTRL = 0;
}

int main() {
    // The origin is the starting pad.
    // angle = 0 degrees is straight up toward the right ramp,
    // so the bot will be pointed toward 315 degrees when placed on the starting pad.
    drivetrain.init_pos_and_angle(0, 0, rad(315));

    const double WHEEL_DIA = 2.5;
    const double INCHES_PER_REV = WHEEL_DIA * M_PI;
    const auto INCHES_PER_COUNT = (float) (INCHES_PER_REV / IGWAN_COUNTS_PER_REV);

    drivetrain.set_inches_per_count(INCHES_PER_COUNT, INCHES_PER_COUNT);

    // Begin the tick loop timer (PIT1_IRQHandler)
    init_PIT<1>(cyc(1 / TICK_RATE));

    // Begin the metrics printing timer (PIT2_IRQHandler)
    init_PIT<2>(cyc(0.2));

    drive();

    return 0;
}

void drive() {
    while (true) {
        // Drive forward for 12 inches at 25% speed
        drivetrain.drive_in_straight_line(25, 12);
        sleep(2000);
        drivetrain.stop();
        sleep(2000);
    }
}