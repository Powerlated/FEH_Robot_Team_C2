#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include "FEHIO.h"
#include <cmath>
#include <Startup/MK60DZ10.h>

FEHMotor
        motor_l(FEHMotor::Motor0, 9.0),
        motor_r(FEHMotor::Motor1, 9.0);

DigitalInputPin
        front_l(FEHIO::FEHIOPin::P2_0),
        front_r(FEHIO::FEHIOPin::P2_1),
        back_l(FEHIO::FEHIOPin::P2_2),
        back_r(FEHIO::FEHIOPin::P2_3);

AnalogInputPin colorSensor(FEHIO::FEHIOPin::P3_0);

FEHServo servo(FEHServo::FEHServoPort::Servo0);

DigitalEncoder enc_l(FEHIO::FEHIOPin::P0_0), enc_r(FEHIO::FEHIOPin::P0_1);

constexpr double PROTEUS_SYSTEM_HZ = 88000000.0;

// Declared in startup_mk60d10.cpp
const int BSP_BUS_DIV = 2;

const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;

const int SERVO_MIN = 500;
const int SERVO_MAX = 2388;

void left(float powerPct) {
    powerPct *= -0.25;
    motor_l.SetPercent(powerPct);
}

void left_ms(float powerPct, int ms) {
    left(powerPct);
    Sleep(ms);
    left(0);
}

void right(float powerPct) {
    powerPct *= -0.25;
    motor_r.SetPercent(powerPct);
}

void right_ms(float powerPct, int ms) {
    right(powerPct);
    Sleep(ms);
    right(0);
}

void drive(float powerPct) {
    powerPct *= -0.25;

    motor_l.SetPercent(powerPct);
    motor_r.SetPercent(powerPct);
}

void drive_ms(float powerPct, int ms) {
    drive(powerPct);
    Sleep(ms);
    drive(0);
}

struct pi_controller {
    float sample_rate, sample_time;
    float I;
    float kP, kI, kD;

    pi_controller(float sample_rate) {
        this->sample_rate = sample_rate;
        this->sample_time = 1 / sample_rate;
    }

    void process(float setpoint, float process_variable) {
        float error = setpoint - process_variable;

        I += error * sample_time;

        float control =
                kP * error +
                kI * I;
    }
};

/**
 * Formulas taken from RBJ's Audio EQ Cookbook:
 * https://www.w3.org/TR/audio-eq-cookbook/
 */
struct biquad {
    float c0, c1, c2, c3, c4;
    float y1, y2;
    float x1, x2;

    constexpr static biquad lpf(float sample_rate, float cutoff_freq, float q) {
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

        return biquad{
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

void fwd_until_bump() {
    LCD.WriteLine("Going forward until front");
    LCD.WriteLine("bump right is hit");
    while (front_r.Value()) {
        drive(100);
    }
    LCD.WriteLine("Hit front bump right,");
    LCD.WriteLine("stopping in 100 ms...");
    Sleep(100);
    drive(0);
}

void back_until_bump() {
    LCD.WriteLine("Going back until back");
    LCD.WriteLine("bumps are hit");
    while (back_l.Value() || back_r.Value()) {
        drive(-100);
    }
    LCD.WriteLine("Hit back bumps,");
    LCD.WriteLine("stopping in 100 ms...");
    Sleep(100);
    drive(0);
}

constexpr uint32_t cyc(const double sec) {
    return (uint32_t) (sec * PROTEUS_SYSTEM_HZ);
}

// Don't use this for waits longer than 2^32 cycles or about 40 seconds!
void cycsleep(uint32_t cycles) {
    constexpr uint32_t sleep_offset = 24;
    // Compensate for a constant lateness that this code produces
    if (cycles >= sleep_offset) {
        cycles -= sleep_offset;
    }
    uint32_t cyccnt_begin = DWT_CYCCNT;
    while (DWT_CYCCNT - cyccnt_begin < cycles);
}

struct cyctimer {
    uint32_t value = 0;

    void begin() {
        value = DWT_CYCCNT;
    }

    uint32_t lap() {
        uint32_t current_cyc = DWT_CYCCNT;
        uint32_t lap_cyc = current_cyc - value;
        value = current_cyc;
        return lap_cyc;
    }
};

void init_cyccount() {
    // Initialize cycle counter
    // 24th bit is TRCENA
    DEMCR |= (0b1 << 24);
    // first bit is CYCCNTENA
    DWT_CTRL |= 0b1;
}

// K60 manual: Chapter 40: Periodic Interrupt Timer (PIT)
template <int pit_num>
void init_PIT(uint32_t cyc_interval) {
    // PIT0 is being used by FEHIO AnalogEncoder and AnalogInputPin, cannot use
    static_assert(pit_num != 0);
    static_assert(pit_num < 4);

    // Enable PIT clock
    PIT_BASE_PTR->MCR = 0;
    // Set up PIT interval
    PIT_BASE_PTR->CHANNEL[pit_num].LDVAL = cyc_interval / BSP_BUS_DIV;
    // Start PIT and enable PIT interrupts
    PIT_BASE_PTR->CHANNEL[pit_num].TCTRL = 0b11;

    // Clear pending PIT interrupt in NVIC
    NVICICPR2 |= 1 << ((INT_PIT0 + pit_num) % 16);
    // Enable PIT in NVIC
    NVICISER2 |= 1 << ((INT_PIT0 + pit_num) % 16);
}

cyctimer pit_check_timer;

int biquad_ops;
const float PIT1_HZ = 1;
const float ENCODER_SAMPLE_RATE = 1;

extern "C" void PIT1_IRQHandler(void) {
    // Clear PIT1 interrupt flag
    PIT_BASE_PTR->CHANNEL[1].TFLG = 1;

    LCD.Write(biquad_ops * PIT1_HZ);
    LCD.WriteLine(" Hz");
    biquad_ops = 0;
}

int main() {
    init_cyccount();

    LCD.Clear(BLACK);
    LCD.WriteLine("Hello World!");

    cyctimer pit_check_timer;
    pit_check_timer.begin();
    init_PIT<1>(cyc(1 / PIT1_HZ));

    biquad b = biquad::lpf(ENCODER_SAMPLE_RATE, 100, 0.5);
    while (true) {
        b.process((float)rand());
        biquad_ops++;
    }

//    servo.SetMin(SERVO_MIN);
//    servo.SetMax(SERVO_MAX);
//
//    while (true) {
//        float cds = colorSensor.Value();
//        float servo_proportion = cds / 3.3;
//        float servo_deg = servo_proportion * 180;
//        servo.SetDegree(servo_deg);
//    }

    fwd_until_bump();
//    drive_ms(-100, 1000);
    right_ms(-100, 3000);
    back_until_bump();
    fwd_until_bump();
    left_ms(-100, 3000);
    back_until_bump();
    fwd_until_bump();

    return 0;
}
