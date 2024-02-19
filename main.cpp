#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <FEHRPS.h>
#include <cmath>
#include <Startup/MK60DZ10.h>

void tick();

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

struct pi_controller {
    float sample_rate, sample_time;
    float I{};
    float kP{}, kI{}, kD{};

    explicit pi_controller(float sample_rate) {
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
    // Enable debugging functions including cycle counter
    // 24th bit is TRCENA
    DEMCR |= (0b1 << 24);
    // Enable the cycle counter itself
    // first bit is CYCCNTENA
    DWT_CTRL |= 0b1;
}

// NXP Freescale K60 manual: Chapter 40: Periodic Interrupt Timer (PIT)
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
    PIT_BASE_PTR->CHANNEL[pit_num].TCTRL = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

    // Enable corresponding NVIC interrupt
    NVICISER2 |= 1 << ((INT_PIT0 + pit_num) % 16);
}

const float TICK_RATE = 1000;

extern "C" void PIT1_IRQHandler(void) {
    // Clear PIT1 interrupt flag
    PIT_BASE_PTR->CHANNEL[1].TFLG = 1;
    tick();
}

// This is called every (1 / TICK_RATE) seconds
void tick() {

}

int main() {
    init_cyccount();

    LCD.Clear(BLACK);

    cyctimer pit_check_timer;
    pit_check_timer.begin();
    init_PIT<1>(cyc(1 / TICK_RATE));

    while (true) {}

    return 0;
}
