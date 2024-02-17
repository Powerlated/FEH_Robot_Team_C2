#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include "FEHIO.h"
#include <cmath>

FEHMotor motorL(FEHMotor::Motor0, 9.0);
FEHMotor motorR(FEHMotor::Motor1, 9.0);

DigitalEncoder encL(FEHIO::FEHIOPin::P0_0);

DigitalEncoder encR(FEHIO::FEHIOPin::P0_1);

DigitalInputPin frontL(FEHIO::FEHIOPin::P2_0);

DigitalInputPin frontR(FEHIO::FEHIOPin::P2_1);

DigitalInputPin backL(FEHIO::FEHIOPin::P2_2);

DigitalInputPin backR(FEHIO::FEHIOPin::P2_3);

AnalogInputPin colorSensor(FEHIO::FEHIOPin::P3_0);

FEHServo servo(FEHServo::FEHServoPort::Servo0);

void left(float powerPct) {
    powerPct *= -0.25;
    motorL.SetPercent(powerPct);
}

void left_ms(float powerPct, int ms) {
    left(powerPct);
    Sleep(ms);
    left(0);
}

void right(float powerPct) {
    powerPct *= -0.25;
    motorR.SetPercent(powerPct);
}

void right_ms(float powerPct, int ms) {
    right(powerPct);
    Sleep(ms);
    right(0);
}

void drive(float powerPct) {
    powerPct *= -0.25;

    motorL.SetPercent(powerPct);
    motorR.SetPercent(powerPct);
}

void drive_ms(float powerPct, int ms) {
    drive(powerPct);
    Sleep(ms);
    drive(0);
}

const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;

/**
 * Formulas taken from RBJ's Audio EQ Cookbook:
 * https://www.w3.org/TR/audio-eq-cookbook/
 */
class biquad {
public:
    float c0, c1, c2, c3, c4;
    float y1, y2;
    float x1, x2;

    constexpr static biquad lpf(float fS, float fC, float q) {
        float w = 2 * (float) M_PI * (fC / fS);
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

const int SERVO_MIN = 500;
const int SERVO_MAX = 2388;

void fwd_until_bump() {
    LCD.WriteLine("Going forward until front");
    LCD.WriteLine("bump right is hit");
    while (frontR.Value()) {
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
    while (backL.Value() || backR.Value()) {
        drive(-100);
    }
    LCD.WriteLine("Hit back bumps,");
    LCD.WriteLine("stopping in 100 ms...");
    Sleep(100);
    drive(0);
}

int main(void) {
    LCD.Clear(BLACK);

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
