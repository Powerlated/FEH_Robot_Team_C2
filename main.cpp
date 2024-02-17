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

struct biquad {
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
    float y1;
    float y2;
    float x1;
    float x2;
};

// https://www.musicdsp.org/en/latest/Filters/197-rbj-audio-eq-cookbook.html
float BiQuad(const float sample, biquad* const b)
{
    float result;

    result = b->a0 * sample + b->a1 * b->x1 + b->a2 * b->x2 -
             b->a3 * b->y1 - b->a4 * b->y2;

    b->x2 = b->x1;
    b->x1 = sample;

    b->y2 = b->y1;
    b->y1 = result;

    return result;
}

biquad lpf(float srate, float freq, float q, float dbGain) {
    biquad f;

    float a = pow(10, dbGain /40);
    float omega = 2 * M_PI * freq / srate;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn * sinh(M_LN2 /2 * q * omega /sn);
    float beta = sqrt(a + a);

    float b0 = (1 - cs) /2;
    float b1 = 1 - cs;
    float b2 = (1 - cs) /2;
    float a0 = 1 + alpha;
    float a1 = -2 * cs;
    float a2 = 1 - alpha;


}

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
