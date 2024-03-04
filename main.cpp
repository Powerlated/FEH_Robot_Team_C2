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

#include "CMSIS/MK60D10.h"
#include <FastLCD.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <cmath>
#include <typeinfo>

using namespace std;

using Radian = float;
using Volt = float;
using Degree = float;
using Microsecond = float;
using Second = float;
using Inch = float;
using Hertz = int;

constexpr Hertz PROTEUS_SYSTEM_HZ = 88000000.0;

constexpr uint32_t cyc(const double sec) {
	return (uint32_t) (sec * PROTEUS_SYSTEM_HZ);
}

constexpr Hertz TICK_RATE = 1000;
constexpr int SYSTICK_INTERVAL_CYCLES = cyc(1.0 / TICK_RATE);

constexpr uint32_t ticks(const double sec) {
	return (uint32_t) (sec * TICK_RATE);
}

constexpr Hertz DIAGNOSTICS_HZ = 20;
constexpr Microsecond TICK_INTERVAL_MICROSECONDS = (1.0 / TICK_RATE) * 1000000;
constexpr Inch TRACK_WIDTH = 8.276;

constexpr float IGWAN_COUNTS_PER_REV = 318;
constexpr Inch WHEEL_DIA = 2.5;
constexpr Inch INCHES_PER_REV = WHEEL_DIA * M_PI;
constexpr Inch INCHES_PER_COUNT = (float) (INCHES_PER_REV / IGWAN_COUNTS_PER_REV);
constexpr float DRIVE_SLEW_RATE = 200; // Percent per m/s
constexpr float TURN_SLEW_RATE = 50; // Percent per radian/s
constexpr float DRIVE_MIN_PERCENT = 0;
constexpr float TURN_MIN_PERCENT = 0;
constexpr float STOPPED_I = 10;
constexpr float STOPPED_I_ACCUMULATE = 6;
constexpr float STOPPED_I_HIGHPASS = 0.999;
constexpr float WAIT_FOR_LIGHT_THRESHOLD_VOLTAGE = 0.5;
constexpr int WAIT_FOR_LIGHT_CONFIRM_TICKS = ticks(0.1);

constexpr Second FORCE_START_HOLD_SEC = 0.5;
constexpr Second FORCE_START_TOTAL_SEC = 1;

constexpr Volt DRIVE_MOTOR_MAX_VOLTAGE = 9.0;
constexpr auto DRIVE_MOTOR_L_PORT = FEHMotor::Motor0;
constexpr auto DRIVE_MOTOR_R_PORT = FEHMotor::Motor1;
constexpr auto ENCODER_L_PIN = FEHIO::FEHIOPin::P1_1;
constexpr auto ENCODER_R_PIN = FEHIO::FEHIOPin::P1_0;
constexpr auto LIGHT_SENSOR_PIN = FEHIO::FEHIOPin::P2_0;
constexpr Inch DRIVE_INCHES_PER_COUNT_L = INCHES_PER_COUNT;
constexpr Inch DRIVE_INCHES_PER_COUNT_R = INCHES_PER_COUNT;

// Declared in startup_mk60d10.cpp
constexpr int BSP_BUS_DIV = 2;

constexpr int LCD_WIDTH = 320;
constexpr int LCD_HEIGHT = 240;

enum {
	Black = 0,
	White,
	Gray,
	Red,
	Yellow,
} PaletteColors;

AnalogInputPin right_opto(FEHIO::P0_0);
AnalogInputPin middle_opto(FEHIO::P0_1);
AnalogInputPin left_opto(FEHIO::P0_2);

int main() {
	float x, y;

	FastLCD::SetPaletteColor(Black, BLACK);
	FastLCD::SetPaletteColor(White, WHITE);

	FastLCD::SetFontPaletteIndex(White);
	FastLCD::Clear();
	while (true) {
		FastLCD::WriteLine(" ");
		FastLCD::WriteLine("Press screen to gather");
		FastLCD::WriteLine("optosensor data. (1000");
		FastLCD::WriteLine("samples)");
		FastLCD::DrawScreen();

		FastLCD::Clear();

		while (!LCD.Touch(&x, &y)); //Wait for screen to be pressed
		while (LCD.Touch(&x, &y)); //Wait for screen to be unpressed

		float s_right = 0;
		float s_middle = 0;
		float s_left = 0;

		constexpr int SAMPLES = 1000;
		for (int i = 0; i < SAMPLES; i++) {
			s_right += right_opto.Value();
			s_middle += middle_opto.Value();
			s_left += left_opto.Value();
		}

		s_right /= SAMPLES;
		s_middle /= SAMPLES;
		s_left /= SAMPLES;

		FastLCD::Write(" Right: ");
		FastLCD::WriteLine(s_right);
		FastLCD::Write("Middle: ");
		FastLCD::WriteLine(s_middle);
		FastLCD::Write("  Left: ");
		FastLCD::WriteLine(s_left);
	}
}