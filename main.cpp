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

constexpr float IGWAN_COUNTS_PER_REV = 636;
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
constexpr auto ENCODER_L_PIN_0 = FEHIO::FEHIOPin::P3_0;
constexpr auto ENCODER_L_PIN_1 = FEHIO::FEHIOPin::P3_1;
constexpr auto ENCODER_R_PIN_0 = FEHIO::FEHIOPin::P0_0;
constexpr auto ENCODER_R_PIN_1 = FEHIO::FEHIOPin::P0_1;
constexpr auto LIGHT_SENSOR_PIN = FEHIO::FEHIOPin::P2_0;
constexpr auto BUMP_SWITCH_PIN = FEHIO::FEHIOPin::P1_7;
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

 static Mat<3, 3> Translation(float dx, float dy) {
   return Mat<3, 3>{
       1, 0, dx,
       0, 1, dy,
       0, 0, 1
   };
 }
};

void stop_robot_control_loop() {
 SysTick_BASE_PTR->CSR = 0;
}

constexpr Radian rad(Degree deg) {
 return (float) (deg * (M_PI / 180));
}

constexpr Degree deg(Radian rad) {
 return (float) (rad * (180 / M_PI));
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

/**
* Sam's Drivetrain Derivations to calculate velocity over time to drive a given distance with
* Constant Acceleration and Deceleration:
*
* V_max = (-at+sqrt((a^2)(t^2) + 4da)) / 2
*/

enum class ControlMode {
 INIT_TASK,
 TURNING,
 FORWARD,
 FORWARD_UNTIL_SWITCH,
 WAIT_FOR_LIGHT
};

struct RobotTask {
 RobotTask *next_task = nullptr;

 RobotTask();

 virtual void execute() = 0;
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
 Volt light_sensor_value{};
 FEHServo servo{FEHServo::FEHServoPort::Servo0};

 RobotTask *current_task{};
 RobotTask *task_list{};
 ControlMode control_mode = ControlMode::INIT_TASK;

 bool force_start{};
 int last_nonconfident_wait_for_light_tick{};

 volatile int tick_count{}, task_tick_count{}, task_number{};

 float pct_l{}, pct_r{};
 float target_pct{};
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
 Inch dist{};
 float R{};

 int last_encoder_l_tick_at = 0;
 int last_encoder_r_tick_at = 0;
 float stopped_i{};

 [[nodiscard]] const char *control_mode_string() const {
   switch (control_mode) {
     case ControlMode::INIT_TASK:
       return "InitTask";
     case ControlMode::FORWARD:
       return "Forward";
     case ControlMode::FORWARD_UNTIL_SWITCH:
       return "FwdTilSwitch";
     case ControlMode::TURNING:
       return "Turning";
     case ControlMode::WAIT_FOR_LIGHT:
       return "Wait4Light";
   }
   return "?????";
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

   if (last_encoder_l_tick_at + ticks(0.05) < tick_count) {
     stopped_i += STOPPED_I_ACCUMULATE / TICK_RATE;
   }
   if (last_encoder_r_tick_at + ticks(0.05) < tick_count) {
     stopped_i += STOPPED_I_ACCUMULATE / TICK_RATE;
   }
   stopped_i *= STOPPED_I_HIGHPASS;

   el.ResetCounts();
   er.ResetCounts();

   Inch arclength_l = DRIVE_INCHES_PER_COUNT_L * (float) counts_l;
   Inch arclength_r = DRIVE_INCHES_PER_COUNT_R * (float) counts_r;

   Inch arclength_inner;
   if (arclength_l > arclength_r) {
     arclength_inner = arclength_r;
   } else {
     arclength_inner = arclength_l;
   }

   Radian dAngle = (arclength_l - arclength_r) / TRACK_WIDTH;
   Inch radius_inner = fabsf(arclength_inner / dAngle);

   // Let R be the distance from the arc center to the point between the wheels
   R = radius_inner + TRACK_WIDTH / 2;

   Inch dx = R * (cosf(angle + dAngle) - cosf(angle));
   Inch dy = R * (sinf(angle + dAngle) - sinf(angle));

   dist += (arclength_l + arclength_r) / 2;

   pos.vec[0] += dx;
   pos.vec[1] += dy;
   angle += dAngle;
 }

 void task_finished() {
   if (current_task != nullptr) {
     current_task->execute();
     current_task = current_task->next_task;
     task_number++;
   } else {
     // Stop the robot control loop once we've finished the last task
     ml.SetPercent(0);
     mr.SetPercent(0);
     stop_robot_control_loop();
   }
 }

 void motor_power(float new_pct_l, float new_pct_r) {
   pct_l = new_pct_l;
   pct_r = new_pct_r;

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
     case ControlMode::INIT_TASK:
       // The origin is the starting pad.
       // angle = 0 degrees is straight up toward the right ramp,
       // so the bot will be pointed toward -45 degrees when placed on the starting pad.
       pos = {0, 0};
       angle = rad(-45);
       target_angle = rad(-45);

       task_finished();
       return;
     case ControlMode::WAIT_FOR_LIGHT:
       ml.Stop();
       mr.Stop();

       if (light_sensor_value >= WAIT_FOR_LIGHT_THRESHOLD_VOLTAGE) {
         last_nonconfident_wait_for_light_tick = tick_count;
       }
       if (force_start || last_nonconfident_wait_for_light_tick + WAIT_FOR_LIGHT_CONFIRM_TICKS < tick_count) {
         task_finished();
       }
       return;
     case ControlMode::TURNING: {
       Radian angle_turned_so_far = fabs(angle - turn_start_angle);
       Radian angle_remain = fabs(target_angle - angle);
       float slewed_pct = slew(
           TURN_SLEW_RATE,
           TURN_MIN_PERCENT,
           target_pct,
           angle_turned_so_far,
           angle_remain
       );
       slewed_pct += stopped_i * STOPPED_I;

       if (turning_right) {
         motor_power(slewed_pct, -slewed_pct);
         if (angle >= target_angle) {
           task_finished();
         }
       } else {
         motor_power(-slewed_pct, slewed_pct);
         if (angle <= target_angle) {
           task_finished();
         }
       }
       return;
     }
     case ControlMode::FORWARD_UNTIL_SWITCH:
       if (!bump_switch.Value()) {
         task_finished();
         return;
       }
     case ControlMode::FORWARD: {
       control_effort = angle_controller.process(target_angle, angle);

       Inch dist_remain = fabs(target_dist) - fabs(dist);
       float slewed_pct = slew(
           DRIVE_SLEW_RATE,
           DRIVE_MIN_PERCENT,
           target_pct,
           dist,
           dist_remain
       );
       slewed_pct += stopped_i * STOPPED_I;

       if (target_dist < 0) {
         motor_power(-slewed_pct + control_effort, -slewed_pct - control_effort);
       } else {
         motor_power(slewed_pct + control_effort, slewed_pct - control_effort);
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

/*
* Adds the newly created task to the task linked list.
* All inheriting constructors must explicitly run this to add themselves to the task list.
*/
RobotTask::RobotTask() {
 RobotTask **head_ptr = &robot.current_task;

 RobotTask *last = *head_ptr;
 // If the LL head is null, set this node to the LL head, store a pointer to the start of the task list, and return
 if (*head_ptr == nullptr) {
   *head_ptr = this;
   robot.task_list = this;
   return;
 }

 // Traverse LL until we reach a task that doesn't have a next task
 while (last->next_task != nullptr) {
   last = last->next_task;
 }

 // Set this task as the next task of the last task
 last->next_task = this;
}

struct WaitForStartLight : RobotTask {
 // TODO: Implement WaitForLight timeout
 int timeout_ms;

 explicit WaitForStartLight(int timeout_ms) : RobotTask(), timeout_ms(timeout_ms) {}

 void execute() override {
   robot.control_mode = ControlMode::WAIT_FOR_LIGHT;
 }
};

struct Straight : RobotTask {
 Inch inches;

 explicit Straight(Inch inches) : RobotTask(), inches(inches) {};

 void execute() override {
   robot.dist = 0;
   robot.pos0 = robot.pos;
   robot.target_dist = inches;
   robot.angle_controller.reset();
   robot.control_mode = ControlMode::FORWARD;
   robot.stopped_i = 0;
 }
};

struct StraightUntilSwitch : RobotTask {
 Inch inches;

 explicit StraightUntilSwitch(Inch inches) : RobotTask(), inches(inches) {};

 void execute() override {
   robot.dist = 0;
   robot.pos0 = robot.pos;
   robot.target_dist = inches;
   robot.angle_controller.reset();
   robot.control_mode = ControlMode::FORWARD_UNTIL_SWITCH;
   robot.stopped_i = 0;
 }
};

struct Speed : RobotTask {
 float percent;

 explicit Speed(float percent) : RobotTask(), percent(percent) {};

 void execute() override {
   robot.target_pct = percent;
 }
};

struct Turn : RobotTask {
 explicit Turn(Degree angle) : RobotTask(), angle(angle) {};

 Degree angle;

 void execute() override {
   robot.target_angle = (float) rad(angle);
   robot.turn_start_angle = (float) rad(angle);
   robot.turning_right = robot.target_angle > robot.angle;
   robot.control_mode = ControlMode::TURNING;
   robot.stopped_i = 0;
 }
};

struct StampPassport : RobotTask {
 explicit StampPassport() : RobotTask() {};

 void execute() override {
   // TODO
 }
};

struct Position4Bar : RobotTask {
 explicit Position4Bar(Degree target_angle) : RobotTask(), target_angle(target_angle) {};

 float target_angle;

 void execute() override {
   // TODO
 }
};

struct WaitForTicketLight : RobotTask {
 explicit WaitForTicketLight(int ms) : RobotTask() {};

 void execute() override {
   // TODO
 }
};

struct Delay : RobotTask {
 explicit Delay(int ms) : RobotTask(), ms(ms) {};

 int ms;

 void execute() override {
   // TODO
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

CycTimer tick_timer{};
int tick_cycles{};
// This is called every (1 / TICK_RATE) seconds by the SysTick timer
extern "C" void SysTick_Handler(void) {
 tick_timer.begin();
 robot.tick();
 tick_cycles = (int) tick_timer.lap();
}

namespace visualization {
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

constexpr int HALF_GRID_WIDTH = 208;
constexpr float GRID_SQUARE_INCHES = 3.0;
constexpr int GRID_SIZE = 32;

void draw_grid_lines(Mat<3, 3> mat) {
 FastLCD::SetFontPaletteIndex(Gray);
 static_assert((HALF_GRID_WIDTH * 2) % GRID_SIZE == 0);
 for (int s = -HALF_GRID_WIDTH; s < HALF_GRID_WIDTH; s += GRID_SIZE) {
   Vec<3> v1{(float) s, (float) -HALF_GRID_WIDTH, 1};
   Vec<3> v2{(float) s, (float) HALF_GRID_WIDTH, 1};
   v1 = mat.multiply(v1);
   v2 = mat.multiply(v2);
   FastLCD::DrawLine(
       (int) v1.vec[0], (int) v1.vec[1],
       (int) v2.vec[0], (int) v2.vec[1], false);

   Vec<3> h1{(float) -HALF_GRID_WIDTH, (float) s, 1};
   Vec<3> h2{(float) HALF_GRID_WIDTH, (float) s, 1};
   h1 = mat.multiply(h1);
   h2 = mat.multiply(h2);
   FastLCD::DrawLine(
       (int) h1.vec[0], (int) h1.vec[1],
       (int) h2.vec[0], (int) h2.vec[1], false);
 }
}

void draw_task_visualizer(Mat<3, 3> mat) {
 FastLCD::SetFontPaletteIndex(Red);
 RobotTask *head = robot.task_list;
 Radian angle = rad(-45);
 Vec<3> pos{0, 0, 1};
 while (head != nullptr) {
   auto straight = dynamic_cast<Straight *>(head);
   if (straight != nullptr) {
     // put 12 inches in a grid square
     float grid_size_mul = GRID_SIZE / GRID_SQUARE_INCHES;
     float dx = straight->inches * cosf(angle + rad(90)) * grid_size_mul;
     float dy = straight->inches * sinf(angle + rad(90)) * grid_size_mul;

     Vec<3> p1 = mat.multiply(pos);
     Vec<3> p2{dx, dy, 0};
     p2 = p2.add(pos);
     p2 = mat.multiply(p2);

     FastLCD::DrawLine(
         (int) p1.vec[0],
         (int) p1.vec[1],
         (int) p2.vec[0],
         (int) p2.vec[1], true);

     Vec<3> dpos{dx, dy, 0};
     pos = pos.add(dpos);
   }

   auto turn = dynamic_cast<Turn *>(head);
   if (turn != nullptr) {
     angle = rad(turn->angle);
   }

   head = head->next_task;
 }
}


extern "C" void PIT1_IRQHandler(void) {
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

 FastLCD::Clear();
 if (display_compass) {
   Mat mat = Mat<3, 3>::RotationTranslation(-robot.angle + rad(180), LCD_WIDTH / 2.0, LCD_HEIGHT / 2.0);
   draw_grid_lines(mat);
   draw_task_visualizer(mat);
   draw_compass(mat);
 } else {
   FastLCD::SetFontPaletteIndex(White);
   FastLCD::Write("Tick CPU usage: ");
   FastLCD::Write((tick_cycles * 100) / (int) cyc(1.0 / TICK_RATE) + 1); // add 1% safety margin
   FastLCD::WriteLine("%");
   FastLCD::Write("Tick count: ");
   FastLCD::WriteLine((int) robot.tick_count);

   //            FastLCD::Write("X/Ymm: ");
   //            FastLCD::Write(robot.pos.vec[0] * 1000);
   //            FastLCD::Write(" ");
   //            FastLCD::WriteLine(robot.pos.vec[1] * 1000);

   FastLCD::Write("Angle: ");
   FastLCD::WriteLine(deg(robot.angle));
   FastLCD::Write("TargetAngle: ");
   FastLCD::WriteLine(deg(robot.target_angle));

   FastLCD::Write("L Motor Angle: ");
   FastLCD::WriteLine((robot.total_counts_l / IGWAN_COUNTS_PER_REV) * 360);
   FastLCD::Write("R Motor Angle: ");
   FastLCD::WriteLine((robot.total_counts_r / IGWAN_COUNTS_PER_REV) * 360);

   FastLCD::Write("Turn radius: ");
   FastLCD::WriteLine(robot.R);
   FastLCD::Write("ControlMode: ");
   FastLCD::WriteLine(robot.control_mode_string());

   FastLCD::Write("ControlEffort: ");
   FastLCD::WriteLine(robot.angle_controller.control_effort);
   FastLCD::Write("Error: ");
   FastLCD::WriteLine(robot.angle_controller.error);
   FastLCD::Write("I: ");
   FastLCD::WriteLine(robot.angle_controller.I);
   FastLCD::Write("CDS Value: ");
   FastLCD::WriteLine(robot.light_sensor_value);

   FastLCD::Write("Dist: ");
   FastLCD::WriteLine(robot.dist);
   FastLCD::Write("TargetDist: ");
   FastLCD::WriteLine(robot.target_dist);
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

 FastLCD::DrawScreen();
}
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

/*
* Queue up robot tasks. The RobotTask constructor will add these to the queue as they are declared.
* Static variables in the same .cpp file are initialized in order of declaration so this is fine.
*/

Speed t0(50);

// Wait for start light to turn on.
WaitForStartLight t1(3000);

// Go up ramp.
Straight t3(3.92100);
Turn t4(45);
Straight t5(5.581);
Turn t6(0);
Straight t7(26.33900);

// Turn toward kiosk
Turn kiosk1(-45.2);
Straight kiosk2(14.35);
Turn kiosk3(0);
StraightUntilSwitch kiosk4(11);

// Take the exact same path backwards 💀💀💀💀💀💀
Straight down1(-10.076);
Turn down2(-48);
Straight down3(-16);
Turn down4(0);
Straight down5(-26.33900);

/*
// 4. Stamp the passport.
Turn t8(-37.6);
Straight t9(9.537);
Turn t10(0);
Straight t11(6.150);

StampPassport t12();

// 5. Drop the luggage into the high chute.
Straight t13(-6.150);
Turn t14(64.4);
Straight t15(-10.631);
Turn t16(0);
Straight t17(-5.365);
Position4Bar t18(90);
Position4Bar t19(0);

// 6. Go to the ticket booth light, wait for it to turn on, and record its color.
Straight t20(5.365);
Turn t21(-9.6);
Straight t22(17.853);

WaitForTicketLight t23();

// 7. Press the left ticket booth button.
// TODO: Press the correct ticket booth button.
Turn t24(-74.7);
Straight t25(-6.342);
Turn t26(0);
Straight t27(3);
Straight t28(-3);

// 8. Flip the fuel lever, wait 5 seconds, and flip it back up.
Turn t29(-142.8); // Turn toward ramp
Straight t30(26.102); // Go to ramp
Turn t31(-180.0); // Turn down ramp
Position4Bar t32(25); // Raise 4-bar
Straight t33(27.993); // Go down ramp
Position4Bar t34(15); // Lower the lever
Straight t35(-5.830); // Go back a few inches
Position4Bar t36(0); // Lower 4-bar
Delay t37(5000); // Wait 5 seconds
Straight t38(5.830); // Go forward again
Position4Bar t39(10); // Raise the lever

// 9. Push end button.
Straight t40(-5.830);
Turn t41(-72.5);
Straight t42(-14.517);
Turn t43(-45);
Straight t44(-16.267); // Back into the course end button
*/

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
 LCD.Clear();
 LCD.WriteLine("Hello FEH!");
 LCD.WriteLine("SP24 Team C2");
 LCD.WriteLine("\"LOREM IPSUM\"");
 LCD.WriteLine("Initializing Robot...");

 /*
    * Assign colors to palette numbers.
  */
 FastLCD::SetPaletteColor(Black, BLACK);
 FastLCD::SetPaletteColor(White, WHITE);
 FastLCD::SetPaletteColor(Gray, GRAY);
 FastLCD::SetPaletteColor(Red, RED);
 FastLCD::SetPaletteColor(Yellow, YELLOW);

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

 return 0;
}