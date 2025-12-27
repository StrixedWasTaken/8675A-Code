#pragma once
using namespace vex;

extern brain Brain;
extern controller Controller1;

// VEXcode devices
extern motor l1;
extern motor l2;
extern motor l3;

extern motor_group Left;

extern motor r1;
extern motor r2;
extern motor r3;
extern motor_group Right;

extern inertial imu;

extern rotation vertical_wheel;

extern rotation horizontal_wheel;

extern optical color_sensor;

extern motor_group intake;

extern motor frontIntake;
extern motor middleIntake;
extern motor topIntake;

extern digital_out dropdown;
extern digital_out height_changer;
extern digital_out ball_stopper;
extern motor all_motors[9];

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );