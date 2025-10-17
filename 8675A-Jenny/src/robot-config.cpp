#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;


brain  Brain;

controller Controller1(primary);

motor l1 = motor(PORT16, ratio6_1, true);
motor l2 = motor(PORT14, ratio6_1, true);
motor l3 = motor(PORT12, ratio6_1, true);
motor_group Left = motor_group(l1, l2, l3);

motor r1 = motor(PORT15, ratio6_1, false);
motor r2 = motor(PORT13, ratio6_1, false);
motor r3 = motor(PORT17, ratio6_1, false);
motor_group Right = motor_group(r1, r2, r3);
inertial imu = inertial(PORT7);

motor leftIntake = motor(PORT5, ratio6_1, true);
motor rightIntake = motor(PORT8, ratio6_1, false);
motor all_motors[8] = {l1, l2, l3, r1, r2, r3, leftIntake, rightIntake};

rotation vertical_wheel = rotation(PORT6);
rotation horizontal_wheel = rotation(PORT4, true);

motor_group intake = motor_group(leftIntake, rightIntake);

optical color_sensor = optical(PORT2);

digital_out piston_sorter = digital_out(Brain.ThreeWirePort.A);
digital_out unloader = digital_out(Brain.ThreeWirePort.B);
digital_out ball_stopper = digital_out(Brain.ThreeWirePort.C);
digital_out height_changer = digital_out(Brain.ThreeWirePort.D);

void vexcodeInit( void ) {
  // nothing to initialize
}