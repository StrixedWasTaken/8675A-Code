#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;


brain  Brain;

controller Controller1(primary);

motor l1 = motor(PORT2, ratio6_1, true);//
motor l2 = motor(PORT4, ratio6_1, true);//
motor l3 = motor(PORT3, ratio6_1, true);//
motor_group Left = motor_group(l1, l2, l3);

motor r1 = motor(PORT8, ratio6_1, false);
motor r2 = motor(PORT20, ratio6_1, false);//
motor r3 = motor(PORT6, ratio6_1, false);//
motor_group Right = motor_group(r1, r2, r3);
inertial imu = inertial(PORT18);

motor frontIntake = motor(PORT5, ratio6_1, true);//
motor middleIntake = motor(PORT19, ratio6_1, true);
motor topIntake = motor(PORT13, ratio18_1, true);

motor all_motors[8] = {l1, l2, l3, r1, r2, r3, frontIntake, middleIntake};

rotation vertical_wheel = rotation(PORT11, true);
rotation horizontal_wheel = rotation(PORT7);

motor_group intake = motor_group(frontIntake, middleIntake, topIntake);

digital_out dropdown = digital_out(Brain.ThreeWirePort.A);
digital_out ball_stopper = digital_out(Brain.ThreeWirePort.B);
digital_out height_changer = digital_out(Brain.ThreeWirePort.C);

void vexcodeInit( void ) {
  // nothing to initialize
}