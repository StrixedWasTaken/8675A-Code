#include "vex.h"
#include "drive_control.h"
#include "screen.h"

using namespace vex;

competition Competition;

void pre_auton(void)
{

  imu.calibrate();
  l2.resetPosition();
  r2.resetPosition();
  vertical_wheel.resetPosition();
  horizontal_wheel.resetPosition();
  //Brain.Screen.print("10010");
  //task sc(printinfo);
  odometry_enabled = true;
  task o(odometry_loop);

  while (!autonomous_started)
  {
    switch (auto_selection)
    {

    case 0:

      Brain.Screen.printAt(10, 20, "null/test");

      break;

    case 1:

      Brain.Screen.printAt(10, 20, "Autonomous: Red/Blue 4 Long 3 Tall");

      break;

    case 2:

      Brain.Screen.printAt(10, 20, "Autonomous: Red/Blue 4 Long 3 Short");

      break;

    case 3:

      Brain.Screen.printAt(10, 20, "Solo AWP Short Side");

      break;

    case 4:
      
      Brain.Screen.printAt(10, 20, "Skills");
      
      break;
    }

    if (Brain.Screen.pressing())
    {
      Brain.Screen.clearScreen();
      auto_selection++;
      wait(300, msec);
    }
    else if (auto_selection > 3)
    {
      auto_selection = 0;
    }
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

double driveKP = 0.5;
double driveKI = 0.0;
double driveKD = 1.7;

double turnKP = 0.4;
double turnKI = 0.01;
double turnKD = 2.2;

double swingKP = 0.0;
double swingKI = 0.0;
double swingKD = 0.0;

double lKP = 0.3;
double lKI = 0;
double lKD = 0.6;

double aKP = 0.4;
double aKI = 0.0;
double aKD = 2.5;

double drive_max_volt = 9;
double heading_max_volt = 8;
double drive_min_volt = 0;
double drive_settle_error = 3.0;

void autonomous(void)
{

  switch (auto_selection)
  {

  case 0:
    //
   skills();
    break;
    //
  case 1:
    BlueRed_4L_3T();
    break;
    //
  case 2:
    BlueRed_4L_3S();
    break;

  case 3:
    SoloAWP_ShortSide();
    break;

  case 4:
    skills();
    break;
  }
}

//---------------------------------------------//

// B height change
// down arror unloader
// Y ball stopper
// up arrow ball sorter

void usercontrol(void)
{

  Controller1.ButtonB.pressed(height_toggle);
  Controller1.ButtonDown.pressed(dropdown_toggle);
  Controller1.ButtonY.pressed(ballstop_toggle);
  l1.setBrake(coast);
  l2.setBrake(coast);
  l3.setBrake(coast);
  r1.setBrake(coast);
  r2.setBrake(coast);
  r3.setBrake(coast);

  while (true)
  {
    
    double leftDrive = Controller1.Axis3.position(pct);
    double rightDrive = Controller1.Axis2.position(pct);

    Left.spin(fwd, leftDrive , percent);
    Right.spin(fwd, rightDrive , percent);

    if (Controller1.ButtonR2.pressing())
    {
      intake.spin(fwd, 100, pct);
    }
    else if (Controller1.ButtonR1.pressing())
    {
      intake.spin(reverse, 100, pct);
    }
    else
    {
      intake.stop();
    }
    if (Controller1.ButtonL1.pressing())
    {
      ball_stopper.set(true);
    }
    else
    {
      ball_stopper.set(false);
    }

    wait(10, msec);
  }
}

int main()
{

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();
  // startOdometry();

  while (true)
  {
    wait(100, msec);
  }
}
