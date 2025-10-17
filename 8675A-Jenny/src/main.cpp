#include "vex.h"
#include "pid.h"
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

  task o = task(odometry_loop);

  while (!autonomous_started)
  {
    switch (auto_selection)
    {

    case 0:

      Brain.Screen.printAt(10, 10, "null/test");

      break;

    case 1:

      Brain.Screen.printAt(10, 10, "Autonomous: Red 7L");

      break;

    case 2:

      Brain.Screen.printAt(10, 10, "Autonomous: Blue 7L");

      break;

    case 3:

      Brain.Screen.printAt(10, 10, "Autonomous: Red 3S 3L");

      break;

    case 4:

      Brain.Screen.printAt(10, 10, "Autonomous: Blue 3T 3L");

      break;
    }

    if (Brain.Screen.pressing())
    {
      auto_selection++;
    }
    else if (auto_selection > 4)
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

double driveKP = 0.75;
double driveKI = 0.02;
double driveKD = 0.75;

double turnKP = 0.4;
double turnKI = 0;
double turnKD = 0.4;

double swingKP = 0.0;
double swingKI = 0.0;
double swingKD = 0.0;

double lKP = 0.75;
double lKI = 0.02;
double lKD = 0.75;

double aKP = 0.175;
double aKI = 0.0;
double aKD = 0;

double drive_max_volt = 12;
double heading_max_volt = 12;
double drive_min_volt = 0;
double drive_settle_error = 3.0;

void autonomous(void)
{

switch (auto_selection)
  {

  case 0:

  to_point(0, 20);

    //     intake.spin(fwd, 100, pct);
    // drive_set(29, 6, DEFAULT_WAIT);
    // wait(200, msec);
    // turn_set(50, 10, DEFAULT_WAIT);
    // ball_stopper.set(true);
    // wait(200, msec);
    // drive_set(13, 3, DEFAULT_WAIT);
    // wait(200, msec);
    // wait(5, sec);
    // drive_set(-12, 6, DEFAULT_WAIT);


    // intake.spin(fwd, 100, pct);
    // drive_set(26.5, 6, DEFAULT_WAIT);
    // wait(200, msec);
    // turn_set(316, 10, DEFAULT_WAIT);
    // wait(200, msec);
    // drive_set(15, 3, DEFAULT_WAIT);
    // wait(200, msec);
    // intake.spin(reverse, 100, pct);
    // wait(5, sec);
    // drive_set(-12, 6, DEFAULT_WAIT);
    break;
    //
  case 1:
    intake.spin(reverse, 100, pct);
    drive_set(35, 7.5, SettleType::DEFAULT_WAIT);
    break;
    //
  case 2:

    break;
    //
  case 3:

    break;
    //
  case 4:

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
  blueActivate = false;
  redActivate = false;
  Controller1.ButtonB.pressed(height_toggle);
  Controller1.ButtonDown.pressed(unloader_toggle);
  Controller1.ButtonY.pressed(ballstop_toggle);
  Controller1.ButtonUp.pressed(sorter_toggle);
  while (true)
  {
    Left.spin(fwd, (Controller1.Axis3.position()), percent);
    Right.spin(fwd, (Controller1.Axis2.position()), percent);

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
  //startOdometry();

  while (true)
  {
    wait(100, msec);
  }
}
