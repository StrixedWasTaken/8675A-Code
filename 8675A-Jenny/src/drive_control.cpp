#include "drive_control.h"
#include "odom.h"
#include "pid.h"

using namespace vex;

double time_settled = 0;
double derivative;
double integral;
double error;
double previous_error;
double output;

// Odometry Variables
double xError;
double yError;
double angularError;
double linearError;
double linearOutput;
double angularOutput;
double leftPower;
double rightPower;
double scale_factor;
double targetDistance;
double carrot_X;
double carrot_Y;
double setback = 1;
double targetAngle;
int timeout = 0;
double settled_time = 0;
double start_time = 0;
double previous_output;

bool pidSettled(SettleType motion, int time_settled, int timeout, int max_timeout)
{
  const int DEFAULT_SETTLE_TIME = 125;
  const int CHAIN_SETTLE_TIME = 0;

  if (timeout >= max_timeout)
    return true;

  switch (motion)
  {
  case SettleType::DEFAULT_WAIT:
    return time_settled > DEFAULT_SETTLE_TIME;
  case SettleType::MOTION_CHAIN:
    return time_settled > CHAIN_SETTLE_TIME;
  default:
    return false;
  }
}

double slew_rateD = 10;
// PID Drives
void drive_set(double target, double mSpeed, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  error = 0;
  integral = 0;
  derivative = 0;
  previous_output = 0;
  l2.resetPosition();
  r2.resetPosition();
  PID drivePID(driveKP, driveKI, driveKD);
  drivePID.setIntegralLimits(8, 3000);
  drivePID.set_constants(2.5, 3000);

  start_time = Brain.timer(msec);
  settled_time = Brain.timer(msec);

  while (!pidSettled(motion, time_settled, timeout, drivePID.max_timeout))
  {

    double wheel_deg = ((l2.position(deg) + r2.position(deg)) / 2) * wheel_inches;

    error = target - wheel_deg;

    output = drivePID.update(error);

    output = slew(output, previous_output, slew_rateD);
    output = clamp(output, -mSpeed, mSpeed); // limits speed based off max speed
    
    timeout = Brain.timer(msec) - start_time;

    Left.spin(fwd, output, volt);
    Right.spin(fwd, output, volt);

    Brain.Screen.printAt(10, 10, "Error: %2f", error);
    std::cout << "error: " << error << endl;

    previous_error = error;
    previous_output;
    if (fabs(error) < drivePID.settle_error && (motion == SettleType::DEFAULT_WAIT))
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else if (fabs(error) < 5.0 && (motion == SettleType::MOTION_CHAIN))
    { // if error is less than 5 (larger threshold than regular) so it can exit quicker to chain the motions
      time_settled = Brain.timer(msec) - settled_time;
    }
    else
    {
      settled_time = Brain.timer(msec);
      time_settled = 0;
    }

    wait(10, msec);
  }

  if (motion == SettleType::DEFAULT_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
    std::cout << "\033[32msettled\033[0m" << std::endl;
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    //
  }
}

double slew_rateT = 9;
// PID Swings
void turn_set(double target, double mSpeed, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  error = 0;
  integral = 0;
  derivative = 0;
  PID turnPID(turnKP, turnKI, turnKD);
  turnPID.setIntegralLimits(6, 3000);
  turnPID.set_constants(1.5, 2000);

  settled_time = Brain.timer(msec);
  start_time = Brain.timer(msec);

  while (!pidSettled(motion, time_settled, timeout, turnPID.max_timeout))
  {

    error = normalizeTarget(target - imu.rotation(deg));

    output = turnPID.update(error);

    output = slew(output, previous_output, slew_rateT);
    output = clamp(output, -mSpeed, mSpeed);

    
    Brain.Screen.printAt(10, 40, "Turn Error: %2f", error);
    cout << "turn error: " << error << endl;
    Left.spin(fwd, output, volt);
    Right.spin(reverse, output, volt);

    timeout++;

    if ((motion == SettleType::DEFAULT_WAIT) && fabs(error) < turnPID.settle_error)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(error) < 5.0)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else
    {
      settled_time = Brain.timer(msec);
      time_settled = 0;
    }

    wait(10, msec);
  }

  if (motion == SettleType::DEFAULT_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
    std::cout << "\033[32msettled\033[0m" << std::endl;
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
    std::cout << "\033[32msettled\033[0m" << std::endl;
  }
}

void to_pose(double xTarget, double yTarget, double angle, double dlead, double dir, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  linearError = 0;
  angularError = 0;
  integral = 0;
  derivative = 0;
  int add = dir > 0 ? 0 : 180;
  targetDistance = 0;
  carrot_X = 0;
  carrot_Y = 0;
  setback = 1;

  PID linearPID(lKP, lKI, lKD);
  PID angularPID(aKP, aKI, aKD);
  linearPID.setIntegralLimits(5, 1000);
  angularPID.setIntegralLimits(3, 1000);
  linearPID.set_constants(3.0, 4000);
  angularPID.set_constants(3.0, 4000);

  float start_angle_deg = toDeg(atan2(xTarget - x, yTarget - y));
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(xTarget, yTarget, start_angle_deg, x, y);

  bool center_line_side = is_line_settled(xTarget, yTarget, start_angle_deg, x, y);
  bool crossed_center_line = false;
  bool prev_center_line_side = center_line_side;

  settled_time = Brain.timer(msec);
  start_time = Brain.timer(msec);

  while (!pidSettled(motion, time_settled, timeout, linearPID.max_timeout))
  {

    line_settled = is_line_settled(xTarget, yTarget, start_angle_deg, x, y);
    if (line_settled && !prev_line_settled)
    {
      time_settled = 100;
    }
    prev_line_settled = line_settled;

    // center_line_side = is_line_settled(xTarget, yTarget, angle + 90, x, y);
    // if (center_line_side != prev_center_line_side)
    // {
    //   crossed_center_line = true;
    // }

    xError = xTarget - x;
    yError = yTarget - y;

    targetDistance = hypot(xError, yError) * dir;

    carrot_X = xTarget - targetDistance * sin(toRad(angle + add)) * dlead;
    carrot_Y = yTarget - targetDistance * cos(toRad(angle + add)) * dlead;

    angularError = reduce_negative_180_to_180(toDeg(atan2(carrot_X - x, carrot_Y - y)) - get_absolute_heading());

    linearError = hypot(carrot_X - x, carrot_Y - y);

    if (linearError > 8)
    {

      angularError = reduce_negative_180_to_180(toDeg(atan2(carrot_X - x, carrot_Y - y)) - get_absolute_heading());
    }
    else if (fabs(targetDistance) > 6)
    {
      angularError = reduce_negative_180_to_180(toDeg(atan2(xError, yError)) - get_absolute_heading() + add);
    }
    else
    {
      angularError = reduce_negative_180_to_180(angle - get_absolute_heading());
    }

    angularError = sin(toRad(angularError)) * linearError;

    linearOutput = linearPID.update(linearError);

    scale_factor = cos(toRad(angularError));

    linearOutput *= scale_factor;

    angularError = reduce_negative_90_to_90(angularError);

    angularOutput = angularPID.update(angularError);

    if (linearError < linearPID.settle_error)
    {
      angularOutput = 0;
    }

    linearOutput = clamp(linearOutput, -fabs(scale_factor) * drive_max_volt, fabs(scale_factor) * drive_max_volt);
    angularOutput = clamp(angularOutput, -heading_max_volt, heading_max_volt);

    linearOutput = clamp_min_voltage(linearOutput, drive_min_volt);

    cout << "linear error: " << linearError << endl;
    cout << "angular error: " << angularError << endl;

    Brain.Screen.printAt(10, 100, "Linear Error: %2f", linearError);
    Brain.Screen.printAt(10, 110, "Angular Error: %2f", angularError);
    Brain.Screen.printAt(10, 120, "Left Power: %2f", leftPower);
    Brain.Screen.printAt(10, 130, "Right Power: %2f", rightPower);

    timeout = Brain.timer(msec) - start_time;

    Left.spin(fwd, left_voltage_scaling(linearOutput, angularOutput), volt);
    Right.spin(fwd, right_voltage_scaling(linearOutput, angularOutput), volt);

    // timeout++;

    if ((motion == SettleType::DEFAULT_WAIT) && fabs(linearError) < linearPID.settle_error && fabs(angularError) < angularPID.settle_error)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(linearError) < 5.0)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
      settled_time = Brain.timer(msec);
    }

    wait(10, msec);
  }
  if (motion == SettleType::DEFAULT_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
  }
}

double prevLinear;
double prevAngular;
void to_point(double xTarget, double yTarget, int dir, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  linearError = 0;
  angularError = 0;
  integral = 0;
  derivative = 0;
  xError = 0;
  yError = 0;
  int add = dir > 0 ? 0 : 180;
  PID linearPID(lKP, lKI, lKD);
  PID angularPID(aKP, aKI, aKD);
  linearPID.setIntegralLimits(5, 3000);
  angularPID.setIntegralLimits(3, 3000);
  linearPID.set_constants(3, 4000);
  angularPID.set_constants(3, 4000);

  float start_angle_deg = toDeg(atan2(xTarget - x, yTarget - y));
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(xTarget, yTarget, start_angle_deg, x, y);

  start_time = Brain.timer(msec);
  settled_time = Brain.timer(msec);

  while (!pidSettled(motion, time_settled, timeout, linearPID.max_timeout))
  {
    line_settled = is_line_settled(xTarget, yTarget, start_angle_deg, x, y);
    if (line_settled && !prev_line_settled)
    {
      time_settled = 100;
    }
    prev_line_settled = line_settled;

    xError = xTarget - x;
    yError = yTarget - y;

    linearError = hypot(xError, yError) * dir;
    targetAngle = toDeg(atan2(xError, yError));
    angularError = reduce_negative_180_to_180(targetAngle - get_absolute_heading()) + add;

    angularError = sin(toRad(angularError)) * linearError;

    linearOutput = linearPID.update(linearError);
    
    scale_factor = cos(toRad(angularError));

    linearOutput *= scale_factor;

    angularOutput = angularPID.update(angularError);

    if (fabs(linearError) < linearPID.settle_error)
    {
      angularOutput = 0;
    }

    linearOutput = clamp(linearOutput, -fabs(scale_factor) * drive_max_volt, fabs(scale_factor) * drive_max_volt);
    angularOutput = clamp(angularOutput, -heading_max_volt, heading_max_volt);

    linearOutput = clamp_min_voltage(linearOutput, drive_min_volt);

    // Overturn logic for sharp turns

    // double overturn_value = fabs(leftPower) + fabs(angularOutput) - heading_max_volt;
    // if (overturn_value > 0)
    // {
    //   if (leftPower > 0)
    //   {
    //     leftPower -= overturn_value;
    //   }
    //   else
    //   {
    //     rightPower += overturn_value;
    //   }
    // }
    // rightPower = leftPower;

    leftPower = left_voltage_scaling(linearOutput, angularOutput);
    rightPower = right_voltage_scaling(linearOutput, angularOutput);
    
    if (dir == -1)
    {
      swap(leftPower, rightPower);
    }
    cout << "linear error: " << linearError << endl;
    cout << "angular error: " << angularError << endl;

    Brain.Screen.printAt(10, 100, "Linear Error: %2f", linearError);
    Brain.Screen.printAt(10, 110, "Angular Error: %2f", angularError);
    Brain.Screen.printAt(10, 120, "Left Power: %2f", leftPower);
    Brain.Screen.printAt(10, 130, "Right Power: %2f", rightPower);

    timeout = Brain.timer(msec) - start_time;
    Left.spin(fwd, leftPower, volt);
    Right.spin(fwd, rightPower, volt);

    if ((motion == SettleType::DEFAULT_WAIT) && fabs(linearError) < linearPID.settle_error && fabs(angularError) < angularPID.settle_error)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(linearError) < 5.0)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
      settled_time = Brain.timer(msec);
    }

    wait(10, msec);
  }
  if (motion == SettleType::DEFAULT_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
  }
}

// PID Swings
void left_swing(double target, double mSpeed, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  error = 0;
  integral = 0;
  derivative = 0;

  PID leftSwingPID(swingKP, swingKI, swingKD);
  leftSwingPID.setIntegralLimits(5, 1000);
  leftSwingPID.set_constants(1, 400);

  settled_time = Brain.timer(msec);
  start_time = Brain.timer(msec);

  while (!pidSettled(motion, time_settled, timeout, leftSwingPID.max_timeout))
  {

    error = target - get_absolute_heading();

    output = leftSwingPID.update(error);

    output = clamp(output, -mSpeed, mSpeed);
    cout << "error: " << error << endl;

    timeout = Brain.timer(msec) - start_time;
    Left.spin(fwd, output, volt);


    if ((motion == SettleType::DEFAULT_WAIT) && fabs(error) < leftSwingPID.settle_error)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(error) < 5.0)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
      settled_time = Brain.timer(msec);
    }
    wait(10, msec);
  }
  if (motion == SettleType::DEFAULT_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
  }
}

void right_swing(double target, double mSpeed, double arcSpeed, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  error = 0;
  integral = 0;
  derivative = 0;

  PID rightSwingPID(swingKP, swingKI, swingKD);
  rightSwingPID.setIntegralLimits(3, 1000);
  rightSwingPID.set_constants(1, 400);

  settled_time = Brain.timer(msec);
  start_time = Brain.timer(msec);

  while (!pidSettled(motion, time_settled, timeout, rightSwingPID.max_timeout))
  {

    error = target - get_absolute_heading();

    output = rightSwingPID.update(error);

    output = clamp(output, -mSpeed, mSpeed);
    cout << "error: " << error << endl;

    Right.spin(fwd, output, volt);

    if ((motion == SettleType::DEFAULT_WAIT) && fabs(error) < rightSwingPID.settle_error)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(error) < 5.0)
    {
      time_settled = Brain.timer(msec) - settled_time;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
      settled_time = Brain.timer(msec);
    }
    wait(10, msec);
  }
  if (motion == SettleType::DEFAULT_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
  }
}