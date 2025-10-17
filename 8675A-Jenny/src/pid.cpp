#include "pid.h"
#include "odom.h"

using namespace vex;

double wrap(double input)
{
  return fmod(input + 180.0, 360.0) - 180.0;
}

double wrapRad(double input)
{
  if (input > M_PI)
    input -= 2 * M_PI;
  if (input < -M_PI)
    input += 2 * M_PI;
  return input;
}

double reduce_negative_180_to_180(float angle)
{
  if (!(angle >= -180 && angle < 180))
  {
    if (angle < -180)
    {
      angle += 360;
    }
    if (angle >= 180)
    {
      angle -= 360;
    }
  }
  return (angle);
}

double reduce_negative_90_to_90(float angle)
{
  if (!(angle >= -90 && angle < 90))
  {
    if (angle < -90)
    {
      angle += 180;
    }
    if (angle >= 90)
    {
      angle -= 180;
    }
  }
  return (angle);
}

double reduce_0_to_360(float angle)
{
  if (!(angle >= 0 && angle < 360))
  {
    if (angle < 0)
    {
      angle += 360;
    }
    if (angle >= 360)
    {
      angle -= 360;
    }
  }
  return (angle);
}

double get_absolute_heading()
{
  return (reduce_0_to_360(imu.rotation() * 360.0 / 360));
}

float left_voltage_scaling(float drive_output, float heading_output)
{
  float ratio = std::max(std::fabs(drive_output + heading_output), std::fabs(drive_output - heading_output)) / 12.0;
  if (ratio > 1)
  {
    return (drive_output + heading_output) / ratio;
  }
  return drive_output + heading_output;
}

float right_voltage_scaling(float drive_output, float heading_output)
{
  float ratio = std::max(std::fabs(drive_output + heading_output), std::fabs(drive_output - heading_output)) / 12.0;
  if (ratio > 1)
  {
    return (drive_output - heading_output) / ratio;
  }
  return drive_output - heading_output;
}

float clamp_min_voltage(float drive_output, float drive_min_voltage)
{
  if (drive_output < 0 && drive_output > -drive_min_voltage)
  {
    return -drive_min_voltage;
  }
  if (drive_output > 0 && drive_output < drive_min_voltage)
  {
    return drive_min_voltage;
  }
  return drive_output;
}

// radians to degrees
extern double toDeg(double input)
{
  return input * (180 / M_PI);
}

double toInch(double input, double wheel_diameter, double gear_ratio)
{
  return (input / 360) * (gear_ratio * (M_PI * wheel_diameter));
}

double time_settled = 0;
double timeout = 0;
double max_timeout = 650; // 4.5 seconds is the max amount of time it can be inside of the pid loop for
double derivative;
double integral;
double error;
double previous_error;
double starti = 10;
double output;

// Odometry Variables
double xError;
double yError;
double angularError;
double linearError;
double previousAngularError;
double linearOutput;
double angularOutput;
double leftPower;
double rightPower;
double angularIntegral;
double linearIntegral;
double scale_factor;

bool pidSettled(SettleType motion)
{

  if (time_settled > (static_cast<double>(motion == SettleType::DEFAULT_WAIT)))
  {
    return true;
  }
  else if (timeout >= max_timeout)
  {
    return true;
  }

  if (time_settled > (static_cast<double>(motion == SettleType::QUICK_WAIT)))
  {
    return true;
  }
  else if (timeout >= max_timeout)
  {
    return true;
  }

  if (time_settled > (static_cast<double>(motion == SettleType::MOTION_CHAIN)))
  {
    return true;
  }

  else if (timeout >= max_timeout)
  {
    return true;
  }

  return false;
}

double compute(double cError, double previous_error, double &integral, double kp, double ki, double kd)
{
  starti = 10;

  derivative =
      cError - previous_error; // difference between error and previous error
                               // account for at the end of the loop

  if (fabs(cError) < starti) // if error is less than start integral condition,
  {
    integral += cError; // accumulation of error overtime while in the threshold
  }
  else
  {
    integral = 0; // if error is more than 10, then it sets it to 0 to regulate
                  // accumulation of integral
  }

  output = (cError * kp) + (integral * ki) + (derivative * kd);

  return output;
}

// PID Drives
void drive_set(double target, double mSpeed, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  error = 0;
  integral = 0;
  derivative = 0;

  l2.resetPosition();
  r2.resetPosition();

  while (!pidSettled(motion))
  {

    double wheel_deg = (l2.position(deg) + r2.position(deg)) / 2;

    error = target - toInch(wheel_deg, 2.75, 1); // computes drive error

    output = compute(error, previous_error, integral, driveKP, driveKI, driveKD); // computes whole output using error

    output = clamp(output, -mSpeed, mSpeed); // limits speed based off max speed

    Left.spin(fwd, output, volt);
    Right.spin(fwd, output, volt);

    Brain.Screen.printAt(10, 10, "Error: %2f", error);
    std::cout << "error: " << error << endl;

    previous_error = error;

    timeout++;

    if (fabs(error) < 1 && (motion == SettleType::DEFAULT_WAIT || // if error is less than 1 for regular waits and quick waits, then it exits
                            motion == SettleType::QUICK_WAIT))
    {
      time_settled++;
    }
    else if (fabs(error) < 5 && (motion == SettleType::MOTION_CHAIN))
    { // if error is less than 5 (larger threshold than regular) so it can exit quicker to chain the motions
      time_settled++;
    }
    else
    {
      time_settled = 0;
    }

    wait(10, msec);
  }

  if (motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT)
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
void turn_set(double target, double mSpeed, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  error = 0;
  integral = 0;
  derivative = 0;

  while (!pidSettled(motion))
  {

    double curr_heading = imu.heading(deg);

    error = wrap(target - curr_heading);

    output = compute(error, previous_error, integral, turnKP, turnKI, turnKD); // computes whole output using error

    output = clamp(output, -mSpeed, mSpeed);

    Brain.Screen.printAt(10, 40, "Turn Error: %2f", error);

    Left.spin(fwd, output, volt);
    Right.spin(reverse, output, volt);

    previous_error = error;

    timeout++;

    if ((motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT) && fabs(error) < 1.0)
    {
      time_settled++;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(error) < 5.0)
    {
      time_settled++;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
    }

    wait(10, msec);
  }

  if (motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
  }
}

void to_pose(double xTarget, double yTarget, double angle, double dlead, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  linearError = 0;
  angularError = 0;
  integral = 0;
  derivative = 0;
  targetDistance = 0;
  carrot_X = 0;
  carrot_Y = 0;
  while (!pidSettled(motion))
  {
    xError = xTarget - x;
    yError = yTarget - y;

    targetDistance = hypot(xError, yError);

    carrot_X = xTarget - cos(toRad(angle)) * (dlead * targetDistance + setback);
    carrot_Y = yTarget - sin(toRad(angle)) * (dlead * targetDistance + setback);

    angularError = reduce_negative_180_to_180(toDeg(atan2(xError, yError)) - get_absolute_heading());
    linearError = hypot(carrot_X, carrot_Y);

    linearOutput = compute(linearError, previous_error, linearIntegral, lKP, lKI, lKD);

    scale_factor = cos(toRad(angularError));

    linearOutput *= scale_factor;
    angularError = reduce_negative_90_to_90(angularError);
    // if (linearError > 1.0) { // Avoid atan2(0,0)
    //   angularError = reduce_negative_180_to_180(toDeg(atan2(xError, yError)) - get_absolute_heading());
    // } else {
    //   angularError = 0;
    // }

    angularOutput = compute(angularError, previousAngularError, angularIntegral, aKP, aKI, aKD);

    if (linearError < drive_settle_error)
    {
      angularOutput = 0;
    }

    //-fabs(scale_factor)*drive_max_volt, fabs(scale_factor) * drive_max_volt

    linearOutput = clamp(linearOutput, -fabs(scale_factor) * drive_max_volt, fabs(scale_factor) * drive_max_volt);
    angularOutput = clamp(angularOutput, -heading_max_volt, heading_max_volt);

    linearOutput = clamp_min_voltage(linearOutput, drive_min_volt);

    cout << "linear error: " << linearError << endl;
    cout << "angular error: " << angularError << endl;

    Brain.Screen.printAt(10, 100, "Linear Error: %2f", linearError);
    Brain.Screen.printAt(10, 110, "Angular Error: %2f", angularError);
    Brain.Screen.printAt(10, 120, "Left Power: %2f", leftPower);
    Brain.Screen.printAt(10, 130, "Right Power: %2f", rightPower);

    Left.spin(fwd, left_voltage_scaling(linearOutput, angularOutput), volt);
    Right.spin(fwd, right_voltage_scaling(linearOutput, angularOutput), volt);

    previousAngularError = angularError;
    previous_error = linearError;
    // timeout++;

    if ((motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT) && fabs(linearError) < 1.0 && fabs(angularError) < 1.0)
    {
      time_settled++;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(linearError) < 5.0 && fabs(angularError) < 1)
    {
      time_settled++;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
    }

    wait(10, msec);
  }
  if (motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
  }
}

void to_point(double xTarget, double yTarget, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  linearError = 0;
  angularError = 0;
  integral = 0;
  derivative = 0;
  xError = 0;
  yError = 0;

  while (!pidSettled(motion))
  {

    xError = xTarget - x;
    yError = yTarget - y;

    linearError = hypot(xError, yError);
    // angularError = reduce_negative_180_to_180(toDeg(atan2(yError, xError)) - get_absolute_heading());

    if (linearError > drive_settle_error)
    {
      double targetAngle = toDeg(atan2(xError, yError)); // returns -180 to 180
      angularError = reduce_negative_180_to_180(targetAngle - get_absolute_heading());
    }
    else
    {
      angularError = 0; // Don't bother rotating — you're already there
    }

    linearOutput = compute(linearError, previous_error, linearIntegral, lKP, lKI, lKD);

    if (fabs(angularError) > 90.0)
    {
      linearOutput *= -1;
    }

    scale_factor = cos(toRad(angularError));

    linearOutput *= scale_factor;
    // angularError = reduce_negative_90_to_90(angularError);

    angularOutput = compute(angularError, previousAngularError, angularIntegral, aKP, aKI, aKD);

    if (linearError < drive_settle_error)
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

    Left.spin(fwd, left_voltage_scaling(linearOutput, angularOutput), volt);
    Right.spin(fwd, right_voltage_scaling(linearOutput, angularOutput), volt);

    previousAngularError = angularError;
    previous_error = linearError;

    // timeout++;

    if ((motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT) && fabs(linearError) < drive_settle_error)
    {
      time_settled++;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(linearError) < 5.0)
    {
      time_settled++;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
    }

    wait(10, msec);
  }
  if (motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT)
  {
    Left.stop(brake);
    Right.stop(brake);
  }
  else if (motion == SettleType::MOTION_CHAIN)
  {
    // doesnt stop at all
  }
}

void turn_to(double xTarget, double yTarget, SettleType motion)
{

  time_settled = 0;
  timeout = 0;
  linearError = 0;
  angularError = 0;
  integral = 0;
  derivative = 0;
  xError = 0;
  yError = 0;

  while (!pidSettled(motion))
  {
    xError = xTarget - x;
    yError = yTarget - y;

    angularError = reduce_negative_180_to_180(toDeg(atan2(xError, yError)) - get_absolute_heading());

    angularOutput = compute(angularError, previousAngularError, angularIntegral, aKP, aKI, aKD);

    //-fabs(scale_factor)*drive_max_volt, fabs(scale_factor) * drive_max_volt

    angularOutput = clamp(angularOutput, -heading_max_volt, heading_max_volt);

    cout << "linear error: " << linearError << endl;
    cout << "angular error: " << angularError << endl;

    Brain.Screen.printAt(10, 100, "Linear Error: %2f", linearError);
    Brain.Screen.printAt(10, 110, "Angular Error: %2f", angularError);
    Brain.Screen.printAt(10, 120, "Left Power: %2f", leftPower);
    Brain.Screen.printAt(10, 130, "Right Power: %2f", rightPower);

    Left.spin(fwd, angularOutput, volt);
    Right.spin(reverse, angularOutput, volt);

    previousAngularError = angularError;

    // timeout++;

    if ((motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT) && fabs(linearError) < drive_settle_error && fabs(angularError) < 1.0)
    {
      time_settled++;
    }
    else if (motion == SettleType::MOTION_CHAIN && fabs(linearError) < 5.0 && fabs(angularError) < 1.0)
    {
      time_settled++;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
    }

    wait(10, msec);
  }
  if (motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT)
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
// void swing_set(SwingSet side, double target, double mSpeed, double arcSpeed, SettleType motion)
// {

//   time_settled = 0;
//   timeout = 0;
//   error = 0;
//   integral = 0;
//   derivative = 0;

//   while (!pidSettled(motion))
//   {

//     double curr_heading = imu.heading(deg);

//     error = wrap(target - curr_heading);

//     output = compute(error, previous_error, integral, swingKP, swingKI, swingKD); // computes whole output using error

//     output = clamp(output, -mSpeed, mSpeed);

//     if (side == SwingSet::LEFT_SWING)
//     {

//       Left.spin(fwd, output, volt);

//       if (arcSpeed == 0)
//       {

//         Right.stop(brake);
//       }
//       else if (arcSpeed > 0)
//       {

//         Right.spin(fwd, arcSpeed, pct);
//       }
//     }

//     if (side == SwingSet::RIGHT_SWING)
//     {

//       Right.spin(fwd, output, volt);

//       if (arcSpeed == 0)
//       {

//         Left.stop(brake);
//       }
//       else if (arcSpeed > 0)
//       {

//         Left.spin(fwd, arcSpeed, pct);
//       }
//     }

//     previous_error = error;

//     if ((motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT) && fabs(error) < 1.0)
//     {
//       time_settled++;
//     }
//     else if (motion == SettleType::MOTION_CHAIN && fabs(error) < 5.0)
//     {
//       time_settled++;
//     }
//     else
//     {
//       time_settled = 0; // Reset if error jumps again
//     }
//     wait(10, msec);
//   }
//   if (motion == SettleType::DEFAULT_WAIT || motion == SettleType::QUICK_WAIT)
//   {
//     Left.stop(brake);
//     Right.stop(brake);
//   }
//   else if (motion == SettleType::MOTION_CHAIN)
//   {
//     // doesnt stop at all
//   }
// }