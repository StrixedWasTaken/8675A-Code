#include "utils.h"

double wrapRad(double input)
{
    if (input > M_PI)
        input -= 2 * M_PI;
    if (input < -M_PI)
        input += 2 * M_PI;
    return input;
}

double reduce_negative_180_to_180(double angle)
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

double reduce_negative_90_to_90(double angle)
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

double reduce_0_to_360(double angle)
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

double left_voltage_scaling(double drive_output, double heading_output)
{
    double ratio = std::max(std::fabs(drive_output + heading_output), std::fabs(drive_output - heading_output)) / 12.0;
    if (ratio > 1)
    {
        return (drive_output + heading_output) / ratio;
    }
    return drive_output + heading_output;
}

double right_voltage_scaling(double drive_output, double heading_output)
{
    double ratio = std::max(std::fabs(drive_output + heading_output), std::fabs(drive_output - heading_output)) / 12.0;
    if (ratio > 1)
    {
        return (drive_output - heading_output) / ratio;
    }
    return drive_output - heading_output;
}

double clamp_min_voltage(double drive_output, double drive_min_voltage)
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

double toRad(double input)
{
    return input * (M_PI / 180);
}

bool is_line_settled(float desired_X, float desired_Y, float desired_angle_deg, float current_X, float current_Y)
{
    return ((desired_Y - current_Y) * cos(toRad(desired_angle_deg)) <= -(desired_X - current_X) * sin(toRad(desired_angle_deg)));
}

void drive_with_volt(double l, double r)
{
    Left.spin(fwd, l, volt);
    Right.spin(fwd, r, volt);
}

int signum(double x)
{
    return (x > 0) - (x < 0);
}

double slew(double target_output, double prev_output, double slew)
{
    double diff = target_output - prev_output;
    if (fabs(diff) > slew)
    {
        return prev_output + signum(diff) * slew;
    }
    else
    {
        return target_output;
    }
}
double getInertialHeading() {
    return imu.rotation(deg);
}
double normalizeTarget(double angle) {
  // Adjust angle to be within +/-180 degrees of the inertial sensor's rotation
  if (angle > 180) {
    while (angle > 180) angle -= 360;
  } else if (angle < -180) {
    while (angle < -180) angle += 360;
  }
  return angle;
}

double driveChassis(double leftDrive, double rightDrive) {
    Left.spin(fwd, leftDrive, volt);
    Right.spin(fwd, rightDrive, volt);
    return 1;
}
