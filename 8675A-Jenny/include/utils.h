
#include "vex.h"
#pragma once

extern double wrap_rad(double input);
double reduce_negative_180_to_180(double angle);
double reduce_negative_90_to_90(double angle);
double reduce_0_to_360(double angle);
double get_absolute_heading();
double left_voltage_scaling(double drive_output, double heading_output);
double right_voltage_scaling(double drive_output, double heading_output);
double clamp_min_voltage(double drive_output, double heading_output);
double toDeg(double input);
double toRad(double input);
bool is_line_settled(float desired_X, float desired_Y, float desired_angle_deg, float current_X, float current_Y);
void drive_with_volt(double l, double r);
int signum(double x);
double slew(double target_output, double prev_output, double slew);
double normalizeTarget(double angle);
double getInertialHeading();

double driveChassis(double leftDrive, double rightDrive);
