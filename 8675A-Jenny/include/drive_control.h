#include "vex.h"
#include "odom.h"
#include "utils.h"

#pragma once

// Enum for swing direction
enum class SwingSet {
    LEFT_SWING,
    RIGHT_SWING
};

// Enum for motion types
enum class SettleType {
    DEFAULT_WAIT = 15,
    QUICK_WAIT = 10,
    MOTION_CHAIN = 0
};

constexpr SettleType DEFAULT_WAIT = SettleType::DEFAULT_WAIT;
constexpr SettleType QUICK_WAIT = SettleType::QUICK_WAIT;
constexpr SettleType MOTION_CHAIN = SettleType::MOTION_CHAIN;

constexpr SwingSet LEFT_SWING = SwingSet::LEFT_SWING;
constexpr SwingSet RIGHT_SWING = SwingSet::RIGHT_SWING;

// Function declarations without default for settleType
extern void drive_set(double target, double mSpeed, SettleType motion = SettleType::DEFAULT_WAIT);

extern void turn_set(double target, double mSpeed, SettleType motion = SettleType::DEFAULT_WAIT);

extern void left_swing(double target, double mSpeed, double arcSpeed = 0, SettleType motion = SettleType::DEFAULT_WAIT);

extern void right_swing(double target, double mSpeed, double arcSpeed = 0, SettleType motion = SettleType::DEFAULT_WAIT);

extern void to_point(double xTarget, double yTarget, int dir, SettleType motion = SettleType::DEFAULT_WAIT);

extern void turn_to(double xTarget, double yTarget, SettleType motion = SettleType::DEFAULT_WAIT);

extern void to_pose(double xTarget, double yTarget, double angle, double dlead, SettleType motion = SettleType::DEFAULT_WAIT);

extern double compute(double input); // PID Compute function

extern bool pidSettled(SettleType motion, int time_settled, int timeout, int max_timeout);

// PID constants for various motions
extern double driveKP, driveKI, driveKD, turnKP, turnKI, turnKD, swingKP, swingKI, swingKD, lKP, lKI, lKD, aKP, aKI, aKD;

extern double xError, yError, angularError, linearError, linearOutput, angularOutput, leftPower, rightPower;

extern double targetAngle;

extern double heading_max_volt, drive_max_volt, dlead, drive_min_volt, targetDistance, carrot_X, carrot_Y, scale_factor, setback;

// wraps intertial error so it finds the fastest way to turn to a point
extern double wrap(double input);

//Wraps heading between pi and -pi for radian heading
extern double wrapRad(double input);

// PID settle check based on the settle type, without a default for SettleType
extern bool pidSettled(SettleType st = SettleType::DEFAULT_WAIT); 

// radians to degrees
extern double toDeg(double input);

//Clamp logic from C++ 17 just copied over since it doesn't exist in this version

template<class T>
const T& clamp(const T& v, const T& lo, const T& hi)
{
    return (v < lo) ? lo : (hi < v) ? hi : v;
}
