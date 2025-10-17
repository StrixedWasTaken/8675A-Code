#include "vex.h"

extern double to_inch(double input);

// degrees to radians
extern double toRad(double input);

 extern double x;                  // global x calculated using carestian and polar coordinate systems
 extern double y;                  // global Y calculated using carestian and polar coordinate systems
 extern double raw_vertical;           // vertical wheel degrees (in inches)
 extern double raw_horizontal;         // horizontal wheel degrees (in inches)
 extern double previous_vertical;      // previous vertical wheel inches
 extern double previous_horizontal;    // previous horizontal wheel inches
 extern double deltaVertical;          // difference between vertical inches and its previous
 extern double deltaHorizontal;        // difference between horizontal inches and its previous
 extern double heading;                // heading of the robot
 extern double previous_heading;       // previous value of the heading of the robot
 extern double localX ;             // local carestian coordinates of X value
 extern double localY;             // local carestian coordinates of Y value
 extern double deltaHeading;           // change in heading
 extern double polarRadius;            // polar coordinate of the distance using carestian x and y coordinates
 extern double polarAngle;             // polar coordinate that uses carestian x and y coordinates to determine the heading of the robot
 extern double globalPolarAngle;       // heading calculation for arc-based movements
 extern double vertical_offset;        // how far vertical wheel is from the center
 extern double horizontal_offset;      // how far horizontal wheel is from the center
 extern bool odometry_enabled; // enables/disables odometry tracking

extern int odometry_loop();

extern void startOdometry();

