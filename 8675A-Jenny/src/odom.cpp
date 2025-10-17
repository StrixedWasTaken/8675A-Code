#include "odom.h"

double to_inch(double input)
{
    return (input / 360) * (1.0 * (M_PI * 2.0));
}

// degrees to radians
double toRad(double input)
{
    return input * (M_PI / 180);
}

 double x;                  // global x calculated using carestian and polar coordinate systems
 double y;                  // global Y calculated using carestian and polar coordinate systems
 double raw_vertical;           // vertical wheel degrees (in inches)
 double raw_horizontal;         // horizontal wheel degrees (in inches)
 double previous_vertical;      // previous vertical wheel inches
 double previous_horizontal;    // previous horizontal wheel inches
 double deltaVertical;          // difference between vertical inches and its previous
 double deltaHorizontal;        // difference between horizontal inches and its previous
 double heading;                // heading of the robot
 double previous_heading;       // previous value of the heading of the robot
 double localX ;             // local carestian coordinates of X value
 double localY;             // local carestian coordinates of Y value
 double deltaHeading;           // change in heading
 double polarRadius;            // polar coordinate of the distance using carestian x and y coordinates
 double polarAngle;             // polar coordinate that uses carestian x and y coordinates to determine the heading of the robot
 double globalPolarAngle;       // heading calculation for arc-based movements
 double vertical_offset = 5.0;        // how far vertical wheel is from the center
 double horizontal_offset = -2.0;      // how far horizontal wheel is from the center
bool odometry_enabled = false; // enables/disables odometry tracking

int odometry_loop()
{

    while (true)
    {
         raw_vertical = to_inch(vertical_wheel.position(deg));
         raw_horizontal = to_inch(horizontal_wheel.position(deg));
         heading = toRad(imu.heading(deg));
         deltaVertical = raw_vertical - previous_vertical;
         deltaHorizontal = raw_horizontal - previous_horizontal;
         deltaHeading = heading - previous_heading;

        if (deltaHeading == 0)
        {
             localX = deltaHorizontal;
             localY = deltaVertical;
        }
        else
        {
            localX = ((2 * sin(deltaHeading / 2)) * ((deltaHorizontal / deltaHeading)));
            localY = ((2 * sin(deltaHeading / 2)) * ((deltaVertical / deltaHeading)));
        }

        if (localX == 0 && localY == 0)
        {
            polarRadius = 0;
            polarAngle = 0;
        }
        else
        {
            polarAngle = atan2(localY, localX);
             polarRadius = sqrt(pow(localX, 2) + powf(localY, 2));
        }
         globalPolarAngle = polarAngle - previous_heading - (deltaHeading / 2);

         x += polarRadius * cos(globalPolarAngle);
         y += polarRadius * sin(globalPolarAngle);

        Brain.Screen.printAt(10, 40, "X: %2f", x);
        Brain.Screen.printAt(10, 50, "Y: %2f", y);

        previous_vertical = raw_vertical;
        previous_horizontal = raw_horizontal;
        previous_heading = heading;
        wait(10, msec);
    }
    return 0;
}

void startOdometry() {
    while(true) {
        odometry_enabled = true;
        task o(odometry_loop);
    }
}