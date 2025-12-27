#include "odom.h"
#include "utils.h"

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
 double vertical_offset = 0.0;        // how far vertical wheel is from the center
 double horizontal_offset = 0.0;      // how far horizontal wheel is from the center
 double sin_multiplier; // multiplier fpor calculating local coordinates
 double vertical_tracking_diameter = 2.0;
 double horizontal_tracking_diameter = 2.0;
bool odometry_enabled = false; // enables/disables odometry tracking

int odometry_loop()
{
    while (true)
    {
         raw_vertical = vertical_wheel.position(deg) * vertical_tracking_diameter * M_PI / 360;
         raw_horizontal = horizontal_wheel.position(deg) * horizontal_tracking_diameter * M_PI / 360;
         heading = toRad(get_absolute_heading());
         deltaVertical = raw_vertical - previous_vertical;
         deltaHorizontal = raw_horizontal - previous_horizontal;
         deltaHeading = heading - previous_heading;

        cout << "heading: " << get_absolute_heading() << endl;

        if (fabs(deltaHeading) < 1e-6)
        {
             localX = deltaHorizontal;
             localY = deltaVertical;
        }
        else
        {
            sin_multiplier = 2 * sin(deltaHeading / 2);
            
            localX =  sin_multiplier * ((deltaHorizontal / deltaHeading) + horizontal_offset);
            localY = sin_multiplier * ((deltaVertical / deltaHeading) + vertical_offset);
        }

        if (fabs(localX) < 1e-6 && fabs(localY) < 1e-6)
        {
            polarAngle = 0;
        }
        else
        {
            polarAngle = atan2(localY, localX);
        }
        
         polarRadius = sqrt(pow(localX, 2) + powf(localY, 2));        
         globalPolarAngle = polarAngle - previous_heading - (deltaHeading / 2);

         x += polarRadius * cos(globalPolarAngle);
         y += polarRadius * sin(globalPolarAngle);

        // Brain.Screen.printAt(10, 100, "X: %2f", x);
        // Brain.Screen.printAt(10, 110, "Y: %2f", y);
        // Brain.Screen.printAt(10, 120, "Heading: %2f", get_absolute_heading());
        previous_vertical = raw_vertical;
        previous_horizontal = raw_horizontal;
        previous_heading = heading;
        wait(10, msec);
    }
    return 0;
}

void setCoordinates(double startX, double startY, double startHeadingDeg) {
    // Set odometry position
    x = startX;
    y = startY;

    // Set robot heading
    heading = startHeadingDeg;   // for odometry math
    imu.setHeading(startHeadingDeg, degrees); // optional: sync with sensor

    // Reset velocities or previous errors if your odometry uses them
    previous_horizontal = startX;
    previous_vertical = startY;
    previous_heading = startHeadingDeg;

    deltaHorizontal = 0;
    deltaVertical = 0;
}
void startOdometry() {
    while(true) {
        odometry_enabled = true;
        task o(odometry_loop);
    }

}
