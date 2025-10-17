#include "vex.h"
#include "autons.h"

using namespace vex;

std::string motornames[8] = {"l1", "l2", "l3", "r1", "r2", "r3", "left_in", "right_in"};
std::string truciatedautos[7] = {"Null", "B7L", "R7L", "R3S3L", "B3T3L", "NA", "NA"};
int amount_calibrated = 1;

int printinfo()
{
    while (true)
    {

        int LDT = ((l1.temperature(celsius) + l2.temperature(celsius) + l3.temperature(celsius)) / 3);
        int RDT = ((r1.temperature(celsius) + r2.temperature(celsius) + r3.temperature(celsius)) / 3);
        int IT = (leftIntake.temperature(celsius));
        int I2T = (rightIntake.temperature(celsius));

        int Braintime = (Brain.timer(sec));
        Controller1.Screen.setCursor(1, 1);

        for (int i = 0; i < 8; i++)
        {
            if ((all_motors[i].installed())==false)
            {
                Controller1.Screen.print(" %s", motornames[i].c_str());
            }
        }
        if (!imu.installed())
        {
            Controller1.Screen.print("Inertial");
        }
        if (!color_sensor.installed())
        {
            Controller1.Screen.print("No Color Sensor");
        }

        // if (!horizontal_wheel.installed())
        // {
        //     Controller1.Screen.print("onodom");
        // }
        
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("LD%d", LDT);
        Controller1.Screen.print(" RD%d", RDT);
        Controller1.Screen.print(" I%d", IT);

        Controller1.Screen.setCursor(3, 2);
        Controller1.Screen.print( "%d %", Brain.Battery.capacity(percent));
        Controller1.Screen.print(" %d", Braintime);
        Controller1.Screen.print("A: %d", auto_selection);
        Controller1.Screen.print(" %s", truciatedautos[auto_selection].c_str());
        Controller1.Screen.print(" %d", amount_calibrated);
        if (imu.isCalibrating())
        {

            Controller1.Screen.print(" B");
        }
        else
        {
            Controller1.Screen.print(" G");
        }
        vex::task::sleep(1000);
        Controller1.Screen.clearScreen();
    }
}

