#include "vex.h"

using namespace vex;

bool redActivate = false;

bool blueActivate = false;

int blueSort()
{
    color_sensor.setLight(ledState::on);
    color_sensor.setLightPower(25, pct);
    while (true)
    {
        if (blueActivate)
        {
            intake.spin(fwd, 100, pct);
            double hue = color_sensor.hue();

            if (hue <= 230)
            {
                piston_sorter.set(true);
                wait(500, msec);
            }
            else if (hue > 230)
            {
                piston_sorter.set(false);
            }
        }
        else
        {
            intake.stop();
            break;
        }

        wait(10, msec);
    }
    return 1;
}

int redSort()
{
    color_sensor.setLight(ledState::on);
    color_sensor.setLightPower(0, pct);
    while (true)
    {
        if (redActivate)
        {
            double hue = color_sensor.hue();
            intake.spin(fwd, 100, pct);
            if (hue >= 0 && hue <= 70)
            {
                piston_sorter.set(true);
                wait(300, msec);
            }
            else if (hue > 70)
            {
                piston_sorter.set(false);
            }
        }
        else
        {
            intake.stop();
            break;
        }

        wait(10, msec);
    }
    return 1;
}