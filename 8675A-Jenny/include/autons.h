#include "vex.h"
#include "colorsort.h"
using namespace vex;

std::string autons[5] = {"Null", "Blue 7L", "Red 7L", "Red 3S 3L", "Blue 3T 3L"};
bool autonomous_started = false;
int auto_selection = 0;

void testauto()
{
    //
}

void blue_7L()
{
    //
}

void red_7L()
{
    //
}

void red_3S_3L()
{
    //
}

void blue_3T_3L()
{
    //
}

// Piston toggles
void ballstop_toggle()
{
    ball_stopper.set(!ball_stopper.value());
    wait(100, msec);
}

void unloader_toggle()
{
    unloader.set(!unloader.value());
    wait(100, msec);
}

void height_toggle()
{
    height_changer.set(!height_changer.value());
    wait(100, msec);
}

void sorter_toggle()
{
    piston_sorter.set(!piston_sorter.value());
    wait(100, msec);
}
