#include "vex.h"
#include "drive_control.h"

using namespace vex;

std::string autons[5] = {"Null", "Blue 7L", "Red 7L", "Red 3S 3L", "Blue 3T 3L"};

bool autonomous_started = false;
int auto_selection = 0;

int drop() {
    wait(1.25, sec);
    dropdown.set(true);
}
void ddrop() {

    task d = task(drop);

}

void testauto()
{
 
    setCoordinates(0, 0, 90);//daneil valersco is a zest cornballl!!!
    drive_set(41, 12);
    turn_set(89, 12);
    dropdown.set(true);
    wait(200, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(4.5, 4.5);
    wait(1.5, sec);
    to_point(-23, 36.4, -1);
     intake.spin(reverse, 50, pct);
    wait(200, msec);
    ball_stopper.set(true);
    intake.spin(fwd, 100, pct);
    wait(3, sec);
    dropdown.set(false);
    drive_set(10, 12);
    ball_stopper.set(false);
    to_point(-33, 23, 1);
    wait(300, msec);
    turn_set(233, 12);
    wait(200, msec);
    drive_set(22, 12);
    intake.spin(reverse, 50, pct);

}

void BlueRed_4L_3T()
{
    //
     // startOdometry();
    imu.setHeading(270, deg);
    setCoordinates(0, 0, 270);
    drive_set(40.25, 12);
    turn_set(-88.5, 12);
    dropdown.set(true);
    wait(200, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(4, 4);
    wait(1.45, sec);
    to_point(18.5, 38, -1);
    intake.spin(reverse, 50, pct);
    wait(350, msec);
    ball_stopper.set(true);
    intake.spin(fwd, 100, pct);
    wait(3, sec);
    dropdown.set(false);
    drive_set(10, 12);
    ball_stopper.set(false);
    //ddrop();
    to_point(33, 23, 1);
    wait(100, msec);
    height_changer.set(true);
    turn_set(316.5, 12);
    drive_set(-14.5, 12);
    ball_stopper.set(true);

}

void hood_task() {
    wait(0.6, sec);
    ball_stopper.set(true);
}

void dropdown_task() {
    wait(1.1, sec);
    dropdown.set(true);
}

void dropdown_task2() {
    wait(0.95, sec);
    dropdown.set(true);
}

void hood_task2() {
    wait(0.6, sec);
    ball_stopper.set(true);
}


void BlueRed_4L_3S()
{
    setCoordinates(0, 0, 90); //daneil valersco is a zest cornballl!!!
    drive_set(30, 12);
    turn_set(89, 12);
    dropdown.set(true);
    wait(175, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(4.5, 4.5);
    wait(1.1, sec);
    thread h(hood_task);
    to_point(-19.5, 27, -1);
    wait(2, sec);
    dropdown.set(false);
    drive_set(6, 12);
    drive_with_volt(-8, -8);
    wait(0.75, sec);
    drive_set(6, 12);
    ball_stopper.set(false);
    drive_max_volt = 5;
    thread d(dropdown_task);
    to_point(-29, 7.75, 1);
    wait(300, msec);
    turn_set(223, 12);
    dropdown.set(false);
    wait(200, msec);
    drive_set(19, 12);
    intake.spin(reverse, 50, pct);
}

void SoloAWP_ShortSide() {
 setCoordinates(0, 0, 90); //daneil valersco is a zest cornballl!!!
 height_changer.set(true);
    drive_set(31, 12);
    turn_set(89, 12);
    dropdown.set(true);
    wait(175, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(5.5, 5.5);
    wait(1, sec);
    thread h(hood_task);
    to_point(-21.5, 28, -1);
    wait(1.35, sec);
    h.interrupt();
    dropdown.set(false);
    ball_stopper.set(false);
    drive_set(6, 12);
    drive_max_volt = 5;
    thread d(dropdown_task);
    to_point(-30, 9.75, 1);
    wait(100, msec);
    dropdown.set(false);
    d.interrupt();
    drive_max_volt = 9;
    height_changer.set(false);
    thread d2(dropdown_task2);
    to_point(-29, -40.5, 1);
    turn_set(142, 12);
    thread h2(hood_task2);
    drive_set(-16.4, 12);
    wait(0.6, sec);
    intake.spin(reverse, 10, pct);
    ball_stopper.set(!ball_stopper.value());
    height_changer.set(!height_changer.value());
    dropdown.set(!dropdown.value());
    intake.spin(fwd, 100, pct);
    to_point(-2, -64, 1);
    intake.spin(fwd, 100, pct);
    thread h3(hood_task);
to_point(-22, -60.5, -1);
    //h2.interrupt();
    
}

void hoodskills() {
    wait(1.15, sec);
     ball_stopper.set(true);
}

void skills() {
    setCoordinates(0, 0, 90);
    height_changer.set(true);
    drive_set(31, 12);
    turn_set(90, 12);
    dropdown.set(true);
    wait(200, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(5.5, 5.5);
    wait(2.2, sec);
    drive_set(-8, 12, MOTION_CHAIN);
    turn_set(-120, 12);
    intake.stop();
    drive_set(55, 12);
    //dropdown.set(false);
    to_point(-105, 40, 1);
    intake.spin(fwd, 100, pct);
    wait(100, msec);
    thread hs(hoodskills);
    to_point(-79, 32.5, -1);
    drive_with_volt(-3, -3);
    wait(1.5, sec);
    drive_with_volt(0, 0);
    wait(2.5, sec);
    ball_stopper.set(!ball_stopper.value());
    to_point(-110, 32.5, 1);
    wait(2.2, sec);
     to_point(-79, 32.5, -1);
     ball_stopper.set(!ball_stopper.value());
    drive_with_volt(-3, -3);
    wait(1.5, sec);
    drive_with_volt(0, 0);
    
}

// Piston toggles
void ballstop_toggle()
{
    ball_stopper.set(!ball_stopper.value());
    wait(100, msec);
}

void dropdown_toggle()
{
    dropdown.set(!dropdown.value());
    wait(100, msec);
}

void height_toggle()
{
    height_changer.set(!height_changer.value());
    wait(100, msec);
}

