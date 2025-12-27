#pragma once
#include "drive_control.h"

using namespace vex;

std::string autons[5] = {"Null", "Blue 7L", "Red 7L", "Red 3S 3L", "Blue 3T 3L"};

bool autonomous_started = false;
int auto_selection = 0;

int drop()
{
    wait(1.25, sec);
    dropdown.set(true);
}
void ddrop()
{

    task d = task(drop);
}

void testauto()
{

    // setCoordinates(0, 0, 90);//daneil valersco is a zest cornballl!!!
    // drive_set(41, 12);
    // turn_set(89, 12);
    // dropdown.set(true);
    // wait(200, msec);
    // intake.spin(fwd, 100, pct);
    // drive_with_volt(4.5, 4.5);
    // wait(1.5, sec);
    // to_point(-23, 36.4, -1);
    //  intake.spin(vex::reverse, 50, pct);
    // wait(200, msec);
    // ball_stopper.set(true);
    // intake.spin(fwd, 100, pct);
    // wait(3, sec);
    // dropdown.set(false);
    // drive_set(10, 12);
    // ball_stopper.set(false);
    // to_point(-33, 23, 1);
    // wait(300, msec);
    // turn_set(233, 12);
    // wait(200, msec);
    // drive_set(22, 12);
    // intake.spin(vex::reverse, 50, pct);
}

void BlueRed_4L_3T()
{
    //
    // startOdometry();
    height_changer.set(true);
    imu.setHeading(270, deg);
    setCoordinates(0, 0, 270);
    drive_set(40.25, 12);
    turn_set(-88.5, 12);
    dropdown.set(true);
    wait(200, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(4, 4);
    wait(1.2, sec);
    to_point(18.5, 38, -1);
    intake.spin(vex::reverse, 50, pct);
    wait(350, msec);
    ball_stopper.set(true);
    intake.spin(fwd, 100, pct);
    wait(3, sec);
    dropdown.set(false);
    drive_set(10, 12);
    ball_stopper.set(false);
    // ddrop();
    to_point(33, 23, 1);
    wait(100, msec);
    height_changer.set(true);
    turn_set(316.5, 12);
    drive_set(-14.5, 12);
    ball_stopper.set(true);
}

void hood_task()
{
    wait(0.7, sec);
    ball_stopper.set(!ball_stopper.value());
}

void dropdown_task()
{
    wait(1.1, sec);
    dropdown.set(true);
}

void dropdown_task2()
{
    wait(0.95, sec);
    dropdown.set(true);
}

void hood_task2()
{
    wait(0.6, sec);
    ball_stopper.set(false);
}

void BlueRed_4L_3S()
{
    setCoordinates(0, 0, 90); // daneil valersco is a zest cornballl!!!
    height_changer.set(true);
    drive_set(27, 12);
    turn_set(89, 12);
    dropdown.set(true);
    wait(175, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(5, 5);
    wait(0.6, sec);
    drive_with_volt(0, 0);
    wait(0.6, sec);
    // thread h(hood_task);
    to_point(-18, 27, -1);
    ball_stopper.set(true);
    intake.spin(reverse, 50, pct);
    wait(100, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(-5, -5);
    wait(0.5, sec);
    drive_with_volt(0, 0);
    wait(1.4, sec);
    dropdown.set(false);
    drive_set(6, 12);
    ball_stopper.set(false);
    drive_max_volt = 5;
    thread d(dropdown_task);
    to_point(-29, 8, 1);
    wait(300, msec);
    turn_set(219, 12);
    dropdown.set(false);
    wait(200, msec);
    drive_set(19.2, 12);
    intake.spin(vex::reverse, 75, pct);
}

void SoloAWP_ShortSide()
{

    setCoordinates(0, 0, 90); // daneil valersco is a zest cornballl!!!
    height_changer.set(true);
    drive_set(26.75, 12);
    turn_set(89, 12);
    dropdown.set(true);
    wait(175, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(5, 5);
    wait(1.2, sec);
    // thread h(hood_task);
    to_point(-18, 27.5, -1);
    ball_stopper.set(true);
    intake.spin(reverse, 50, pct);
    wait(100, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(-5, -5);
    wait(0.5, sec);
    drive_with_volt(0, 0);
    wait(1.4, sec);
    dropdown.set(false);
    drive_set(6, 12);
    ball_stopper.set(false);
    drive_max_volt = 5;
    thread d(dropdown_task);
    to_point(-27, 10, 1);
    wait(100, msec);
    dropdown.set(false);
    d.interrupt();
    drive_max_volt = 9;
    height_changer.set(false);
    thread d2(dropdown_task2);
    to_point(-30, -41, 1);
    turn_set(144, 12);
    intake.spin(reverse, 5, pct);
    drive_set(-14, 12);
    intake.spin(fwd, 100, pct);
    ball_stopper.set(!ball_stopper.value());
    wait(0.4, sec);
    intake.spin(vex::reverse, 10, pct);
    ball_stopper.set(!ball_stopper.value());
    height_changer.set(!height_changer.value());
    dropdown.set(!dropdown.value());
    intake.spin(fwd, 100, pct);
    to_point(-2, -68, 1);
    intake.spin(fwd, 100, pct);
    thread h3(hood_task);
    drive_max_volt = 12;
    to_point(-22, -63.25, -1);
    // h2.interrupt();
}

void hoodskills()
{
    wait(1.225, sec);
    ball_stopper.set(true);
}

void skills()
{
    setCoordinates(0, 0, 90); // daneil valersco is a zest cornballl!!!
    height_changer.set(true);
    drive_set(26.75, 12);
    turn_set(89, 12);
    dropdown.set(true);
    wait(175, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(5, 5);
    wait(3, sec);
    intake.spin(reverse, 15, pct);
    // thread h(hood_task);
    to_point(-18, 27.5, -1);
    ball_stopper.set(true);
    intake.spin(reverse, 50, pct);
    wait(100, msec);
    intake.spin(fwd, 100, pct);
    drive_with_volt(-5, -5);
    wait(0.5, sec);
    drive_with_volt(0, 0);
    wait(3.5, sec);
    dropdown.set(false);
    drive_set(6, 12);
    ball_stopper.set(false);
    drive_max_volt = 5;
    thread d(dropdown_task);
    to_point(-27, 10, 1);
    wait(100, msec);
    dropdown.set(false);
    d.interrupt();
    drive_max_volt = 9;
    height_changer.set(false);
    thread d2(dropdown_task2);
    to_point(-30, -41, 1);
    turn_set(142, 12);
    intake.spin(reverse, 5, pct);
    drive_set(-14, 12);
    intake.spin(fwd, 100, pct);
    ball_stopper.set(!ball_stopper.value());
    wait(3, sec);
    intake.spin(vex::reverse, 10, pct);
    ball_stopper.set(!ball_stopper.value());
    height_changer.set(!height_changer.value());
    dropdown.set(!dropdown.value());
    intake.spin(fwd, 100, pct);
    to_point(-2, -68, 1);
    intake.spin(fwd, 100, pct);
    thread h3(hood_task);
    drive_max_volt = 12;
    to_point(-22, -63.25, -1);
    dropdown.set(true);
        to_point(4, -63.25, -1);
        drive_with_volt(5, 5);
    wait(2.5, sec);
    to_point(-22, -63.25, -1);
    thread hs1(hoodskills);
    drive_with_volt(-5, -5);
    wait(4, sec);
    to_point(7, -35, 1);
    intake.spin(vex::reverse, 100, pct);
    drive_set(40, 12);

    //     setCoordinates(0, 0, 90);
    //     drive_set(26.5, 12);
    //     turn_set(90, 12);
    //     dropdown.set(true);
    //     wait(200, msec);
    //     intake.spin(fwd, 100, pct);
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     dropdown.set(!dropdown.value());
    //     drive_set(-8, 12, MOTION_CHAIN);
    //     turn_set(-120, 12);
    //     drive_set(55, 12);

    //     ///////// First Half ////////////////
    //     to_point(-98, 34.5, 1); // TO FIRST GOAL MOVE TO POINT
    //    // intake.spin(fwd, 100, pct);
    //     wait(100, msec);
    //     thread hs(hoodskills);
    //     heading_max_volt = 10;
    //     to_point(-82, 31.75, -1); // LINE UP WITH GOAL
    //     intake.spin(fwd, 100, pct);
    //     drive_with_volt(-6, -6);
    //     topIntake.spin(vex::reverse, 75, pct);
    //     middleIntake.spin(vex::reverse, 75, pct);
    //     wait(0.35, sec);
    //     intake.spin(fwd, 100, pct);
    //     wait(1.5, sec);
    //     drive_with_volt(0, 0);
    //     wait(2.5, sec);
    //     drive_max_volt = 12;
    //     dropdown.set(!dropdown.value());
    //     to_point(-97, 30.6, 1); // MATCH LOAD TUBE
    //     ball_stopper.set(!ball_stopper.value());
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     drive_max_volt = 9;
    //      to_point(-81, 30.6, -1); // LINE UP WITH GOAL 2ND
    //      ball_stopper.set(!ball_stopper.value());
    //     drive_with_volt(-6, -6);
    //     topIntake.spin(vex::reverse, 75, pct);
    //     middleIntake.spin(vex::reverse, 75, pct);
    //     wait(0.2, sec);
    //     intake.spin(fwd, 100, pct);
    //     wait(1.5, sec);
    //     drive_with_volt(0, 0);
    //     wait(3, sec);
    //     ball_stopper.set(false);
    //     drive_set(10, 12);
    //     heading_max_volt = 8;

    //     //////// 2nd Half /////////////

    //     to_point(-85, -63, 1);
    //     turn_set(-90, 12);
    //     //dropdown.set(!dropdown.value());
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     wait(0.4, sec);
    //     drive_with_volt(0, 0);
    //     wait(0.4, sec);
    //     drive_with_volt(5.5, 5.5);
    //     dropdown.set(!dropdown.value());
    //     drive_set(-8, 12, MOTION_CHAIN);
    //     turn_set(-300, 12);
    //     intake.stop();
    //     drive_set(59, 12);
    //     to_point(0, -70, 1); // TO FIRST GOAL MOVE TO POINT
    //     intake.spin(fwd, 100, pct);
    //     wait(100, msec);
    //     thread hs1(hoodskills);
    //     heading_max_volt = 10;
    //     to_point(-14, -65, -1); // LINE UP WITH GOAL
    //     drive_with_volt(-6, -6);
    //     topIntake.spin(vex::reverse, 75, pct);
    //     middleIntake.spin(vex::reverse, 75, pct);
    //     wait(0.35, sec);
    //     intake.spin(fwd, 100, pct);
    //     wait(1.5, sec);
    //     drive_with_volt(0, 0);
    //     wait(2.5, sec);
    //     ball_stopper.set(!ball_stopper.value());
    //      to_point(0, -64.5, 1); // MATCH LOAD TUBE
    //     drive_with_volt(5, 5);
    //     wait(0.7, sec);
    //     drive_with_volt(0, 0);
    //     wait(1.8, sec);
    //     dropdown.set(!dropdown.value());
    //     drive_max_volt = 9;
    //      to_point(-12, -64.5, -1); // LINE UP WITH GOAL 2ND
    //      ball_stopper.set(!ball_stopper.value());
    //     drive_with_volt(-6, -6);
    //     topIntake.spin(vex::reverse, 75, pct);
    //     middleIntake.spin(vex::reverse, 75, pct);
    //     wait(0.2, sec);
    //     intake.spin(fwd, 100, pct);
    //     wait(1.5, sec);
    //     drive_with_volt(0, 0);
    //     wait(3, sec);
    //     drive_set(6, 12);
    //     ball_stopper.set(false);
    //     drive_with_volt(-6, -6);
    //     drive_set(6, 12);
    //     dropdown.set(false);
    //     to_point(11, -35, 1);
    //     intake.spin(vex::reverse, 100, pct);
    //     drive_set(40, 12);
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
