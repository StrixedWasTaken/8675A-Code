#include "autons.h"
#include <deque>
#include <vector>
#include <algorithm>
#pragma once

const double FromRad=180/M_PI;
int Sx;
int Sy;
int BrainScreen_Mode=0;
int Previous_Brain_Screen;
bool BrainScreen_Latch=false;
bool BrainScreen_Switch=false;
bool BrainScreen_Print=true;
double Drive_KIslider=0.5;
double Drive_KPslider=0.5;
double Drive_KDslider=0.5;
double Turn_KIslider=0.5;
double Turn_KPslider=0.5;
double Turn_KDslider=0.5;
double Slider_Drive_Distance=0.5;
double Circle_Turn_Angle=0;
double Time_Ran=0;



deque<double> KPreadings(1000);
deque<double> KIreadings(1000);
deque<double> KDreadings(1000);

double Test_driveKP = 0.5;
double Test_driveKI = 0.0;
double Test_driveKD = 1.7;

double Test_turnKP = 0.4;
double Test_turnKI = 0.01;
double Test_turnKD = 2.2;
int timeoutt = 0;

// PID Drives
void drive_test()
{

  double time_settled = 0;
  timeoutt=0;
  double error = 0;
  double integral = 0;
  double derivative = 0;
  l2.resetPosition();
  r2.resetPosition();

  PID drivePID(Test_driveKP, Test_driveKI, Test_driveKD);
  drivePID.setIntegralLimits(10, 1000);
  drivePID.set_constants(2.5, 100);

  while (!pidSettled(SettleType::DEFAULT_WAIT, time_settled, timeoutt, drivePID.max_timeout))
  {

    double wheel_deg = ((l2.position(deg) + r2.position(deg)) / 2) * wheel_inches;

    error = (Slider_Drive_Distance-.5)*48 - wheel_deg;

    double output = drivePID.update(error);

    output = clamp<double>(output, -12, 12); // limits speed based off max speed

    KPreadings.emplace_back(drivePID.P_Result);
    KIreadings.emplace_back(drivePID.I_Result);
    KDreadings.emplace_back(drivePID.D_Result);


    Left.spin(fwd, output, volt);
    Right.spin(fwd, output, volt);

   

    double previous_error = error;

    timeoutt++;

    if (fabs(error) < drivePID.settle_error )
    {
      time_settled++;
     
    }
    
    else
    {
      time_settled = 0;
    }

    wait(10, msec);
    Time_Ran++;
  }
    Left.stop(brake);
    Right.stop(brake);
}

// PID Swings
void turn_test()
{

  double time_settled = 0;
   timeoutt = 0;
  double error = 0;
  double integral = 0;
  double derivative = 0;
  PID turnPID(Test_turnKP, Test_turnKI, Test_turnKD);
  turnPID.setIntegralLimits(6, 1000);
  turnPID.set_constants(2, 200);

  while (!pidSettled(SettleType::DEFAULT_WAIT, time_settled, timeoutt, turnPID.max_timeout))
  {

    error = reduce_negative_180_to_180(Circle_Turn_Angle*FromRad - get_absolute_heading());

    double output = turnPID.update(error);

    output = clamp(output, -10.0, 10.0);

    Brain.Screen.printAt(10, 40, "Turn Error: %2f", error);
    
    KPreadings.emplace_front(turnPID.P_Result);
    KIreadings.emplace_front(turnPID.I_Result);
    KDreadings.emplace_front(turnPID.D_Result);
    KPreadings.pop_back();
    KIreadings.pop_back();
    KDreadings.pop_back();
    
    cout << "turn error: " << error << endl;
    Left.spin(fwd, output, volt);
    Right.spin(vex::directionType::rev, output, volt);

    timeoutt++;

    if ( fabs(error) < turnPID.settle_error)
    {
      time_settled++;
    }
    else
    {
      time_settled = 0; // Reset if error jumps again
    }

    wait(10, msec);
    Time_Ran++;
  }

 
    Left.stop(brake);
    Right.stop(brake);
 
}



std::string Route_Descriptions[6][3] = {
 {"YOU","SHOULD NOT", "BE HERE"},
 {"Scores 4 in the Long Goal","And 3 in the Short Central Goal", "Starts Facing towards Left Long Goal"},
 {"Scores 4 in the Long Goal","And 3 in the Tall Central Goal", "Starts Facing towards Right Long Goal"},
 {"Scores 5 in both Long goals", "and 1 in the Tall Central Goal", "Starts Facing Towards Right"},
 {"Long Skills Run", "Averages About 60 Points", "Scores All Over The Field"},
 {"","",""}

};

std::string Route_Names[6]= {"Null","4 Long 3 Tall", "4 Long 3 Short ", "SS Solo AWP", "Skills","None"};
std::string Autonames[5] {
  " null",
  " 4L3T",
  " 4L3S",
  " Solo AWP",
  " Skills"
 };
void makeintbutton(int xpos1, int ypos1, int xpos2, int ypos2, int &affectedvariable, int desired_value, string color, string button_message, string textcolor) {
if (BrainScreen_Print) {
    Brain.Screen.drawRectangle(xpos1, ypos1, xpos2-xpos1, ypos2-ypos1, color.c_str());
 Brain.Screen.setPenColor(textcolor.c_str());
    Brain.Screen.printAt((xpos1+xpos2-Brain.Screen.getStringWidth(button_message.c_str()))/2,(ypos1+ypos2+Brain.Screen.getStringHeight(button_message.c_str())*.5)*.5,false,button_message.c_str());
    Brain.Screen.setPenColor("#FFFFFF");} 
if (BrainScreen_Switch) {
if (Sx>xpos1 && Sx<xpos2 && Sy>ypos1 && Sy<ypos2) {
affectedvariable=desired_value;}

}
}
void autobutton(int xpos1, int ypos1, int xpos2, int ypos2, int desired_value, string color, string button_message, string textcolor) {
    if (BrainScreen_Print) {
        
        
        Brain.Screen.drawRectangle(xpos1, ypos1, xpos2-xpos1, ypos2-ypos1, color.c_str());
     Brain.Screen.setPenColor(textcolor.c_str());
        Brain.Screen.printAt((xpos1+xpos2-Brain.Screen.getStringWidth(button_message.c_str()))/2,(ypos1+ypos2+Brain.Screen.getStringHeight(button_message.c_str())*.5)*.5,false,button_message.c_str());
        Brain.Screen.setPenColor("#FFFFFF");
    } 
    if (BrainScreen_Switch) {
    if (Sx>xpos1 && Sx<xpos2 && Sy>ypos1 && Sy<ypos2) {
    auto_selection=desired_value;
    Previous_Brain_Screen=BrainScreen_Mode;
    BrainScreen_Mode=10;
    }
    
    }
    }

typedef int (*FunctionPointer)();

void runintbutton(int xpos1, int ypos1, int xpos2, int ypos2, FunctionPointer runprogram, string color, string button_message, string textcolor) {
    if (BrainScreen_Print) {
        
        
        Brain.Screen.drawRectangle(xpos1, ypos1, xpos2-xpos1, ypos2-ypos1, color.c_str());
     Brain.Screen.setPenColor(textcolor.c_str());
        Brain.Screen.printAt((xpos1+xpos2-Brain.Screen.getStringWidth(button_message.c_str()))/2,(ypos1+ypos2+Brain.Screen.getStringHeight(button_message.c_str())*0.5)*.5,false,button_message.c_str());
        Brain.Screen.setPenColor("#FFFFFF");
    } 
    if (BrainScreen_Switch) {
    if (Sx>xpos1 && Sx<xpos2 && Sy>ypos1 && Sy<ypos2) {
    runprogram();
    }
    
    }
    }
    

void makeboolbutton(int xpos1, int ypos1, int xpos2, int ypos2, bool &affectedvariable, bool desired_value, string color, string button_message, string textcolor) {
    if (BrainScreen_Print) {
        
        
        Brain.Screen.drawRectangle(xpos1, ypos1, xpos2-xpos1, ypos2-ypos1, color.c_str());
     Brain.Screen.setPenColor(textcolor.c_str());
        Brain.Screen.printAt((xpos1+xpos2-Brain.Screen.getStringWidth(button_message.c_str()))/2,(ypos1+ypos2+Brain.Screen.getStringHeight(button_message.c_str()))/2,false,button_message.c_str());
        Brain.Screen.setPenColor("#FFFFFF");
    } 
    if (BrainScreen_Switch) {
    if (Sx>xpos1 && Sx<xpos2 && Sy>ypos1 && Sy<ypos2) {
    affectedvariable=desired_value;
    }
    
    }
    }

int settoposition() {
    Brain.programStop();
    return 0;
}

void rotatePoint(float cx, float cy, float &x, float &y, float angle) {
    float x_new = cx + (x - cx) * cos(angle) - (y - cy) * sin(angle);
    float y_new = cy + (x - cx) * sin(angle) + (y - cy) * cos(angle);
    x = x_new;
    y = y_new;
}

void drawrobot() {
    Brain.Screen.setPenWidth(3);
    Brain.Screen.setPenColor("#FFFFFF");



    
    float points[4][2] = {
        {240+y*10 - 50 / 2, 135+x*10 - 50 / 2},  // P1
        {240+y*10 + 50 / 2, 135+x*10 - 50 / 2},  // P2
        {240+y*10 + 50 / 2, 135+x*10 + 50 / 2},  // P3
        {240+y*10 - 50 / 2, 135+x*10 + 50 / 2}   // P4
    };

    
 double ang= toRad(360-imu.rotation(deg));
    
    for (int i = 0; i < 4; ++i) {
        rotatePoint(240+y*10, 135+x*10, points[i][0], points[i][1], ang);
    }
   
    float stick_x = 240+y*10 + 40 * cos(ang);  
    float stick_y = 135+x*10 + 40 * sin(ang);  

    
    Brain.Screen.drawLine(240+y*10, 135+x*10, stick_x, stick_y);
   
    for (int i = 0; i < 4; ++i) {
        int next = (i + 1) % 4;  // Wrap around to the first point after the last one
        Brain.Screen.drawLine(points[i][0], points[i][1], points[next][0], points[next][1]);
    }
}

void X_slider(int xpos1, int ypos1, int xpos2, int ypos2, double &slider_percent, float slider_width, string slider_color, string back_color) {
    if (BrainScreen_Print) {
    Brain.Screen.drawRectangle(xpos1, ypos1+0.33*(ypos2-ypos1), xpos2-xpos1, (ypos2-ypos1)*0.33, back_color.c_str());
    Brain.Screen.drawRectangle(xpos1+(xpos2-xpos1)*slider_percent-slider_width*0.5,ypos1, slider_width, ypos2-ypos1, slider_color.c_str());
    }
   
   if (BrainScreen_Switch) {
    cout << "Touch at (" << Sx << ", " << Sy << "), switch = " << BrainScreen_Switch << endl;
    if (Sx>xpos1 && Sx<xpos2 && Sy>ypos1 && Sy<ypos2) {
        
         slider_percent = double(Sx - xpos1) / (xpos2 - xpos1);
         cout << slider_percent << endl;
    }
}
}


void Y_slider(int xpos1, int ypos1, int xpos2, int ypos2, double &slider_percent, float slider_width, string slider_color, string back_color) {
    if (BrainScreen_Print) {
    Brain.Screen.drawRectangle(xpos1+0.33*(xpos2-xpos1), ypos1, (xpos2-xpos1)*0.33, ypos2-ypos1, back_color.c_str());
    Brain.Screen.drawRectangle(xpos1,ypos1+(ypos2-ypos1)*slider_percent-slider_width*0.5, xpos2-xpos1, slider_width, slider_color.c_str());
    }
   
   if (BrainScreen_Switch) {
    cout << "Touch at (" << Sx << ", " << Sy << "), switch = " << BrainScreen_Switch << endl;
    if (Sx>xpos1 && Sx<xpos2 && Sy>ypos1 && Sy<ypos2) {
        
         slider_percent = double(Sy - ypos1) / (ypos2 - ypos1);
         cout << slider_percent << endl;
    }
}
}
void circleslider(int center_x, int center_y, int radius, int slider_radius, double &slider_angle, string slider_color, string back_color) {
if (BrainScreen_Print) {
    Brain.Screen.drawCircle(center_x,center_y,radius,back_color.c_str());
        Brain.Screen.drawCircle(center_x + (radius-slider_radius) * cos(slider_angle),center_y + (radius-slider_radius) * sin(slider_angle), slider_radius,slider_color.c_str());
    }
    if (BrainScreen_Switch) {
        int dx = Sx - center_x;
        int dy = Sy - center_y;
        if ((dx * dx + dy * dy) <= (radius * radius)) {
            slider_angle=atan2(dy,dx);
        }
    }
}


void Graph_Data(int x1, int y1, int x2, int y2, string penColor, const deque<double>& Data) {
    if (Data.size() < 2) return; 

    double pointSpacing = static_cast<double>(x2 - x1) / (Data.size() - 1);
    int midpoint = (y1 + y2) / 2;
 int scalar = (y2 - y1) / 2;

    double maxAbs = max(
        abs(*max_element(Data.begin(), Data.end())),
        abs(*min_element(Data.begin(), Data.end()))
    );
    if (maxAbs == 0) maxAbs = 1; 

    Brain.Screen.setPenColor(penColor.c_str());

   
       for (int i = 0; i < Data.size() - 1; i++) {
        int xA = static_cast<int>(x1 + i * pointSpacing);
        int xB = static_cast<int>(x1 + (i + 1) * pointSpacing);
        int yA = static_cast<int>(midpoint - (Data[i] / maxAbs) * scalar);
        int yB = static_cast<int>(midpoint - (Data[i + 1] / maxAbs) * scalar);
        Brain.Screen.drawLine(xA, yA, xB, yB);
}
}

int PID_Turn_Test() {
    KPreadings.clear();
    KIreadings.clear();
    KDreadings.clear();
    wait(3,sec);
    turn_test();
    return 0;
}

int PID_Drive_Test() {
    KPreadings.clear();
    KIreadings.clear();
    KDreadings.clear();
    Time_Ran=0;
    wait(3,sec);
    drive_test();
    return 0;
}

int Reset_Robot() {
x=0;
y=0;
imu.calibrate();
}

void ScreenCases() {
     Brain.Screen.setFont(monoM);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setPenColor("#FFFFFF");
Brain.Screen.print(imu.rotation(degrees));
Brain.Screen.print(" ");
Brain.Screen.print(imu.heading(degrees));
Brain.Screen.print(" %.2f", x);
Brain.Screen.print(" %.2f", y);

Brain.Screen.print(" %d", auto_selection);
Brain.Screen.print(Autonames[auto_selection].c_str());

if (BrainScreen_Switch==true || BrainScreen_Print ==true)
{
switch (BrainScreen_Mode)
{
default: {
makeintbutton(20,45,100,200,BrainScreen_Mode,2, "#f5c200","Tuning","#FFFFFF");
 Brain.Screen.drawImageFromFile("Logo.png",150,40);
 makeintbutton(380,45,460,200,BrainScreen_Mode,9, "#f5c200","Routes","#FFFFFF");
break;
} 
case 2 : {
 Brain.Screen.setPenWidth(1);
  Brain.Screen.setFont(monoM);
  makeintbutton(20,60,153,200,BrainScreen_Mode,4,"#f5c200","PID","#FFFFFF");
  makeintbutton(200,130,280,200,BrainScreen_Mode,0,"#FF0000","Back", "#FFFFFF");
  makeintbutton(326,60,460,200,BrainScreen_Mode,3,"#f5c200","Odometry","#FFFFFF");
    break;
}
case 3:
{
Brain.Screen.setFont(monoM);
makeintbutton(0,25,100,75,BrainScreen_Mode ,2,"#FF0000","Back","#FFFFFF");
drawrobot();
BrainScreen_Switch=true;
    break;
}
case 4: {
Brain.Screen.setPenWidth(1);
  Brain.Screen.setFont(monoM);
  makeintbutton(20,60,153,200,BrainScreen_Mode,5,"#f5c200","Drives","#FFFFFF");
  makeintbutton(200,130,280,200,BrainScreen_Mode,2,"#FF0000","Back", "#FFFFFF");
  makeintbutton(326,60,460,200,BrainScreen_Mode,7,"#f5c200","Turns","#FFFFFF");
  break;
}
case 5: {
X_slider(20,20,350,70,Drive_KPslider,10,"#f5c200","#FFFFFF");
X_slider(20,90,350,140,Drive_KIslider,10,"#f5c200","#FFFFFF");
X_slider(20,160,350,210,Drive_KDslider,10,"#f5c200","#FFFFFF");
Test_driveKP=(Drive_KPslider-.5)*.5+.5;
Test_driveKI=(Drive_KIslider-.5)*.02;
Test_driveKD=(Drive_KDslider-.5)*.25+1.7;
Y_slider(400,30,460,190,Slider_Drive_Distance,10,"#f5c200","#FFFFFF");
Brain.Screen.printAt(60,220,"KP: %.2f", Test_driveKP);
Brain.Screen.printAt(150,220,"KI: %.2f", Test_driveKI);
Brain.Screen.printAt(240,220,"KD: %.2f", Test_driveKD);
Brain.Screen.printAt(330,220,"Inch:%.0f",(Slider_Drive_Distance-.5)*48);
makeintbutton(0,200,50,240,BrainScreen_Mode,4,"#FF0000","Exit","#FFFFFF");
makeintbutton(420,200,480,240,BrainScreen_Mode,6,"#f5c200","Test","#FFFFFF");

break;
}
case 6: {
     Graph_Data(0,20,480,210,"#FF0000",KDreadings);
    Graph_Data(0,20,480,210,"#f5c200",KIreadings);
    Graph_Data(0,20,480,210,"#FFFFFF",KPreadings);

Brain.Screen.printAt(60,220,"Time Ran: %.2f", Time_Ran*.01);
makeintbutton(0,200,50,240,BrainScreen_Mode,5,"#FF0000","Exit","#FFFFFF");
runintbutton(420,200,480,240,PID_Drive_Test,"#00FF00", "Run", "#000000");
break;
}
case 7: {
X_slider(20,20,350,70,Turn_KPslider,10,"#f5c200","#FFFFFF");
X_slider(20,90,350,140,Turn_KIslider,10,"#f5c200","#FFFFFF");
X_slider(20,160,350,210,Turn_KDslider,10,"#f5c200","#FFFFFF");
Test_driveKP=(Turn_KPslider-.5)*.5+.5;
Test_driveKI=(Turn_KIslider-.5)*.02;
Test_driveKD=(Turn_KDslider-.5)*.25+1.7;
Brain.Screen.printAt(60,220,"KP: %.2f", Test_driveKP);
Brain.Screen.printAt(150,220,"KI: %.2f", Test_driveKI);
Brain.Screen.printAt(240,220,"KD: %.2f", Test_driveKD);
Brain.Screen.printAt(330,220,"Ang:%.0f",(Slider_Drive_Distance-.5)*720);
circleslider(420,125,45,10,Circle_Turn_Angle,"#f5c200", "#cec7c7");
makeintbutton(0,200,50,240,BrainScreen_Mode,4,"#FF0000","Exit","#FFFFFF");
makeintbutton(420,200,480,240,BrainScreen_Mode,8,"#f5c200","Test","#FFFFFF");

break;
}
case 8: {
Graph_Data(0,20,480,220,"#FFFFFF",KDreadings);
    Graph_Data(0,20,480,220,"#FFFFFF",KIreadings);
    Graph_Data(0,20,480,220,"#FFFFFF",KPreadings);
makeintbutton(0,200,50,240,BrainScreen_Mode,7,"#FF0000","Exit","#FFFFFF");
runintbutton(420,200,480,240,PID_Turn_Test,"#00FF00", "Run", "#000000");
BrainScreen_Switch=true;
break;
}
case 9: {
Brain.Screen.setFont(monoM);

  Brain.Screen.printAt(35,50,"Special");
  autobutton(20,60,180,90,4,"#f5c200","Skills", "#FFFFFF");
  autobutton(20,105,180,135,3,"#f5c200","SS Solo AWP", "#FFFFFF");
  Brain.Screen.printAt(315,50,"Routine");
  autobutton(300,60,460,90,1,"#f5c200","4 Long 3 Tall", "#FFFFFF");
  autobutton(300,105,460,135,2,"#f5c200","4 Long 3 Short", "#FFFFFF");
  
  makeintbutton(200,60,280,220,BrainScreen_Mode,1,"#460000","Back", "#FFFFFF");
  break;
}
case 10: {
Brain.Screen.setPenColor("#FFFFFF");
  Brain.Screen.setFont(monoXL);
  Brain.Screen.printAt(20,60,Route_Names[auto_selection].c_str());
  Brain.Screen.setFont(monoL);
  
  Brain.Screen.setFont(monoM);
  Brain.Screen.setPenColor("#FFFFFF");
  Brain.Screen.printAt(20,130,Route_Descriptions[auto_selection][0].c_str());
  Brain.Screen.printAt(20,150,Route_Descriptions[auto_selection][1].c_str());
  Brain.Screen.printAt(20,170,Route_Descriptions[auto_selection][2].c_str());
  makeintbutton(40,185,150,225,BrainScreen_Mode ,9,"#FF0000","Back","#000000");
  runintbutton(330,185,440,225,Reset_Robot,"#444444","Reset","#000000");
  break;
}
 }
}
}

void Screen_Manager() {
    if (Brain.Screen.pressing())
{
 if (BrainScreen_Latch) {
BrainScreen_Switch=true;
  BrainScreen_Latch=false;
 }
}
else {
  BrainScreen_Latch=true;
}
ScreenCases();

Brain.Screen.render();
wait(100,msec);
Sx=Brain.Screen.xPosition();
Sy=Brain.Screen.yPosition();
if (BrainScreen_Switch)
{
 Brain.Screen.clearScreen();
}
else {
  Brain.Screen.setFont(monoM);
Brain.Screen.clearLine(1);
}
BrainScreen_Print=BrainScreen_Switch;
BrainScreen_Switch=false;
}
