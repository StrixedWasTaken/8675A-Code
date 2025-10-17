/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cmath>

#include "v5.h"
#include "v5_vcs.h"
#include "robot-config.h"
#include <iostream>


using namespace vex;
using namespace std;

// to in

// double toInch(double input, double wheel_diameter, double gear_ratio) {
//   return (input / 360) * (gear_ratio * (M_PI * wheel_diameter));
// }

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)