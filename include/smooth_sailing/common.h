#ifndef COMMON_H
#define COMMON_H

#include <math.h>
#include <gtsam/inference/Symbol.h>

#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::L;  // Point3 (bLbs - lever arm from body to ship buyonacy center)
using gtsam::symbol_shorthand::Z;  // Z(0) is mean height over waterline

#endif