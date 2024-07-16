#ifndef COMMON_H
#define COMMON_H

#include <math.h>
#include <gtsam/inference/Symbol.h>

#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::G;  // GNSS OFFSET (north, east)

#endif