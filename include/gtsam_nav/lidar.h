#ifndef LIDAR_H
#define LIDAR_H

#include <string.h>
#include <proj.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;
using namespace gtsam;


class AltitudeFactor : public NoiseModelFactor1<Pose3> {
private:
    // Altitude measurement
    double altitude_measurement_;

public:
    // Constructor
    AltitudeFactor(Key poseKey, double altitude_measurement, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor1<Pose3>(noiseModel, poseKey), altitude_measurement_(altitude_measurement) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H) const override {
        // Calculate altitude from pose
        double altitude = pose.translation().z();

        // Compute error (residual)
        double error = altitude - altitude_measurement_;

        // Compute Jacobian if requested
        if (H) {
            // Numerical differentiation to compute Jacobian
            H->resize(1,6); // jacobian wrt pose
            (*H) << 0, 0, 0, pose.rotation().matrix().block(2, 0, 1, 3);
        }

        return (Vector(1) << error).finished();
    }
};


#endif