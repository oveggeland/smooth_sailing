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
    virtual Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const override {
        // Calculate altitude from pose
        double altitude = pose.translation().z();

        // Compute error (residual)
        double error = altitude - altitude_measurement_;

        // Compute Jacobian if requested
        if (H) {
            // Numerical differentiation to compute Jacobian
            *H = numericalDerivative11<Vector, Pose3>(std::function<Vector(const Pose3&)>([this](const Pose3& p) {
                return evaluateError(p);
            }), pose);
        }

        return (Vector(1) << error).finished();
    }

    // Static create function for convenient factor creation
    static boost::shared_ptr<AltitudeFactor> Create(Key poseKey, double altitude_measurement, const SharedNoiseModel& noiseModel) {
        return boost::make_shared<AltitudeFactor>(poseKey, altitude_measurement, noiseModel);
    }
};


#endif