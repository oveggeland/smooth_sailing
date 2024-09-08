#ifndef EULER_ANGLE_FACTOR_H
#define EULER_ANGLE_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

class EulerAngleFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    gtsam::Vector3 measuredAngles_;  // Euler angles (roll, pitch, yaw)

    // Constructor
    EulerAngleFactor(gtsam::Key key, const gtsam::Vector3& measuredAngles, const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), measuredAngles_(measuredAngles) {}

    // Error function
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H = boost::none) const override {
        gtsam::Matrix3 H_rpy;  // Jacobian of roll, pitch, yaw
        gtsam::Vector3 rpy = pose.rotation().rpy(H_rpy);  // Roll, pitch, yaw
        gtsam::Vector3 error = rpy - measuredAngles_;

        // Compute the Jacobian if needed
        if (H) {
            *H = H_rpy;
        }

        return error;
    }
};

#endif // EULER_ANGLE_FACTOR_H