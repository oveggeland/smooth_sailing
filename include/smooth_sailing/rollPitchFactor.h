#ifndef ROLL_PITCH_FACTOR_H
#define ROLL_PITCH_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>

class RollPitchFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    gtsam::Point2 measured_;  // The Point2 measurement (roll, pitch)

    // Constructor
    RollPitchFactor(gtsam::Key key, const gtsam::Point2& measured, const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key), measured_(measured) {}

    // Error function
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H = boost::none) const override {

        // Compute the error between measured roll/pitch and the current pose's roll/pitch
        gtsam::Matrix3 H_rpy;
        gtsam::Vector3 rpy = pose.rotation().rpy(H_rpy);
        gtsam::Vector2 error = {rpy.x() - measured_.x(), rpy.y() - measured_.y()};

        // Compute the Jacobian if needed
        if (H) {
            *H = H_rpy.topRows<2>();
            // Pose3's rotation matrix affects roll and pitch
            // Set perturbations to evaluate how changes in rotation affect roll and pitch
            //*H = (gtsam::Matrix(2, 6) << 1, 0, 0, 0, 0, 0,   // Roll w.r.t. rotation (affects the last 3 elements)
            //                            0, 1, 0, 0, 0, 0).finished(); // Pitch w.r.t. rotation
        }

        return error;
    }
};

#endif // ROLL_PITCH_FACTOR_H
