#ifndef ALTITUDEFACTOR_H
#define ALTITUDEFACTOR_H

#include <string.h>
#include <proj.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;
using namespace gtsam;

// Vanilla factor
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
        double altitude = pose.translation(H).z();

        // Compute error (residual)
        Vector1 error = Vector1(altitude - altitude_measurement_);

        if (H) {
            // Numerical differentiation to compute Jacobian
            //H->resize(1, 6);
            (*H) = H->block(2, 0, 1, 6);
        }

        return error;
    }
};

// Constrained altitude factor
class ConstrainedAltitudeFactor : public NoiseModelFactor2<Pose3, Pose3> {
    private:

    double dt_;
    double tau_;

    SharedGaussian discreteNoiseModel(double sigma, double dt){
        return noiseModel::Isotropic::Sigma(1, sigma / sqrt(dt));
    }

public:
    // Constructor
    ConstrainedAltitudeFactor(Key pose1Key, Key pose2Key, double dt, double tau, double sigma)
        : NoiseModelFactor2<Pose3, Pose3>(discreteNoiseModel(sigma, dt), pose1Key, pose2Key), dt_(dt), tau_(tau) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& pose1, const Pose3& pose2, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const override {
        // Calculate altitude from pose
        double z1 = pose1.translation(H1).z();
        double z2 = pose1.translation(H2).z();

        double alpha = exp(-1/tau_ * dt_);

        if (H1) *H1 = -alpha*H1->block(2, 0, 1, 6);
        if (H2) *H2 = H2->block(2, 0, 1, 6);

        return Vector1(z2 - alpha*z1);
    }
};






class LeveredAltitudeFactor : public NoiseModelFactor2<Pose3, Point3> {
private:
    // No measurement

public:
    // Constructor
    LeveredAltitudeFactor(Key poseKey, Key leverKey, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Point3>(noiseModel, poseKey, leverKey) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& pose, const Point3& bLbs, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const override {
        
        // Predicted altitude from lever arm and rotation
        Point3 wLws = pose.transformFrom(bLbs, H1, H2);
        Vector1 error(wLws[2]);

        if (H1 || H2) {
            // cout << endl;
            // cout << "bLbs: " << bLbs << endl;
            // cout << "wLws: " << wLws << endl;

            *H1 = H1->block(2, 0, 1, 6);
            // cout << "H1 is: " << *H1 << endl;

            *H2 = H2->block(2, 0, 1, 3);
            // cout << "H2 is: " << *H2 << endl;
        }

        return error;
    }
};


#endif