#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;


// Factor to make bLbs approach the plane (a,b,c,d) defined in plane_model
class LeverArmFactor : public NoiseModelFactor1<Point3> {
private:
    Vector4 plane_;
    Matrix13 H_;

public:
  LeverArmFactor(gtsam::Key key, Vector4 plane_model, const SharedNoiseModel& noiseModel)
    : NoiseModelFactor1<Point3>(noiseModel, key), plane_(plane_model){
        H_ = plane_model.head<3>();
    }

  gtsam::Vector evaluateError(const Point3& bLbs, boost::optional<gtsam::Matrix&> H = boost::none) const override {

    if (H) {
      *H = H_; // Jacobian in constant
    }

    return Vector1(plane_[0]*bLbs[0] + plane_[1]*bLbs[1] + plane_[2]*bLbs[2] + plane_[3]);
  }
};