#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;


// Factor to make bLbs approach the plane fitted by LiDAR measurements
class LeverArmFactor : public NoiseModelFactor1<Point3> {
private:
    Vector4 plane_;
    double normalization_factor_;
    Matrix13 H_;

public:
  LeverArmFactor(gtsam::Key key, double a, double b, double c, double d, const SharedNoiseModel& noiseModel)
    : NoiseModelFactor1<Point3>(noiseModel, key){
        plane_ = (Vector(4) << a, b, c, d).finished();
        normalization_factor_ = 1 / sqrt(a*a + b*b + c*c);
        H_ = normalization_factor_*(Vector(3) << a, b, c).finished();
    }

  gtsam::Vector evaluateError(const Point3& bLbs, boost::optional<gtsam::Matrix&> H = boost::none) const override {

    if (H) {
      *H = H_; // Jacobian in constant
    }

    return Vector1(normalization_factor_*(plane_[0]*bLbs[0] + plane_[1]*bLbs[1] + plane_[2]*bLbs[2] + plane_[3]));
  }
};