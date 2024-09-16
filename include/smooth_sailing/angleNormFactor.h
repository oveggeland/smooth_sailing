#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Unit3.h>

using namespace gtsam;


// Constraints the vector L to be within a angle theta of nG_.s
class AngularConstraintFactor : public NoiseModelFactor1<Point3> {
private:
    double dotProductThreshold_;
    Unit3 nG_;

public:
  AngularConstraintFactor(gtsam::Key key, Unit3 nG, double maxAngle, const SharedNoiseModel& noiseModel)
    : NoiseModelFactor1<Point3>(noiseModel, key), nG_(nG){
      dotProductThreshold_ = cos(maxAngle);
    }

  gtsam::Vector evaluateError(const Point3& L, boost::optional<gtsam::Matrix&> H = boost::none) const override {

    double dotProduct = Unit3(L).dot(nG_);

    if (L.isZero() || dotProduct >= dotProductThreshold_) {
      if (H) *H = gtsam::Matrix::Zero(1, 3);
      return gtsam::Vector1::Zero();
    }
    
    if (H) {
      *H = nG_.unitVector().transpose();
    }
    return Vector1(dotProduct - dotProductThreshold_); 
  }
};