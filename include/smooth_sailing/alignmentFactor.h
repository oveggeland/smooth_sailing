#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;
using namespace std;


// Finds a Rot3 that aligns a known ship rotation to a estimate of the roll and pitch
class AlignmentFactor : public NoiseModelFactor1<Rot3> {
private:
    Point2 sys_attitude_;
    Rot3 R_ship_;

public:
  AlignmentFactor(gtsam::Key key, Point2 sys_attitude, Rot3 R_ship, const SharedNoiseModel& noiseModel)
    : NoiseModelFactor1<Rot3>(noiseModel, key), sys_attitude_(sys_attitude), R_ship_(R_ship){}

  gtsam::Vector evaluateError(const Rot3& R_align, boost::optional<gtsam::Matrix&> H = boost::none) const override {

    // We use right multiplication with R_align (gives identity jacobian)
    Rot3 R_comp = R_ship_.compose(R_align);//, H1, H2); //.compose(R_align);//, H_Rcomp_Rship, H_Rcomp_Ralign);
    Point2 roll_pitch = R_comp.rpy(H).head<2>();
    
    if (H) {
        *H = H->topRows(2);
    }

    return roll_pitch - sys_attitude_; 
  }
};