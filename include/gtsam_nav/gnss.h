#ifndef GNSS_H
#define GNSS_H

#include <string.h>
#include <proj.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "sensor_msgs/NavSatFix.h"

using namespace std;
using namespace gtsam;


class BiasedGNSSFactor: public NoiseModelFactor2<Pose3, Point2> {

  private:

    typedef BiasedGNSSFactor This;
    typedef NoiseModelFactor2<Pose3, Point2> Base;

    Point2 measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<BiasedGNSSFactor> shared_ptr;

    /** default constructor - only use for serialization */
    BiasedGNSSFactor() {}

    /** Constructor */
    BiasedGNSSFactor(Key posekey, Key biaskey, const Point2 measured,
        const SharedNoiseModel& model) :
      Base(model, posekey, biaskey), measured_(measured) {
    }

    ~BiasedGNSSFactor() override {}

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "BiasedGNSSFactor("
          << keyFormatter(this->key<1>()) << ","
          << keyFormatter(this->key<2>()) << ")\n"
          << "  measured: " << measured_.transpose() << std::endl;
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& pose, const Point2& bias,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
            boost::none) const override {

      if (H1 || H2){
        H1->resize(2,6); // jacobian wrt pose
        (*H1) << Z_3x3.block(0, 0, 2, 3),  pose.rotation().matrix().block(0, 0, 2, 3);
        H2->resize(2,2); // jacobian wrt bias
        (*H2) << I_2x2;
      }
      return pose.translation().head<2>() + bias - measured_;
    }

    /** return the measured */
    const Point2 measured() const {
      return measured_;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class BiasedGNSSFactor


class GNSSHandle{
    public:
        // Constructors
        GNSSHandle(string src="EPSG:4326", string target="EPSG:6052");

        Vector2 projectCartesian(sensor_msgs::NavSatFix::ConstPtr msg); // Retrieve local cartesian coordinates from navsatfix message

    private:
        // Member variables
        string crs_source;
        string crs_target;

        PJ_CONTEXT *C;
        PJ *P;

        PJ_COORD input_coords, output_coords; // https://proj.org/development/reference/datatypes.html#c.PJ_COORD
};


#endif