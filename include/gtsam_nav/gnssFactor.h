#ifndef GNSSFACTOR_H
#define GNSSFACTOR_H

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;




class GNSSFactor: public NoiseModelFactor1<Pose3> {

  private:

    typedef GNSSFactor This;
    typedef NoiseModelFactor1<Pose3> Base;

    Point2 measured_; /** The measurement */

  public:

    /** default constructor - only use for serialization */
    GNSSFactor() {}

    /** Constructor */
    GNSSFactor(Key posekey, const Point2 measured,
        const SharedNoiseModel& model) :
      Base(model, posekey), measured_(measured) {
    }

    ~GNSSFactor() override {}

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "BiasedGNSSFactor("
          << keyFormatter(this->key<1>()) << ")\n"
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
    Vector evaluateError(const Pose3& pose,
        OptionalMatrixType H) const override {

      if (H){
        H->resize(2,6); // jacobian wrt pose
        (*H) << Z_3x3.block(0, 0, 2, 3),  pose.rotation().matrix().block(0, 0, 2, 3);
      }
      return pose.translation().head<2>() - measured_;
    }

    /** return the measured */
    const Point2 measured() const {
      return measured_;
    }

  private:


  }; // \class GNSSFactor




class BiasedGNSSFactor: public NoiseModelFactor2<Pose3, Point2> {

  private:

    typedef BiasedGNSSFactor This;
    typedef NoiseModelFactor2<Pose3, Point2> Base;

    Point2 measured_; /** The measurement */

  public:

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
        OptionalMatrixType H1, OptionalMatrixType H2) const override {

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


  }; // \class BiasedGNSSFactor




#endif