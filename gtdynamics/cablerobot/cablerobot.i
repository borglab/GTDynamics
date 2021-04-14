/******************************* Cable Robot Wrapper Interface File *******************************/

namespace gtdynamics {

/****************************************** Factors ******************************************/

#include <gtdynamics/cablerobot/factors/CableLengthFactor.h>
class CableLengthFactor : gtsam::NonlinearFactor {
  CableLengthFactor(gtsam::Key l_key, gtsam::Key wTee_key,
                    const gtsam::noiseModel::Base *cost_model,
                    const gtsam::Point3 &wPb, const gtsam::Point3 &eePem);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/cablerobot/factors/CableVelocityFactor.h>
class CableVelocityFactor : gtsam::NonlinearFactor {
  CableVelocityFactor(gtsam::Key ldot_key, gtsam::Key wTee_key, gtsam::Key Vee_key,
                      const gtsam::noiseModel::Base* cost_model,
                      const gtsam::Point3 &wPb, const gtsam::Point3 &eePem);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/cablerobot/factors/CableTensionFactor.h>
class CableTensionFactor : gtsam::NonlinearFactor {
  CableTensionFactor(gtsam::Key tension_key, gtsam::Key eePose_key, gtsam::Key wrench_key,
                 const gtsam::noiseModel::Base* cost_model,
                 const gtsam::Point3 &wPb, const gtsam::Point3 &eePem);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

// need to borrow this from GTSAM since GTSAM doesn't have fixed-size vector versions
#include <gtdynamics/cablerobot/factors/PriorFactor.h>
template<T = {double, gtsam::Vector2, gtsam::Vector3, gtsam::Vector4, gtsam::Vector5, gtsam::Vector6}>
class PriorFactor : gtsam::NonlinearFactor {
  PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);
  T prior() const;

  void print(const string &s,
             const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/cablerobot/utils/CustomWrap.h>
gtsam::GaussianBayesNet* EliminateSequential(gtsam::GaussianFactorGraph graph,
                                             const gtsam::Ordering &ordering);
gtsam::GaussianBayesNet* BlockEliminateSequential(
    gtsam::GaussianFactorGraph graph, const BlockOrdering &ordering);

}  // namespace gtdynamics
