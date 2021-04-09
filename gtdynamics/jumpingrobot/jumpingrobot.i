/******************************* Jumping Robot Wrapper Interface File *******************************/

namespace gtdynamics {

/****************************************** Factors ******************************************/

#include <gtdynamics/jumpingrobot/factors/PneumaticFactors.h>
#include <gtdynamics/jumpingrobot/factors/PneumaticActuatorFactors.h>
class GasLawFactor: gtsam::NonlinearFactor{
   GasLawFactor(gtsam::Key p_key, gtsam::Key v_key, gtsam::Key m_key,
                 const gtsam::noiseModel::Base *cost_model,
                 const double c);
};

class MassFlowRateFactor: gtsam::NonlinearFactor{
  MassFlowRateFactor(gtsam::Key pm_key, gtsam::Key ps_key, gtsam::Key mdot_key,
                 const gtsam::noiseModel::Base* cost_model,
                 const double D, const double L, const double mu, const double epsilon, const double k);
};

class ValveControlFactor: gtsam::NonlinearFactor{
  ValveControlFactor(gtsam::Key t_key, gtsam::Key to_key, gtsam::Key tc_key, 
                     gtsam::Key mdot_key, gtsam::Key true_mdot_key,
                     const gtsam::noiseModel::Base *cost_model,
                    const double ct);
};

class ActuatorVolumeFactor: gtsam::NonlinearFactor{
  ActuatorVolumeFactor(gtsam::Key v_key, gtsam::Key l_key,
                 const gtsam::noiseModel::Base *cost_model,
                 const double D, const double L);
};

class ClippingActuatorFactor: gtsam::NonlinearFactor{
  ClippingActuatorFactor(gtsam::Key delta_x_key, gtsam::Key p_key,
                          gtsam::Key f_key,
                          const gtsam::noiseModel::Base *cost_model);
};

class SmoothActuatorFactor: gtsam::NonlinearFactor{
  SmoothActuatorFactor(gtsam::Key delta_x_key, gtsam::Key p_key,
                          gtsam::Key f_key,
                          const gtsam::noiseModel::Base *cost_model);
};

class ForceBalanceFactor: gtsam::NonlinearFactor{
  ForceBalanceFactor(gtsam::Key delta_x_key, gtsam::Key q_key, gtsam::Key f_key,
                     const gtsam::noiseModel::Base *cost_model,
                     const double k, const double r, const double q_rest,
                     const bool contract);
};

class JointTorqueFactor: gtsam::NonlinearFactor{
  JointTorqueFactor(gtsam::Key q_key, gtsam::Key v_key, gtsam::Key f_key,
                    gtsam::Key torque_key,
                    const gtsam::noiseModel::Base *cost_model,
                    const double q_limit, const double ka, const double r,
                    const double b, const bool positive);
};

}  // namespace gtdynamics
