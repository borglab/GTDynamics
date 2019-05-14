/**
 *  @file  testPaddleFactor.h
 *  @brief A factor that evaluates forward movements on a simple turtle model.
 *  @author Stephen Eick
 *  @author Frank Dellaert
 *  @date April 2019
 */

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam
{
/**
 * A factor that evaluates forward movements on a simple turtle model.
 */
class PaddleFactor : public NoiseModelFactor1<Vector3>
{
private:
  // Constructor variables
  double t_, omega_, length_, alpha_, beta_;

public:
  /// Construct from time and penalty parameters
  /// t - current time
  /// omega - rate of change of the phase
  /// length - the length of the leg
  /// alpha - scaling factor for error
  /// beta - scaling factor for error
  PaddleFactor(Key key, double t = 0, double omega = 0, double length = 1,
               double alpha = 1, double beta = 1) : NoiseModelFactor1(nullptr, key), t_(t), omega_(omega), length_(length), alpha_(alpha), beta_(beta)
  {
  }

  /// Evaluate the error which favors underwater backwards velocity.
  Vector evaluateError(const Vector3 &c, boost::optional<Matrix &> error_H_c =
                                             boost::none) const override;
};
} // namespace gtsam
