#include <gtdynamics/utils/DebugUtils.h>

#include <iostream>

namespace gtsam {

/* ************************************************************************* */
void PrintGraphWithError(
    const NonlinearFactorGraph &graph, const Values &values,
    double error_threshold,
    const KeyFormatter &key_formatter) {
  std::cout << "total error: " << graph.error(values) << std::endl;
  for (const auto& factor: graph) {
    double error = factor->error(values);
    if (error > error_threshold) {
      std::cout << "====================== factor ==========================\n";
      factor->print("", key_formatter);
      std::cout << "error: " << error << std::endl;
    }
  }

}


}  // namespace gtsam
