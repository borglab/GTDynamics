/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file capture.cpp
 * @brief Export a full G1 QP before solving, using a pinned GTSAM ABI.
 * @author GTDynamics contributors
 */

#include <dlfcn.h>
#include <gtsam/constrained/ActiveSetSolver.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace {
// Native-endian uint64 headers and column-major float64 arrays. This temporary
// stream is converted to portable, non-pickled sparse NPZ by pack.py.
void writeInteger(std::ofstream& stream, uint64_t value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

void writeFactor(std::ofstream& stream, const gtsam::JacobianFactor& factor,
                 bool objective) {
  const gtsam::Matrix matrix = factor.getA();
  const gtsam::Vector rhs = factor.getb();
  gtsam::Vector sigmas = gtsam::Vector::Ones(rhs.size());
  gtsam::Vector inverseSigmas = sigmas;
  if (objective && factor.get_model()) {
    if (factor.get_model()->isConstrained())
      throw std::runtime_error("Unexpected constrained objective noise");
    sigmas = factor.get_model()->sigmas();
    inverseSigmas = factor.get_model()->invsigmas();
  }
  writeInteger(stream, factor.size());
  writeInteger(stream, matrix.rows());
  writeInteger(stream, matrix.cols());
  for (auto key = factor.begin(); key != factor.end(); ++key) {
    writeInteger(stream, *key);
    writeInteger(stream, factor.getA(key).cols());
  }
  stream.write(reinterpret_cast<const char*>(matrix.data()),
               matrix.size() * sizeof(double));
  stream.write(reinterpret_cast<const char*>(rhs.data()),
               rhs.size() * sizeof(double));
  stream.write(reinterpret_cast<const char*>(sigmas.data()),
               sigmas.size() * sizeof(double));
  stream.write(reinterpret_cast<const char*>(inverseSigmas.data()),
               inverseSigmas.size() * sizeof(double));
}
}  // namespace

namespace gtsam {
// This diagnostic interposes the existing method; it does not change the first
// solve, working-set initialization, tolerances, or objective. No solver code
// is patched. Stop before initializing the SECOND full-size QP. Smaller
// initialization/projection QPs are passed through.
ActiveSetSolver::InternalState ActiveSetSolver::initialState(
    const Values& values, const State* warmStart) const {
  const char* output = std::getenv("G1_QP_CAPTURE_DIR");
  size_t dimension = 0;
  for (const auto& [key, dim] : keyDims_) dimension += dim;
  static size_t globalQpCount = 0;
  const char* minimumDimension = std::getenv("G1_QP_CAPTURE_MIN_DIM");
  const size_t minDim =
      minimumDimension ? std::stoull(minimumDimension) : 10000;
  const bool globalQp =
      output && problemType_ == ProblemType::QP && dimension >= minDim;
  if (globalQp) {
    ++globalQpCount;
    std::fprintf(stderr, "GLOBAL_QP index=%zu dimension=%zu inequalities=%zu\n",
                 globalQpCount, dimension, constraints_.inequalities.size());
  }
  if (globalQp && globalQpCount == 2) {
    if (!constraints_.equalities.empty() || params_->regularization != 0)
      throw std::runtime_error("Unexpected equality rows or regularization");
    if (constraints_.inequalities.empty())
      throw std::runtime_error("Expected active inequalities in second QP");
    const std::string prefix = std::string(output) + "/";
    std::ofstream costs(prefix + "costs.bin", std::ios::binary);
    std::ofstream inequalities(prefix + "inequalities.bin", std::ios::binary);
    costs.exceptions(std::ios::badbit | std::ios::failbit);
    inequalities.exceptions(std::ios::badbit | std::ios::failbit);
    std::ofstream keys(prefix + "keys.csv");
    keys << "key,offset,dim\n";
    size_t offset = 0;
    VectorValues probe, zero;
    for (const auto& [key, dim] : keyDims_) {
      if (!values.at<Vector>(key).isZero(0) || warmStart)
        throw std::runtime_error(
            "Expected zero initial values and no warm start");
      keys << key << ',' << offset << ',' << dim << '\n';
      Vector block(dim);
      for (size_t i = 0; i < dim; ++i)
        block(i) = 1e-4 * (static_cast<int>((offset + i) % 17) - 8);
      probe.insert(key, block);
      zero.insert(key, Vector::Zero(dim));
      offset += dim;
    }
    writeInteger(costs, qpCostGraph_.size());
    for (const auto& factor : qpCostGraph_) {
      const auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(factor);
      if (!jacobian) throw std::runtime_error("Expected square-root cost");
      writeFactor(costs, *jacobian, true);
    }
    std::ofstream gradients(prefix + "native_gradients.bin", std::ios::binary);
    gradients.exceptions(std::ios::badbit | std::ios::failbit);
    for (const auto& point : {zero, probe}) {
      const auto gradient = qpCostGraph_.gradient(point);
      for (const auto& [key, dim] : keyDims_)
        gradients.write(reinterpret_cast<const char*>(gradient.at(key).data()),
                        dim * sizeof(double));
    }
    std::ofstream rows(prefix + "rows.csv");
    rows << "index,constraint_index,block_row,multiplier_key,probe_residual\n";
    rows << std::setprecision(17);
    writeInteger(inequalities, constraints_.inequalities.size());
    for (size_t i = 0; i < constraints_.inequalities.size(); ++i) {
      const auto& row = constraints_.inequalities[i];
      writeFactor(inequalities, *row.factor, false);
      rows << i << ',' << row.constraintIndex << ',' << row.row << ','
           << row.multiplierKey << ',' << row.factor->unweighted_error(probe)(0)
           << '\n';
    }
    std::ofstream checks(prefix + "native_checks.json");
    checks << std::setprecision(17) << "{\"dimension\":" << dimension
           << ",\"zero_cost\":" << qpCostGraph_.error(zero)
           << ",\"probe_cost\":" << qpCostGraph_.error(probe)
           << ",\"regularization\":" << params_->regularization
           << ",\"max_iterations\":" << params_->maxIterations
           << ",\"active_tolerance\":" << params_->activeTolerance
           << ",\"step_tolerance\":" << params_->stepTolerance
           << ",\"feasibility_tolerance\":" << params_->feasibilityTolerance
           << ",\"multiplier_tolerance\":" << params_->multiplierTolerance
           << "}\n";
    costs.close();
    inequalities.close();
    keys.close();
    rows.close();
    checks.close();
    gradients.close();
    std::fprintf(stderr, "CAPTURE_COMPLETE dimension=%zu inequalities=%zu\n",
                 dimension, constraints_.inequalities.size());
    // Intentional capture termination, not a solver failure.
    std::_Exit(86);
  }
  using Function =
      InternalState (*)(const ActiveSetSolver*, const Values&, const State*);
  const char* libraryPath = std::getenv("G1_QP_GTSAM_LIBRARY");
  if (!libraryPath) throw std::runtime_error("Set G1_QP_GTSAM_LIBRARY");
  static void* library = dlopen(libraryPath, RTLD_NOW | RTLD_NOLOAD);
  static auto original = reinterpret_cast<Function>(
      dlsym(library,
            "_ZNK5gtsam15ActiveSetSolver12initialStateERKNS_6ValuesEPKNS_"
            "20ActiveSetSolverStateE"));
  if (!original)
    throw std::runtime_error("Cannot resolve original initialState");
  return original(this, values, warmStart);
}
}  // namespace gtsam
