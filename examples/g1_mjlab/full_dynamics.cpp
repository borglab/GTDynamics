/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file full_dynamics.cpp
 * @brief Build and solve the full G1 dynamics constraint graph.
 * @author Frank Dellaert
 */

// Full GTDynamics forward-dynamics constraint graph for the G1 comparison.
// Only fixed model parameters, q/v, applied forces and the collision/contact
// law are imported. No mass matrix, bias force, or body/contact Jacobian
// enters.
#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <Eigen/Cholesky>
#include <boost/json/src.hpp>
#include <chrono>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtdynamics/constraints/FrictionConeConstraints.h>
#include <gtdynamics/dynamics/TwistAccelFactor.h>
#include <gtdynamics/dynamics/WrenchFactor.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/universal_robot/ForwardKinematics.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>
#include <iostream>
#include <map>
#include <type_traits>

using namespace gtsam;
using namespace gtdynamics;
namespace json = boost::json;
using Clock = std::chrono::steady_clock;

namespace {
Vector vector(const json::value &value) {
  const auto &array = value.as_array();
  Vector out(array.size());
  for (size_t i = 0; i < array.size(); ++i)
    out(i) = json::value_to<double>(array[i]);
  return out;
}
Matrix matrix(const json::value &value) {
  const auto &array = value.as_array();
  Matrix out(array.size(), array.empty() ? 0 : array[0].as_array().size());
  for (size_t i = 0; i < array.size(); ++i)
    out.row(i) = vector(array[i]).transpose();
  return out;
}
json::array encode(const Vector &value) {
  json::array out;
  for (double v : value)
    out.push_back(v);
  return out;
}
json::array encodeMatrix(const Matrix &value) {
  json::array out;
  for (int i = 0; i < value.rows(); ++i)
    out.push_back(encode(value.row(i)));
  return out;
}
Pose3 pose(const json::value &position, const json::value &quaternion) {
  const Vector q = vector(quaternion).normalized();
  return Pose3(Rot3::Quaternion(q(0), q(1), q(2), q(3)), vector(position));
}
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// Unary quadratic factors keep each link/joint/contact as its own graph key.
template <typename T> class LinearCost : public NoiseModelFactor1<T> {
  Matrix A_;
  Vector b_;

public:
  LinearCost(Key key, const Matrix &A, const Vector &b)
      : NoiseModelFactor1<T>(noiseModel::Unit::Create(b.size()), key), A_(A),
        b_(b) {}
  Vector evaluateError(const T &value,
                       OptionalMatrixType H = nullptr) const override {
    if (H)
      *H = A_;
    if constexpr (std::is_same_v<T, double>)
      return A_ * Vector1(value) - b_;
    else
      return A_ * value - b_;
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<LinearCost>(*this);
  }
};

// Rotor inertia is an explicit scalar term in the joint effort equation.
class ArmatureTorque : public NoiseModelFactor2<Vector6, double> {
  Vector6 axis_;
  double armature_, applied_;

public:
  ArmatureTorque(const JointSharedPtr &joint, double armature, double applied)
      : NoiseModelFactor2<Vector6, double>(
            noiseModel::Unit::Create(1),
            WrenchKey(joint->child()->id(), joint->id(), 0),
            JointAccelKey(joint->id(), 0)),
        axis_(joint->cScrewAxis()), armature_(armature), applied_(applied) {}
  Vector evaluateError(const Vector6 &wrench, const double &accel,
                       OptionalMatrixType H1 = nullptr,
                       OptionalMatrixType H2 = nullptr) const override {
    if (H1)
      *H1 = axis_.transpose();
    if (H2)
      *H2 = Matrix11::Constant(armature_);
    return Vector1(axis_.dot(wrench) + armature_ * accel - applied_);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ArmatureTorque>(*this);
  }
};

// A contact force produces a wrench at its link COM. The moment arm comes from
// GTDynamics FK and the collision point; no generalized contact Jacobian is
// used.
template <typename Force>
class ContactWrench : public NoiseModelFactor2<Vector6, Force> {
  Matrix map_;

public:
  ContactWrench(Key wrench, Key force, const Matrix &map)
      : NoiseModelFactor2<Vector6, Force>(noiseModel::Unit::Create(6), wrench,
                                           force),
        map_(map) {}
  Vector evaluateError(const Vector6 &wrench, const Force &force,
                       OptionalMatrixType H1 = nullptr,
                       OptionalMatrixType H2 = nullptr) const override {
    if (H1)
      *H1 = Matrix6::Identity();
    if (H2)
      *H2 = -map_;
    return wrench - map_ * force;
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ContactWrench>(*this);
  }
};

// MuJoCo serializes contact forces as (n,t_x,t_y); the shared GTDynamics
// constraints use (t_x,t_y,n). Apply this permutation at input/output boundaries
// and in the wrench/cost maps, keeping the external comparison schema unchanged.
Matrix3 mujocoForceOrder() {
  Matrix3 order;
  order << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  return order;
}

struct Problem {
  Robot robot;
  LinkSharedPtr root;
  std::map<int, LinkSharedPtr> links;
  std::map<int, JointSharedPtr> joints;
  std::map<int, int> dof;
  std::map<int, Vector6> accelerationBias;
  std::map<int, Matrix> accelerationMap; // Validation/output only; never used
                                         // to condense dynamics.
  std::map<Key, std::pair<int, int>> forceBlocks;
  Matrix6 rootMap;
  Values known, zero, initial;
  NonlinearFactorGraph cost, physicalCost, frictionPenalty;
  NonlinearEqualityConstraints equalities;
  NonlinearInequalityConstraints inequalities;
  int nv, nf;
  double assemblySeconds = 0, initializationSeconds = 0, objectiveConstant = 0;

  explicit Problem(const json::object &input) {
    auto start = Clock::now();
    if (input.at("schema").as_string() != "gtdynamics-full-graph-v1")
      throw std::invalid_argument("Expected a full graph snapshot");
    for (const char *forbidden : {"M", "J", "bias", "free_acc", "response"})
      if (input.contains(forbidden))
        throw std::invalid_argument("Precomputed dynamics are forbidden");
    const Vector q = vector(input.at("qpos")), v = vector(input.at("qvel"));
    const Vector armature = vector(input.at("armature")),
                 applied = vector(input.at("applied"));
    const Vector R = vector(input.at("R")), aref = vector(input.at("aref"));
    nv = v.size();
    nf = R.size();
    if (applied.size() != nv || armature.size() != nv || aref.size() != nf ||
        (R.array() <= 0).any())
      throw std::invalid_argument("Invalid dynamics/contact dimensions");
    std::map<int, Pose3> rest{{0, Pose3()}}, bodyToCom;
    LinkMap linkMap;
    JointMap jointMap;
    for (const auto &value : input.at("bodies").as_array()) {
      const auto &body = value.as_object();
      int id = json::value_to<int>(body.at("id"));
      int parent = json::value_to<int>(body.at("parent"));
      rest[id] =
          rest.at(parent) * pose(body.at("position"), body.at("quaternion"));
      bodyToCom[id] = pose(body.at("com_position"), body.at("com_quaternion"));
      auto link = std::make_shared<Link>(
          id, json::value_to<std::string>(body.at("name")),
          json::value_to<double>(body.at("mass")),
          vector(body.at("inertia")).asDiagonal(),
          rest.at(id) * bodyToCom.at(id), rest.at(id));
      links[id] = link;
      linkMap[link->name()] = link;
      const auto &joint = body.at("joint").as_object();
      const int j = json::value_to<int>(joint.at("id"));
      const int qi = json::value_to<int>(joint.at("qpos_address"));
      const int vi = json::value_to<int>(joint.at("dof_address"));
      if (joint.at("free").as_bool()) {
        if (root || qi != 0 || vi != 0 || parent != 0 ||
            armature.head<6>().norm() != 0)
          throw std::invalid_argument(
              "Expected one floating root without armature");
        root = link;
        // Recorded GPU states have float32 quaternion rounding. Normalize the
        // orientation on import, as MuJoCo's kinematics does as well.
        const Vector4 rotation = q.segment<4>(3).normalized();
        const Pose3 worldBody(Rot3::Quaternion(rotation(0), rotation(1),
                                               rotation(2), rotation(3)),
                              q.head<3>());
        const Matrix6 comFromBody = bodyToCom.at(id).inverse().AdjointMap();
        Matrix6 bodyMap = Matrix6::Zero();
        bodyMap.topRightCorner<3, 3>().setIdentity();
        bodyMap.bottomLeftCorner<3, 3>() =
            worldBody.rotation().matrix().transpose();
        rootMap = comFromBody * bodyMap;
        const Vector6 bodyVelocity = bodyMap * v.head<6>();
        Vector6 bias = Vector6::Zero();
        bias.tail<3>() = -bodyVelocity.head<3>().cross(bodyVelocity.tail<3>());
        accelerationBias[id] = comFromBody * bias;
        accelerationMap[id] = Matrix::Zero(6, nv);
        accelerationMap[id].leftCols(6) = rootMap;
        known.insert(PoseKey(id, 0), worldBody * bodyToCom.at(id));
        known.insert<Vector6>(TwistKey(id, 0), rootMap * v.head<6>());
      } else {
        const Pose3 worldJoint =
            rest.at(id) * Pose3(Rot3(), vector(joint.at("position")));
        auto hinge = std::make_shared<RevoluteJoint>(
            j, json::value_to<std::string>(joint.at("name")), worldJoint,
            links.at(parent), link, vector(joint.at("axis")));
        links.at(parent)->addJoint(hinge);
        link->addJoint(hinge);
        joints[j] = hinge;
        jointMap[hinge->name()] = hinge;
        dof[j] = vi;
        known.insert(JointAngleKey(j, 0),
                     q(qi) - json::value_to<double>(joint.at("reference")));
        known.insert(JointVelKey(j, 0), v(vi));
      }
    }
    if (!root)
      throw std::invalid_argument("Missing floating root");
    robot = Robot(linkMap, jointMap);
    FKTraversal(root).run(0, &known);

    // Independently propagate the zero-generalized-acceleration bias. This is
    // kinematics, not a forward-dynamics solution. A_i-b_i = J_i*qdd supplies
    // the quadratic acceleration energy without building J' I J or its inverse.
    for (const auto &[j, joint] : joints) {
      int child = joint->child()->id(), parent = joint->parent()->id();
      const Pose3 cTp = known.at<Pose3>(PoseKey(child, 0))
                            .between(known.at<Pose3>(PoseKey(parent, 0)));
      const Matrix6 transform = cTp.AdjointMap();
      accelerationBias[child] =
          transform * accelerationBias.at(parent) +
          Pose3::adjointMap(known.at<Vector6>(TwistKey(child, 0))) *
              joint->cScrewAxis() * v(dof.at(j));
      accelerationMap[child] = transform * accelerationMap.at(parent);
      accelerationMap[child].col(dof.at(j)) += joint->cScrewAxis();
    }

    NonlinearFactorGraph equalityCosts;
    std::map<int, std::vector<Key>> external;
    const Key baseExternal = Symbol('e', root->id());
    known.insert<Vector6>(
        baseExternal, rootMap.transpose().fullPivLu().solve(applied.head<6>()));
    external[root->id()].push_back(baseExternal);
    const bool elliptic = input.at("cone").as_string() == "elliptic";
    const std::string frictionModel = input.if_contains("friction_model")
        ? json::value_to<std::string>(input.at("friction_model"))
        : (elliptic ? "elliptic_norm" : "pyramidal");
    const bool signedSquare = frictionModel == "signed_square";
    const bool frictionFactor = frictionModel == "friction_factor";
    if ((elliptic && frictionModel != "elliptic_norm" && !signedSquare && !frictionFactor) ||
        (!elliptic && frictionModel != "pyramidal"))
      throw std::invalid_argument("Friction formulation does not match contact coordinates");
    const double frictionSigma = input.if_contains("friction_sigma")
        ? json::value_to<double>(input.at("friction_sigma")) : 0.001;
    if (frictionFactor && (!(frictionSigma > 0) || !std::isfinite(frictionSigma)))
      throw std::invalid_argument("Friction factor requires a positive finite sigma");
    NonlinearFactorGraph frictionCosts;
    int count = 0;
    for (const auto &value : input.at("contacts").as_array()) {
      const auto &contact = value.as_object();
      int id = json::value_to<int>(contact.at("body"));
      int offset = json::value_to<int>(contact.at("address")),
          dim = elliptic ? 3 : 4;
      Vector2 mu = vector(contact.at("friction"));
      if (offset < 0 || offset + dim > nf || (mu.array() <= 0).any())
        throw std::invalid_argument("Invalid contact block");
      Key force = Symbol('f', count), wrench = ContactWrenchKey(id, count, 0);
      forceBlocks[force] = {offset, dim};
      ++count;
      const Matrix edges = elliptic
          ? Matrix(mujocoForceOrder())
          : Matrix(mujocoForceOrder() *
                   PyramidalFrictionConeConstraint::ForceMatrix(mu));
      const Pose3 worldCom = known.at<Pose3>(PoseKey(id, 0));
      const Vector3 point =
          worldCom.transformTo(Vector3(vector(contact.at("position"))));
      const Matrix directions = json::value_to<double>(contact.at("sign")) *
                                worldCom.rotation().matrix().transpose() *
                                matrix(contact.at("frame")).transpose() * edges;
      Matrix map(6, dim);
      map.bottomRows(3) = directions;
      for (int col = 0; col < dim; ++col)
        map.col(col).head<3>() = point.cross(Vector3(directions.col(col)));
      external[id].push_back(wrench);
      zero.insert<Vector6>(wrench, Vector6::Zero());
      const Vector sqrtR = R.segment(offset, dim).cwiseSqrt();
      const Vector target = aref.segment(offset, dim).cwiseQuotient(sqrtR);
      objectiveConstant += 0.5 * target.squaredNorm();
      auto constraints = elliptic
          ? EllipticFrictionConeConstraint::Create(force, mu)
          : PyramidalFrictionConeConstraint::Create(force);
      if (elliptic) {
        equalityCosts.emplace_shared<ContactWrench<Vector3>>(wrench, force, map);
        zero.insert<Vector3>(force, Vector3::Zero());
        cost.emplace_shared<LinearCost<Vector3>>(
            force, sqrtR.asDiagonal() * mujocoForceOrder(), target);
      } else {
        equalityCosts.emplace_shared<ContactWrench<Vector4>>(wrench, force, map);
        zero.insert<Vector4>(force, Vector4::Zero());
        cost.emplace_shared<LinearCost<Vector4>>(force, sqrtR.asDiagonal(), target);
      }
      if (signedSquare) {
        // Same elliptic feasible set, but a differentiable signed-square
        // residual whose gradient vanishes at the zero-force apex.
        constraints.resize(0);
        constraints.emplace_shared<SignedSquareFrictionConeConstraint>(force, mu);
      } else if (frictionFactor) {
        // Keep the normal >= 0 row. The tangential cone is enforced only by
        // the actual library penalty factor, which changes the objective.
        constraints.resize(1);
        const Matrix frame = matrix(contact.at("frame"));
        const Vector3 gravity = vector(input.at("gravity"));
        if (std::abs(mu(0) - mu(1)) > 1e-12 ||
            std::abs(std::abs(frame(0, 2)) - 1) > 1e-10 ||
            gravity(0) != 0 || gravity(1) != 0 || gravity(2) == 0)
          throw std::invalid_argument("Friction penalty requires isotropic friction on horizontal ground");
        frictionCosts.emplace_shared<ContactDynamicsFrictionConeFactor>(
            PoseKey(id, 0), wrench, noiseModel::Isotropic::Sigma(1, frictionSigma),
            mu(0), gravity);
      }
      for (const auto &constraint : constraints)
        inequalities.push_back(constraint);
    }
    std::vector<bool> covered(nf, false);
    for (const auto &[key, block] : forceBlocks) {
      for (int i = block.first; i < block.first + block.second; ++i) {
        if (covered[i])
          throw std::invalid_argument("Overlapping contact blocks");
        covered[i] = true;
      }
    }
    if (std::find(covered.begin(), covered.end(), false) != covered.end())
      throw std::invalid_argument(
          "Contact forces must cover the contact-law coordinates");
    const Vector3 gravity = vector(input.at("gravity"));
    for (const auto &[id, link] : links) {
      std::vector<Key> wrenches = external[id];
      for (const auto &joint : link->joints()) {
        const Key key = WrenchKey(id, joint->id(), 0);
        wrenches.push_back(key);
        zero.insert<Vector6>(key, Vector6::Zero());
      }
      zero.insert<Vector6>(TwistAccelKey(id, 0), Vector6::Zero());
      equalityCosts.add(WrenchFactor(noiseModel::Unit::Create(6), link,
                                     wrenches, 0, gravity));
      const Matrix sqrtI = link->inertiaMatrix().llt().matrixU();
      cost.emplace_shared<LinearCost<Vector6>>(TwistAccelKey(id, 0), sqrtI,
                                               sqrtI * accelerationBias.at(id));
    }
    for (const auto &[j, joint] : joints) {
      zero.insert(JointAccelKey(j, 0), 0.0);
      equalityCosts.add(
          TwistAccelFactor(noiseModel::Unit::Create(6), joint, 0));
      equalityCosts.emplace_shared<ExpressionFactor<Vector6>>(
          noiseModel::Unit::Create(6), Vector6::Zero(),
          joint->wrenchEquivalenceConstraint(0));
      equalityCosts.emplace_shared<ArmatureTorque>(joint, armature(dof.at(j)),
                                                   applied(dof.at(j)));
      if (armature(dof.at(j)) > 0)
        cost.emplace_shared<LinearCost<double>>(
            JointAccelKey(j, 0),
            Matrix11::Constant(std::sqrt(armature(dof.at(j)))),
            Vector1::Zero());
    }
    physicalCost = cost;
    frictionPenalty = ConstVarGraph(frictionCosts, known);
    cost.add(frictionPenalty);
    equalities = NonlinearEqualityConstraints::FromCostGraph(
        ConstVarGraph(equalityCosts, known));
    if (zero.dim() != equalities.dim() + nf)
      throw std::runtime_error("Unexpected full dynamics graph dimension");
    assemblySeconds = elapsed(start);

    // Both solvers start with identical feasible forces, then solve the full
    // equality graph once to initialize accelerations and internal wrenches.
    // No reference accelerations/forces or condensed dynamics enter this solve.
    start = Clock::now();
    NonlinearFactorGraph initialization = equalities.penaltyGraph();
    for (const auto &[key, block] : forceBlocks) {
      Vector force = Vector::Zero(block.second);
      if (elliptic)
        force(0) = 1;
      if (input.contains("prescribed_force"))
        force = vector(input.at("prescribed_force"))
                    .segment(block.first, block.second);
      if (elliptic)
        initialization.addPrior<Vector3>(
            key, Vector3(mujocoForceOrder().transpose() * force),
            noiseModel::Unit::Create(3));
      else
        initialization.addPrior<Vector4>(key, Vector4(force),
                                        noiseModel::Unit::Create(4));
    }
    initial =
        zero.retract(initialization.linearize(zero)->optimize(EliminateQR));
    initializationSeconds = elapsed(start);
    if (equalities.violationNorm(initial) > 1e-7)
      throw std::runtime_error(
          "Full dynamics initialization did not satisfy equalities");
  }

  Vector acceleration(const Values &values) const {
    Vector result = Vector::Zero(nv);
    result.head<6>() = rootMap.fullPivLu().solve(
        values.at<Vector6>(TwistAccelKey(root->id(), 0)) -
        accelerationBias.at(root->id()));
    for (const auto &[j, vi] : dof)
      result(vi) = values.at<double>(JointAccelKey(j, 0));
    return result;
  }
  Vector force(const Values &values) const {
    Vector result = Vector::Zero(nf);
    for (const auto &[key, block] : forceBlocks) {
      if (block.second == 3)
        result.segment<3>(block.first) =
            mujocoForceOrder() * values.at<Vector3>(key);
      else
        result.segment<4>(block.first) = values.at<Vector4>(key);
    }
    return result;
  }
  json::object diagnostics() const {
    json::array bodies;
    for (const auto &[id, link] : links) {
      bodies.push_back(json::object{
          {"id", id},
          {"pose", encodeMatrix(known.at<Pose3>(PoseKey(id, 0)).matrix())},
          {"twist", encode(known.at<Vector6>(TwistKey(id, 0)))},
          {"acceleration_bias", encode(accelerationBias.at(id))},
          {"acceleration_map", encodeMatrix(accelerationMap.at(id))}});
    }
    return {{"links", links.size()},
            {"joints", joints.size()},
            {"variables", initial.size()},
            {"ambient_dimension", initial.dim()},
            {"equality_factors", equalities.size()},
            {"equality_dimension", equalities.dim()},
            {"inequality_dimension", inequalities.dim()},
            {"contact_dimension", nf},
            {"initial_equality_l2", equalities.violationNorm(initial)},
            {"bodies", bodies}};
  }
};

json::object solve(const json::object &input) {
  Problem problem(input);
  const std::string solver =
      input.contains("solver") ? json::value_to<std::string>(input.at("solver"))
                               : "CMC";
  Values result;
  size_t iterations = 0;
  bool converged = false;
  json::array components;
  double manifoldSeconds = 0;
  auto start = Clock::now();
  if (input.contains("prescribed_force") || problem.nf == 0) {
    result = problem.initial;
  } else if (solver == "AL") {
    auto p = std::make_shared<AugmentedLagrangianParams>();
    p->maxIterations = 60;
    p->absoluteViolationTolerance = 1e-6;
    p->absoluteStationarityTolerance = 1e-6;
    p->storeOptProgress = true;
    p->lmParams.maxIterations = 200;
    p->lmParams.absoluteErrorTol = p->lmParams.relativeErrorTol = 1e-12;
    p->lmParams.lambdaUpperBound = 1e12;
    AugmentedLagrangianOptimizer optimizer(
        ConstrainedOptProblem(problem.cost, problem.equalities,
                              problem.inequalities),
        problem.initial, p);
    result = optimizer.optimize();
    if (!optimizer.progress().empty()) {
      iterations = optimizer.progress().back().iteration;
      converged = optimizer.progress().back().converged;
    }
  } else if (solver == "CMC") {
    IELMParams p;
    p.iqpMaxIterations = 1000;
    p.lmParams.maxIterations = 500;
    p.lmParams.absoluteErrorTol = p.lmParams.relativeErrorTol = 1e-12;
    auto mp = std::make_shared<IEConstraintManifold::Params>();
    mp->affineEqualities = true; // Fixed q/v makes the dynamics graph affine.
    // GTDynamics constructs the recovery basis by eliminating the physical
    // constraint graph. Force keys select coordinates; no recovery matrix or
    // manually reduced objective is supplied by the benchmark.
    mp->equalityBasisCreator =
        std::make_shared<EliminationBasisCreator>([](const KeyVector &keys) {
          KeyVector independent;
          for (Key key : keys)
            if (Symbol(key).chr() == 'f')
              independent.push_back(key);
          return independent;
        });
    auto rp = std::make_shared<IERetractorParams>();
    rp->ensureFeasible = true;
    rp->feasibleThreshold = 1e-8;
    rp->lmParams.absoluteErrorTol = rp->lmParams.relativeErrorTol = 1e-12;
    // With q and v fixed, these GTDynamics dynamics/acceleration equalities
    // are affine. The retractor obtains their reduction from eBasis(), and
    // evaluates the original inequalities with AL in those reduced coordinates.
    const std::string backend = input.if_contains("retractor")
        ? json::value_to<std::string>(input.at("retractor")) : "reduced_al";
    IERetractor::shared_ptr retractor;
    if (backend == "reduced_al") {
      retractor = std::make_shared<ReducedALRetractor>(rp);
    } else if (backend == "ambient_barrier" || backend == "reduced_barrier") {
      rp->useReducedCoordinates = backend == "reduced_barrier";
      retractor = std::make_shared<BarrierRetractor>(rp);
    } else {
      throw std::invalid_argument("Unknown CMC retractor");
    }
    mp->retractorCreator =
        std::make_shared<UniversalIERetractorCreator>(retractor);
    auto manifolds = IEOptimizer::identifyManifolds(
        problem.equalities, problem.inequalities, problem.initial, mp);
    manifoldSeconds = elapsed(start);
    for (const auto &[key, manifold] : manifolds)
      components.push_back(
          json::object{{"variables", manifold.values().size()},
                       {"ambient_dimension", manifold.values().dim()},
                       {"reduced_dimension", manifold.dim()},
                       {"basis_dimension", manifold.eBasis()->dim()}});
    IELMOptimizer optimizer(p, mp);
    result = optimizer.optimizeManifolds(
        problem.cost, manifolds,
        IEOptimizer::identifyUnconstrainedValues(
            problem.equalities, problem.inequalities, problem.initial));
    iterations = optimizer.details().size();
  } else
    throw std::invalid_argument("Unknown full-graph solver");
  const double seconds = elapsed(start);
  return {{"solver", solver},
          {"acceleration", encode(problem.acceleration(result))},
          {"force", encode(problem.force(result))},
          {"objective", problem.physicalCost.error(result) - problem.objectiveConstant},
          {"penalty_cost", problem.frictionPenalty.error(result)},
          {"equality_l2", problem.equalities.violationNorm(result)},
          {"inequality_l2", problem.inequalities.violationNorm(result)},
          {"seconds", seconds},
          {"assembly_seconds", problem.assemblySeconds},
          {"initialization_seconds", problem.initializationSeconds},
          {"manifold_seconds", manifoldSeconds},
          {"dynamics_seconds",
           seconds + problem.assemblySeconds + problem.initializationSeconds},
          {"iterations", iterations},
          {"al_converged",
           solver == "AL" ? json::value(converged) : json::value(nullptr)},
          {"components", components},
          {"graph", problem.diagnostics()}};
}
template <typename Force> void testContact(const Force &force,
    const NonlinearInequalityConstraints &constraints) {
  const Key key = Symbol('f', 0), wrench = Symbol('w', 0);
  Values values;
  values.insert<Force>(key, force);
  for (const auto &constraint : constraints) {
    const auto boundary = constraint->createEqualityConstraint();
    if (!internal::testFactorJacobians("contact boundary",
        *boundary->penaltyFactor(), values, 1e-5, 1e-7))
      throw std::runtime_error("Contact boundary Jacobian mismatch");
    if (constraint->unwhitenedExpr(values)(0) > 0 ||
        boundary->unwhitenedError(values)(0) !=
            constraint->unwhitenedExpr(values)(0))
      throw std::runtime_error("Contact inequality/boundary sign mismatch");
  }
  const Matrix map = Matrix::Random(6, force.size());
  values.insert<Vector6>(wrench, Vector6::Random());
  ContactWrench<Force> contact(wrench, key, map);
  if (!internal::testFactorJacobians("contact wrench", contact, values, 1e-5, 1e-7))
    throw std::runtime_error("Contact wrench Jacobian mismatch");
}

void selfTest() {
  const Key key = Symbol('f', 0);
  const Vector2 mu(0.6, 0.4);
  testContact(Vector3(0.2, 0.1, 3),
              EllipticFrictionConeConstraint::Create(key, mu));
  testContact(Vector4(3, 0.2, 0.1, 0.4),
              PyramidalFrictionConeConstraint::Create(key));
  NonlinearInequalityConstraints signedConstraints;
  signedConstraints.emplace_shared<SignedSquareFrictionConeConstraint>(key, mu);
  testContact(Vector3(0.2, 0.1, 3), signedConstraints);
  const Vector3 force(0.2, -0.1, 3);
  const Vector3 mujocoForce(3, 0.2, -0.1);
  if (!(mujocoForceOrder() * force).isApprox(mujocoForce) ||
      !(mujocoForceOrder().transpose() * mujocoForce).isApprox(force))
    throw std::runtime_error("Contact coordinate permutation mismatch");
  Matrix expected(3, 4);
  expected << 1, 1, 1, 1, mu(0), -mu(0), 0, 0, 0, 0, mu(1), -mu(1);
  if (!(mujocoForceOrder() * PyramidalFrictionConeConstraint::ForceMatrix(mu))
          .isApprox(expected))
    throw std::runtime_error("Pyramid ray order mismatch");
  {
    const Key pk = Symbol('p', 0), wk = Symbol('w', 0);
    const Pose3 pose(Rot3::RzRyRx(.2, -.3, .4), Point3(1, 2, 3));
    ContactDynamicsFrictionConeFactor factor(pk, wk,
        noiseModel::Isotropic::Sigma(1, .001), .6, Vector3(0,0,-9.81));
    // Check the exact factor, including its pose Jacobian and fixed-pose use.
    for (const Vector3& f : {Vector3(.2,.1,3), Vector3(.2,.1,-3),
                            Vector3(2,1,1), Vector3(2,1,-1),
                            Vector3::Zero().eval(), Vector3(.2,.1,0)}) {
      Vector6 wrench = Vector6::Zero();
      wrench.tail<3>() = pose.rotation().unrotate(f);
      Values values; values.insert(pk,pose); values.insert(wk,wrench);
      const double expected = std::max(0.0,f.head<2>().squaredNorm()-.36*f(2)*f(2));
      if (std::abs(factor.unwhitenedError(values)(0)-expected)>1e-12)
        throw std::runtime_error("Friction factor residual mismatch");
      if (!internal::testFactorJacobians("actual friction factor",factor,values,1e-5,1e-5))
        throw std::runtime_error("Friction factor Jacobian mismatch");
      Values known;known.insert(pk,pose);
      NonlinearFactorGraph graph;graph.add(factor);
      auto fixed=ConstVarGraph(graph,known);
      Values unknown;unknown.insert(wk,wrench);
      if (std::abs(fixed.error(unknown)-factor.error(values))>1e-8)
        throw std::runtime_error("Fixed-pose friction penalty mismatch");
    }
    Matrix hp,hw;Vector6 boundary=Vector6::Zero();boundary.tail<3>()=Vector3(.6,0,1);
    if (factor.evaluateError(Pose3(),boundary,&hp,&hw)(0)!=0 || hp.norm()!=0 || hw.norm()!=0)
      throw std::runtime_error("Boundary derivative convention changed");
  }

}

} // namespace

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-test") {
    selfTest();
    std::cout << "Full-graph contact boundary and wrench Jacobians passed.\n";
    return 0;
  }
  if (argc == 2 && std::string(argv[1]) == "--build-info") {
    std::cout << json::serialize(json::object{
                     {"formulation", "full GTDynamics constraint graph"},
                     {"build_type", GTDYNAMICS_BENCHMARK_BUILD_TYPE},
                     {"compiler", GTDYNAMICS_BENCHMARK_COMPILER},
                     {"cmc_basis",
                      "GTDynamics EliminationBasis; contact force keys"},
                     {"cmc_retractor", "reduced_al"},
                     {"affine_basis_reuse", true},
                     {"projection_metric_cache", true},
                     {"shared_affine_projection", true},
                     {"feasible_projection_bypass", true},
                     {"qp_backend", "GTSAM dense ActiveSetSolver"}})
              << '\n';
    return 0;
  }
  std::ostream protocol(std::cout.rdbuf());
  std::cout.rdbuf(std::cerr.rdbuf());
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      protocol << json::serialize(solve(json::parse(line).as_object())) << '\n'
               << std::flush;
    } catch (const std::exception &e) {
      protocol << json::serialize(json::object{{"error", e.what()}}) << '\n'
               << std::flush;
    }
  }
}
