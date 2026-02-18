# Kinodynamics Architecture Refactor Plan

## Purpose

This document defines a phased refactor plan for separating kinematics, statics, and dynamics responsibilities while preserving the current `DynamicsGraph` public API.  
Each phase is intended to be independently implementable by an agent.

Primary goals:
- Keep behavior stable while reducing duplicated logic.
- Make slice-level physics explicit (`Statics`, `Dynamics`) and keep trajectory assembly in `DynamicsGraph`.
- Make shared wrench/torque logic reusable by both statics and dynamics.

## Current State (As Of 2026-02-13)

### What exists today

- `Kinematics` is a solver-oriented class with slice/interval/phase support.
  - Q/V-level graph construction and optimization helpers exist.
  - See:
    - `gtdynamics/kinematics/Kinematics.h`
    - `gtdynamics/kinematics/KinematicsSlice.cpp`
    - `gtdynamics/kinematics/KinematicsInterval.cpp`
    - `gtdynamics/kinematics/KinematicsPhase.cpp`

- `DynamicsGraph` is a broad orchestrator/facade.
  - It already delegates q/v factors to an internal `Kinematics` instance.
  - It still directly builds acceleration factors and dynamic wrench/contact factors.
  - It also owns trajectory and multiphase collocation assembly.
  - See:
    - `gtdynamics/dynamics/DynamicsGraph.h`
    - `gtdynamics/dynamics/DynamicsGraph.cpp`

- `Statics` exists and is functional for slice-level static wrench solving.
  - Inherits `Kinematics`.
  - Implements static wrench balance graph + solve/minimize methods.
  - See:
    - `gtdynamics/statics/Statics.h`
    - `gtdynamics/statics/StaticsSlice.cpp`
    - `gtdynamics/statics/StaticWrenchFactor.h`

- `Dynamics` currently exists only as math utilities.
  - Contains `Coriolis` and `MatVecMult`.
  - It is not yet a solver class.
  - See:
    - `gtdynamics/dynamics/Dynamics.h`
    - `gtdynamics/dynamics/Dynamics.cpp`

- Shared joint-side factor expressions already exist and are reused:
  - `WrenchEquivalenceFactor`
  - `TorqueFactor`
  - `WrenchPlanarFactor`
  - See `gtdynamics/factors/*.h`.

### Key architectural gap

`DynamicsGraph` still contains slice-level dynamics assembly that should move into a dedicated `Dynamics` solver layer, and there is partial overlap between statics/dynamics wrench graph building loops.

There is also a parameter hierarchy mismatch:
- `OptimizerSetting` currently derives from `KinematicsParameters`.
- `Statics` has its own parameter type, but there is no shared statics/dynamics parameter base.
- `Dynamics` (as a future solver class) needs a parameter model aligned with `Kinematics` style.

## End State

### Class responsibilities

- `Kinematics`
  - Kinematic constraints and objectives (q/v level).
  - Context support: `Slice`, `Interval`, `Phase`, `Trajectory` where applicable.

- `Statics`
  - Dedicated static solver for rest-state wrench equilibrium.
  - Remains independent from `Dynamics` implementation.

- `Dynamics` (new solver class)
  - Dedicated dynamic solver for slice-level motion factors.
  - Does not inherit from or compose with `Statics`.
  - Owns:
    - `aFactors(Slice, ...)` (migrated from `DynamicsGraph`)
    - `graph(Slice, ...)` for dynamic-only wrench/contact factors that are not in the three statics-provided groups

- `DynamicsGraph`
  - Backward-compatible facade and trajectory assembler.
  - Delegates:
    - q/v to `Kinematics`
    - a-level slice assembly to `Dynamics`
    - dynamics-level assembly as:
      - `Statics` slice interface for the three shared groups
      - `Dynamics::graph(Slice, ...)` for all remaining factors
  - Owns collocation and multiphase stitching.

### Parameter hierarchy

Parameter classes should follow the same style as `KinematicsParameters`: explicit typed parameter structs with inheritance for shared concerns.

Required target hierarchy:
- `KinematicsParameters` (existing).
- `MechanicsParameters` (new): shared statics/dynamics parameters.
  - Typical shared fields: `gravity`, `planar_axis`, and any shared wrench/joint/contact dynamics noise models.
- `StaticsParameters : MechanicsParameters`.
- `DynamicsParameters : MechanicsParameters`.
- `OptimizerSetting : DynamicsParameters`.

Notes:
- This is a class hierarchy requirement, not a behavior rewrite.
- Existing constructor ergonomics should be preserved where possible.

### Reuse model (no duplication)

`DynamicsGraph::dynamicsFactors` should obtain the following three factor groups through the `Statics` slice interface, not by direct standalone factor calls in the dynamics path.

These factor groups are:
- `WrenchEquivalenceFactor`
- `TorqueFactor`
- `WrenchPlanarFactor`

Practical rule:
- The direct constructors above are allowed inside `Statics` implementation.
- `DynamicsGraph::dynamicsFactors` should add these groups via `Statics` slice methods.
- `Dynamics::graph(Slice, ...)` must not add these three groups and should only add the remaining dynamic-specific factors.

## Non-Goals

- No redesign of factor math or noise model semantics in this refactor.
- No forced API break for existing `DynamicsGraph` callers.
- No full optimizer framework rewrite.
- No broad Python API expansion until C++ layering is stable.
- No migration of linear FD/ID path in this plan (`linearDynamicsGraph`, `linearSolveFD`, `linearSolveID` remain out of scope).

## Implementation Rules For Agents

- Keep each phase isolated. Do not implement future phases early.
- Preserve existing behavior and test expectations unless explicitly called out.
- Maintain compatibility with current wrappers unless phase says otherwise.
- Do not introduce inheritance/composition between `Dynamics` and `Statics`.
- In dynamics assembly paths, do not directly call `WrenchEquivalenceFactor`, `TorqueFactor`, or `WrenchPlanarFactor`; obtain those groups via the `Statics` slice interface.
- `Dynamics::graph(Slice, ...)` is dynamic-only delta: it must exclude those three statics-provided groups.
- Enforce parameter hierarchy target: `OptimizerSetting` must derive from `DynamicsParameters`.

---

## Phased Plan

## Phase 0 (Prerequisite): Parameter Hierarchy Alignment

### Objective

Align statics/dynamics parameter types with `Kinematics`-style parameter modeling before moving factor builders.

### Scope

In scope:
- Introduce a shared statics/dynamics base parameter class, e.g. `MechanicsParameters`.
- Define/adjust:
  - `StaticsParameters : MechanicsParameters`
  - `DynamicsParameters : MechanicsParameters`
- Change `OptimizerSetting` inheritance to:
  - `OptimizerSetting : DynamicsParameters`
- Preserve behavior and defaults.

Out of scope:
- Factor-graph migration.
- `DynamicsGraph` delegation changes.

### Expected file touchpoints

- `gtdynamics/statics/Statics.h`
- `gtdynamics/dynamics/Dynamics.h`
- `gtdynamics/dynamics/OptimizerSetting.h`
- optional `.cpp` files for constructor/default wiring.

### Acceptance criteria

- Parameter hierarchy matches the target model in this document.
- Existing call sites compile without behavior regressions.
- No factor construction logic is moved in this phase.

### Suggested tests

- `make -j6 testStatics.run` (from `build/`)
- `make -j6 testDynamicsGraph.run` (from `build/`)

---

## Phase 1: Move `DynamicsGraph::aFactors` To `Dynamics::aFactors(Slice, ...)`

### Objective

Create the first real slice-level API in `Dynamics` by migrating acceleration-related factor construction out of `DynamicsGraph`.

### Scope

In scope:
- Introduce/expand `Dynamics` class to expose `aFactors(const Slice&, ...)`.
- Move implementation currently in `DynamicsGraph::aFactors` into `Dynamics`.
- Keep behavior identical.

Out of scope:
- Changing factor math.
- Moving `dynamicsFactors` logic.

### Expected file touchpoints

- Update:
  - `gtdynamics/dynamics/Dynamics.h`
  - `gtdynamics/dynamics/Dynamics.cpp`
  - `gtdynamics/dynamics/DynamicsSlice.cpp` (new or equivalent)
  - `gtdynamics/dynamics/DynamicsGraph.cpp`

### Acceptance criteria

- `Dynamics::aFactors(Slice, ...)` exists and is used by `DynamicsGraph`.
- `DynamicsGraph::aFactors(...)` is removed or reduced to a thin wrapper.
- Existing nonlinear behavior is unchanged.

### Suggested tests

- `make -j6 testDynamicsGraph.run` (from `build/`)
- `make -j6 testTrajectory.run` (from `build/`, smoke for integration)

---

## Phase 2: Move `DynamicsGraph::dynamicsFactors` To `Dynamics::graph(Slice, ...)`

### Objective

Move slice-level dynamic wrench/contact graph assembly into `Dynamics`.

### Scope

In scope:
- Implement `Dynamics::graph(const Slice&, const Robot&, contact_points, mu)`.
- Migrate only the non-statics-group part of current `DynamicsGraph::dynamicsFactors`.
- Keep these three groups out of `Dynamics::graph(...)`:
  - `WrenchEquivalenceFactor`
  - `TorqueFactor`
  - `WrenchPlanarFactor`
- Keep `DynamicsGraph::dynamicsFactors` responsible for obtaining those three groups via `Statics` slice methods and composing with `Dynamics::graph(...)`.
- Keep behavior identical, including contact/friction handling.

Out of scope:
- Trajectory/collocation migration.
- Linear FD/ID path.

### Expected file touchpoints

- `gtdynamics/dynamics/Dynamics.h`
- `gtdynamics/dynamics/Dynamics.cpp`
- New file(s), e.g.:
  - `gtdynamics/dynamics/DynamicsSlice.cpp`
  - `gtdynamics/dynamics/DynamicsParameters.h` (or nested in `Dynamics.h`)

### Acceptance criteria

- `Dynamics::graph(Slice, ...)` contains only factors not in the three statics-provided groups.
- `Dynamics` remains independent of `Statics` (no inheritance/composition).
- `DynamicsGraph::dynamicsFactors` composes:
  - `Statics` slice groups (`WrenchEquivalenceFactor`, `TorqueFactor`, `WrenchPlanarFactor`)
  - `Dynamics::graph(Slice, ...)` delta.
- `Dynamics::graph(Slice, ...)` does not directly add those three factor groups.
- Existing callers are unaffected.
- Existing tests continue passing.

### Suggested tests

- `make -j6 testDynamics.run`
- `make -j6 testDynamicsGraph.run`

---

## Phase 3: Delegate `DynamicsGraph` Slice Dynamics To `Dynamics`

### Objective

Turn `DynamicsGraph` into orchestration/facade for slice-level dynamics instead of owning the full implementation.

### Scope

In scope:
- `DynamicsGraph` composes/uses `Dynamics` similarly to current internal `Kinematics` usage.
- `dynamicsFactors` delegates to `Dynamics::graph(Slice, ...)` (or becomes a thin adapter).
- `dynamicsFactorGraph` uses `Dynamics` for both a-level and dynamics-level slice assembly.
- `dynamicsFactors` explicitly combines `Statics` slice groups plus `Dynamics` delta graph.
- Preserve current signatures and behavior.

Out of scope:
- Removing current public methods from `DynamicsGraph`.
- Trajectory rewrites.

### Expected file touchpoints

- `gtdynamics/dynamics/DynamicsGraph.h`
- `gtdynamics/dynamics/DynamicsGraph.cpp`

### Acceptance criteria

- `DynamicsGraph` still passes all existing tests.
- No behavioral diff for known tests and examples.
- TODOs about migrating to `Dynamics::graph<Slice>` are resolved.

### Suggested tests

- `make -j6 testDynamicsGraph.run`
- `make -j6 testTrajectory.run`
- `make -j6 testPhase.run`

---

## Phase 4: Finalize Delegation And Remove Redundant Paths

### Objective

Finalize delegation, remove redundant code paths, and resolve migration TODOs.

### Scope

In scope:
- Remove migrated duplicate logic from `DynamicsGraph`.
- Resolve migration TODOs in `DynamicsGraph` related to `Dynamics::graph<Slice>` and statics reuse comments where no longer applicable.
- Keep interfaces clean and explicit for future phases.

Out of scope:
- Major symbolic expression redesign.
- Changing numerical behavior intentionally.

### Expected file touchpoints

- Update:
  - `gtdynamics/dynamics/DynamicsGraph.cpp`
  - `gtdynamics/dynamics/Dynamics.h`
  - `gtdynamics/dynamics/DynamicsSlice.cpp` (or equivalent)

### Acceptance criteria

- No duplicated a/dynamics slice assembly remains in `DynamicsGraph`.
- `DynamicsGraph` acts as facade/orchestrator for slice-level dynamics calls.
- Tests pass with unchanged behavior.

### Suggested tests

- `make -j6 testStaticsSlice.run`
- `make -j6 testDynamicsGraph.run`

---

## Phase 5: Context Expansion (Optional But Recommended)

### Objective

Mirror `Kinematics` context pattern for `Dynamics` and `Statics` where useful.

### Scope

In scope:
- Add explicit support helpers for `Interval` / `Phase` composition from slice-level builders where practical.
- Keep trajectory assembly in `DynamicsGraph`.

Out of scope:
- Removing existing trajectory API.

### Acceptance criteria

- Context expansion is additive and tested.
- No regression in multiphase planning paths.

### Suggested tests

- `make -j6 testPhase.run`
- `make -j6 testTrajectory.run`
- `make -j6 testDynamicsGraph.run`

---

## Phase 6: Wrapper Exposure And Documentation

### Objective

Expose stable new architecture pieces to wrappers only after C++ layering is stable.

### Scope

In scope:
- Update wrapper interface (`gtdynamics.i`) if `Statics` and new `Dynamics` APIs are deemed stable.
- Add/update docs and examples.

Out of scope:
- Early wrapper exposure before stability.

### Acceptance criteria

- Wrapper builds and tests pass (if wrapper changes are included).
- Docs reflect final class responsibilities and recommended usage.

### Suggested tests

- `make -j6 python-test` (if wrapper touched)
- Existing C++ test set for touched areas.

---

## Agent Handoff Template

Use this exact prompt pattern when assigning:

1. "Implement Phase X from `doc/kinodynamics-architecture-refactor-plan.md`."
2. "Do not implement future phases."
3. "Preserve behavior and public API unless the phase explicitly says otherwise."
4. "Run the listed suggested tests for that phase."
5. "Summarize:
   - files changed
   - any deviations from phase scope
   - test results."

## Definition Of Done (Whole Refactor)

- `DynamicsGraph` is a clean facade/orchestrator.
- `Kinematics`, `Statics`, and `Dynamics` each have clear solver responsibilities.
- Shared wrench/torque infrastructure is reused, not duplicated.
- Existing behaviors remain stable per test suite.
- Documentation and wrappers (if updated) match the final architecture.
