# Runtime manifold types and their values

Authors: Yetong Zhang and Frank Dellaert.

The runtime types and point values have matching specialization hierarchies:

```text
ConstraintManifold           CMPoint
└── IEConstraintManifold      └── IECMPoint
    └── EConstraintManifold       └── ECMPoint
```

`ConstraintManifold` is the abstract chart and recovery contract.
`IEConstraintManifold` defines equalities and inequalities;
`EConstraintManifold` guarantees an empty inequality collection. Equality-only
problems can therefore use CMC, while equality-only optimizer entry points take
`ECMPoints`. A tangent increment remains an ordinary `gtsam::Vector`.

## Directory layout

| Directory | Code |
| --- | --- |
| `constrained` | `ConstraintManifold`, `CMPoint`, `CMTangentSpace`, shared basis/metric/Jacobian operations, tangent-cone geometry, and `CMCSubstituteFactor` |
| `constrained/internal` | Read-only basis views, independent-component scheduling, and the shared compiled-chart implementation |
| `cmopt` | `EConstraintManifold`, `ECMPoint`, equality retractors, and equality optimizer backends |
| `cmopt/internal` | The compiled equality-only adapter |
| `cmcopt` | `IEConstraintManifold`, `IECMPoint`, corner-aware retractors, and IE optimizer/QP backends |
| `cmcopt/internal` | The compiled IE adapter |

Common point operations are implemented in `CMPoint.cpp`; default recovery is
implemented in `ConstraintManifold.cpp`. The equality point owns its assignment
validation, so the common point implementation does not depend on either
concrete point category. Existing class names and numerical algorithms are
unchanged by this directory organization.

The directory move is validated by 32 native suites, 25 Python tests, three
selected ProxQP-disabled suites, and seven example builds. A temporary install
also compiles the common/E/IE headers without source-tree include paths, covering
the new public and internal header installation locations.

| Object | Owns |
| --- | --- |
| Runtime type | Local definitions, slot types/dimensions, immutable conditioning, symbolic equality structure, constant affine equality geometry, and retraction policy |
| `CMPoint` base | Runtime type identity, ambient-key bindings, numerical ambient values, and shared geometry |
| `IECMPoint` / `ECMPoint` | Concrete GTSAM value contract; IE points additionally expose active boundary blocks |
| `CMTangentSpace` | Read-only equality basis, raw/whitened active rows, cone projection, and point-local metric caches |

The type contains the operations for constructing the geometry and retracting a
point. It does not contain the current configuration, a current active set, or a
shared mutable projection. Factor graphs and the robot model supplied to a type
must remain immutable, as in ordinary GTSAM factor-graph construction.

## Constructing types and points

Factor keys name local slots. They need not equal trajectory keys, and a shared
type can bind those slots to different slices:

```cpp
auto type = std::make_shared<IEConstraintManifold>(equalities, inequalities, prototype,
                                      settings);
auto first = type->point({{0, x17}, {1, y17}}, firstValues);
auto second = type->point({{0, x18}, {1, y18}}, secondValues);
auto geometry = type->linearize(first);
gtsam::Vector delta = ...;
auto next = first.retract(delta);
```

The prototype supplies only value types and dimensions; its numerical state is
not retained. Use `EConstraintManifold(equalities, prototype, settings)`
when the absence of inequalities is part of the public contract. General IE
definitions may also have an empty inequality collection. Bindings must be
complete and one-to-one, and a point must contain exactly the expected ambient
values with matching types and dimensions. A point retains its type and bindings
when retracted. Chart comparisons require the same type identity and bindings;
points from differently conditioned spaces cannot be compared accidentally.

Types are constructed with `std::make_shared` and held through
`std::shared_ptr<const IEConstraintManifold>`. Create values with `type->point(...)`;
`IECMPoint` never constructs a type. Its default constructor exists only for GTSAM
value-container compatibility. Copies of points share immutable geometry.
Each concrete point has its own GTSAM traits and nonvirtual chart methods:
`ECMPoint::retract(Vector)` returns an `ECMPoint`, while the IE version returns
an `IECMPoint`. The common `CMPoint` base is used by reference, never stored
as a GTSAM value.
`IEConstraintManifold::linearize` constructs geometry without changing the type or point;
`IECMPoint::tangentSpace` accesses the cached geometry for that value.

The [small manual example](../../examples/example_constraint_manifold/main_shared_ieconstraint_manifold.cpp)
constructs two differently bound points on y=2x, x>=0, shares one definition,
and compares its coupled solve with automatic discovery. All existing robot
examples retain automatic discovery. Automatic structural type matching is not
implemented.

## Point keys and cost factors

`IEOptimizer::identifyConnectedComponents` returns `Components{types, points}`.
The type collection contains runtime definitions; every point references its own
definition. The collections have no index or size correspondence contract, so
future discovery can share equivalent types across many points. Today discovery
still creates one definition per connected component.

Discovery allocates component-point keys independently of ambient keys, using
`Symbol('c', index)` and skipping occupied keys.
`optimize` validates explicit keys and disjoint ambient ownership before
building the transformed graph. Unconstrained variables keep their existing
keys. Ambient-to-point ownership and slot bindings are explicit.

CMC substitution factors recover ambient values and Jacobians directly from
`IECMPoint`, including zero-dimensional points. GTSAM Values hold these points
through their dynamic-dimensional manifold traits. Equality optimization stores `ECMPoint`
values instead, using the same substitution implementation with its equality
point category. GTSAM values require the exact concrete value type; inheritance
does not permit `values.at<IECMPoint>(key)` when that key stores an `ECMPoint`.

G1's temporal ordering reads the slice from ambient bindings. Component-point
keys have no time encoding. Raw QP captures therefore use the new point keys;
recovery into ambient coordinates is unchanged.

## Conditioning and retraction

`type->condition(fixedLocalValues)` creates a distinct immutable type. It shares
the constraint definition, captures fixed numerical parameters, removes fixed
slots from the free layout, and chooses a fresh general equality chart. Fully
fixed equalities must be satisfied; remaining inequality block indices are
preserved. Original bindings and points remain valid and unchanged.

<a id="tangent-basis"></a>

Equality preparation belongs to the type. `TangentSpaceBasisCreator::prepare`
uses only local slot dimensions and constraints to build variable layout,
independent-coordinate selection, and symbolic ordering. It performs no numerical
linearization and retains no sample point values. Equality-only and CMC
optimizers share this preparation and the same numerical basis algorithms.

Each nonlinear point constructs its numerical basis from that prepared structure.
Under the explicit Euclidean `affineEqualities` contract, the type instead builds
its local numerical basis once on first use, using `std::call_once`. The fully
constructed basis is published read-only; a failed attempt can be retried. Ambient
bindings only wrap this local geometry. `ReadOnlyBasis` prevents mutable basis
interfaces from changing geometry shared by other points; this is a shared
geometry safeguard, not an adapter to the removed value implementation.

Every conditioning gets fresh preparation and its own affine cache. Conditioning
does not infer affinity: q*x-y=0 at different fixed q defines different spaces,
and matching structure alone never permits sharing numerical bases. Affine
offsets remain in the type's constraint definitions. Objective weights, LM
damping, and retraction metrics never enter equality preparation or invalidate
constant equality geometry.

The type selects the immutable retraction policy and captures binding-independent
preparation, including articulated robot topology validation. Every
`type->point(bindings, values, active)` construction rebuilds bound factor views
and binding-specific retraction preparation; there is no previous-point argument
or `IECMPoint::createWithNewValues`. Active boundary geometry and projection metric
caches are fresh per point. Repeated metric queries on the same point can reuse
matching weights. Every retraction call clones mutable work, including
fixed-variable factors and metric parameters. Different points sharing a type can retract in
parallel. Trial-dependent metric sigmas are passed explicitly through the CMC
retraction overload; changing an original parameter pointer does not change an
existing type. An LM-derived metric is applied only to types requesting a
varying projection metric, so a different type's fixed metric remains its own.
Metric maps are scoped to each point's ambient variables. Returned metric objects remain valid when a different metric is
computed later.

Articulated retraction still recovers q/v with FK, conditions the mechanics on
those trial values, and projects accelerations/wrenches/torques on that temporary
affine problem. That numerical projection is never shared across different q/v
values.

## Compiled types and GTSAM

The internal [shared compiled chart](internal/CompiledChart.h) accepts a type T
satisfying `gtsam::traits<T>` plus an immutable recovery object providing
`fromAmbient`, `ambient`, recovery `jacobians`, inequality `boundaries`, and
explicit `boundary` movement. The native type owns Local and Retract; no
constraint factors are fabricated. The [IE adapter](../cmcopt/internal/CompiledIEConstraintManifold.h)
adds native boundary operations. The [equality adapter](../cmopt/internal/CompiledEConstraintManifold.h),
`CompiledEConstraintManifold<T, Recovery>`, needs only recovery/Jacobians and
ordinary GTSAM traits. A generic compiled IE chart cannot be relaxed through
`equalityView()` because its boundary-free chart and recovery policy cannot be
inferred from constraint factors; that operation rejects the conversion. Supply
an explicit `CompiledEConstraintManifold` recovery policy instead. Conditioning
for a native type is expressed
through its immutable recovery parameters.

The nonnegative-line test implements an ordinary compiled GTSAM value and runs
it through the same CMC optimizer as its factor-defined counterpart. Both use
ordinary vector tangents. This is the extension path for replacing selected
runtime definitions with C++ implementations without changing the optimizer.

Manifold-interface compatibility does not teach an unconstrained GTSAM optimizer
about corners. A boundary projection need not invert an outward tangent vector.
CMC's tangent-cone QP controls admissible directions; chart identities are checked
in the interior and along admissible boundary directions.

## Migration and solver behavior

The old equality-only `ConstraintManifold` value class, `EManifoldValues`,
duplicate `SubstituteFactor`, and `LegacyCMAdapter` are removed.
`ConstraintManifold` now names the abstract runtime type, while
`IEConstraintManifold` retains its existing runtime name and
`EConstraintManifold` adds the equality-only specialization. Point values use
the separate `IECMPoint` and `ECMPoint` classes; no aliases retain the old value
representation.

- Use `EConstraintManifold` / `ECMPoint` / `ECMPoints` for equality-only APIs;
  use `IEConstraintManifold` / `IECMPoint` / `IECMPoints` for the general CMC API.
- `ManifoldOptimizerParameters::constraintManifoldParams` holds
  `EConstraintManifold::Params`. Rename `basisCreator` to
  `equalityBasisCreator`; `retractorCreator` captures the selected policy.
  Python exposes `EConstraintManifoldParams` and the same field names.
- Construct types with constraint factors and slot prototypes, then construct
  points only with `type->point(bindings, values, active)` or identity-binding
  `type->point(values, active)`. Point constructors never create runtime types.
- Existing equality `BasisRetractorCreator`, `ProjectionRetractorCreator`,
  `UnconstrainedOptimizationRetractorCreator`, and `DynamicsRetractorCreator`
  implement the shared `IERetractorCreator` interface. Configure their original
  `RetractionParams` before constructing the type. Policies snapshot parameters
  and isolate mutable workspaces; approximate CM(I) budgets retain their meanings.
  Explicit retraction LM orderings use local slots, are bound independently for
  each point, and are restricted after conditioning and for each inner graph.
  `ProjectionRetractorCreator` with `useBasisKeys=true` uses the prepared
  elimination basis's bound coordinate keys. A positive-dimensional basis
  without variable-key coordinates rejects that option explicitly.
- `ManifoldOptimizationProblem.types` holds definitions; `values` stores
  concrete `ECMPoint` values and free variables; `fixedPoints` holds
  zero-dimensional points. Replace `components` with `types` when counting
  discovered definitions. `points()` and `constPoints()` return typed maps.
- `NonlinearManifoldOptimizer` keeps GTSAM LM, Gauss–Newton, and Dogleg;
  `LMManifoldOptimizer` retains its own LM loop. CMC keeps `IELMOptimizer` and
  `IEGDOptimizer`. Their damping, acceptance, stopping, and callback contracts
  are preserved. The custom LM loops remain separate.

Equality points widen explicitly with `widenPoints(equalityPoints)`, or through
CMC's `optimize(costs, ECMPoints, freeValues)` overload. This preserves numerical
values, runtime type identity, and geometry. Once inserted in general CMC
`gtsam::Values`, the concrete stored value is `IECMPoint`; retrieve that exact
value type. Python adds `Values.insert(key, point)`, `atECMPoint(key)`, and
`atIECMPoint(key)` with the same exact-value rule. To relax inequalities, call `ieType->equalityView()` or
`iePoint.equalityView()`: the operation creates a distinct equality-only runtime
type and an `ECMPoint`. If the runtime type is already equality-only, the view
preserves its existing identity, including compiled equality charts. Ordinary
base-class conversion never removes constraints. Python `point.values()` returns
an independent snapshot so mutation cannot invalidate a point's cached geometry.
Runtime definitions cannot be assigned through base references. Assigning through
an IE point reference to a concrete `ECMPoint` checks the equality-only type
guarantee before changing its state.

Requested initial feasibility projection runs before the first numerical basis
construction through shared discovery. Optional final projection and approximate
retraction remain available. Custom basis/retraction policies must implement the
shared cloning contract so configuration and mutable work are separated.

The global tangent-cone QP, GTSAM/ProxQP backends, LM damping, active-block
promotion, nonlinear boundary handling, callbacks, and history behavior retain
their algorithms. See [component coordinates](../cmcopt/ComponentCoordinates.md) for the
linear-step contract. Manifold traits alone do not make unconstrained GTSAM
optimizers aware of corners.

## Validation of the typed migration

The final Release/TBB build passes 30 selected native suites with ProxQP and
all 29 available without ProxQP; the additional suite in the enabled build is
the optional G1 gauge test. Coverage includes every retained optimizer, affine
and nonlinear equalities, concrete GTSAM value casts, compiled charts,
zero-dimensional and coupled points, free variables, callbacks, all equality
retraction policies, CM(I), initial/final projection, and concurrent workspaces.
New regressions also cover assignment through point base references, immutable
type definitions, bound explicit orderings, conditioning of inner solve graphs,
and projection with prepared basis keys.

All 25 affected Python tests pass, including G1 integration, concrete point
insertion/extraction in `gtsam::Values`, callbacks, and independent `values()`
snapshots. A further compatibility check runs those enabled Python modules
against the disabled runtime library: the library rejects ProxQP explicitly,
and the optional ProxQP branch is then gated off in that test process. This is
not a separately built disabled Python wrapper. Seven enabled example
executables link, and eleven additional optional source checks pass.

The three-repeat bounded affine/nonlinear G1 comparison against committed
baseline `6aea8ba1` preserves objectives and state arrays exactly for every
matched case. No median slowdown exceeds 5%; see the
[timings and reproduction commands](../cmopt/EqualityOnlyOptimization.md#typed-migration-comparison).
No other initializer, full-trajectory optimization, or app recording was changed.

## Historical validation records

The records below are preserved from the pre-migration baseline commit
`6aea8ba1`. Their test names and timing scopes describe those earlier versions;
they are not claims that removed implementation tests still exist today.

## Validation of this refactor

The Release build uses TBB and the existing GTSAM QP branch. The following
18 enabled native suites pass: `testIEConstraintManifoldTypes`, `testIELMQp`,
`testIEConstraintManifold`, `testIEManifoldLinearization`,
`testIEManifoldOptimizer`, `testIEHalfSphere`, `testCMCImprovements`,
`testArticulatedRetractor`, `testAffineProjection`, `testReducedALRetractor`,
`testReducedALRetractionValidation`, `testSparseQpDependentRows`,
`testSubstituteFactor`, `testConstraintManifold`, `testManifoldOptimizer_rot2`,
`testRetractor`, `testFixedKinematicsDynamics`, and `testKinodynamicGauge`.
The new type/point suite contains 20 tests, including both QP backends when
ProxQP is enabled. A separate `GTDYNAMICS_WITH_PROXQP=OFF` build passes
`testIEConstraintManifoldTypes`, `testIELMQp`, and `testConstraintManifold`.

All seven G1 Python integration tests, the G1 full-graph factor self-test, and
the manual shared-type example pass. The 15 migrated legacy example scripts
compile. `testIERetractor.cpp` and `testIEQuadrupedUtilsManifold.cpp` remain
excluded by the pre-existing root CMake configuration; their migrated source
was checked with the test build's compiler flags using `-fsyntax-only`.
They are not counted as passing runtime suites.

### Bounded G1 comparison against PR 476

These are three fresh-process runs per version on the same 78–98 second clip,
with 171 support slices, the default `contact-gp-dynamics` initializer,
five LM iterations, eight retraction workers, CPUs 0–7, single-threaded BLAS,
and TBB enabled. The selected QP settings are GTSAM feasible steps, projected
initialization off, automatic ordering, and zero additional regularization.
The baseline is PR 476 head `caceae3945b69632796f3283a527bde1a1bb1ee9`.
No other initializer or full-trajectory benchmark was run for this comparison.

| Measure | PR 476 | Type/point refactor |
| --- | ---: | ---: |
| Median optimization time (s) | 7.692223 | 7.694217 |
| Median initialization + solve time (s) | 9.681305 | 9.702225 |
| Final objective | 9272.533829554453 | 9272.533829554563 |
| Equality violation L2 | 1.973559e-12 | 1.967818e-12 |
| Inequality violation L2 | 5.088856e-8 | 5.088856e-8 |
| Nonlinear feasibility | pass | pass |

Individual optimization times are `[7.687993, 7.692223, 7.700602]` seconds for
PR 476 and `[7.737123, 7.694217, 7.693625]` seconds for this refactor.
The median optimization difference is +0.026%; these measurements establish
no material regression or speedup. The largest absolute difference among the
compared pose, velocity, acceleration, wrench, and torque arrays is 2.07e-11.
The independent MuJoCo inverse-dynamics check passes on 21 samples; its largest
force/moment/torque disagreement is below 4.2e-12, swing wrenches remain zero,
and maximum sole-cone violation is 2.235e-8. This is a bounded five-iteration
comparison, not a convergence or full-trajectory performance claim.

Both versions load GTSAM revision
`c940b44158d8bfa7394da233dd63aa225bc8fed2` with library SHA-256
`49edc43dc92749353306c1c94ce7cdc0d951b719e06f611450aa6679c526940f`,
and identical system TBB/TBB allocator libraries. Input and model hashes,
settings, CPU affinity, source fingerprints, output arrays, and physics checks
are recorded with each run in the local ignored directories
`build/cmanifold-baseline` and `build/cmanifold-final`.
`build/cmanifold-final/comparison.json` contains the aggregate comparison.
The refactor build's source fingerprint is
`895659bd4433595b55a9b1cf2f37d3e7ed9c05284746e3d36d7c2ff9f94a124f`;
it covers tracked and new C++/CMake/wrapper sources, including the uncommitted
implementation, rather than identifying the candidate only by its base HEAD.


## Equality ownership follow-up

The comparison baseline includes the pending explicit type/point factory and
discovery changes. Both sides use the same bounded 78–98 s support-slice input,
default `contact-gp-dynamics` initializer, five LM iterations, eight TBB
retraction workers, CPUs 0–7, single-threaded BLAS, identical GTSAM binary,
feasible QP steps, projected-gradient initialization off, and automatic ordering.
Three fresh processes were run per version.

| Measurement | Before ownership correction | Type-owned equality geometry |
| --- | ---: | ---: |
| Median optimization time (s) | 7.696876 | 7.728266 |
| Median initialization + solve time (s) | 9.704914 | 9.739416 |
| Final objective | 9272.533829554563 | 9272.533829554563 |
| Equality violation L2 | 1.967818e-12 | 1.967818e-12 |
| Inequality violation L2 | 5.088856e-08 | 5.088856e-08 |

Median optimization time changed by **+0.408%**, below the 5% investigation
threshold. Objective, feasibility, and all eight exported numerical state arrays
are identical in this comparison. Every run was feasible and passed the same
21-sample MuJoCo validation. Rebuilding binding-specific preparation therefore
showed no material regression in this bounded test; no speedup is claimed.

The ownership checks include 20 type/point tests, 19 enabled native suites,
seven G1 Python tests, the full-graph factor self-test, and the manual example.
The ProxQP-disabled build passes the type/point, QP, and legacy CM suites.
Raw runs, source snapshots, library/input hashes, and the comparison are retained
in the ignored `build/equality-owner-baseline` and `build/equality-owner-final`
directories. No other initializer or full trajectory was benchmarked.
