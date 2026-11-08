# Runtime manifold types and their values

Authors: Yetong Zhang and Frank Dellaert.

`IEConstraintManifold` defines a space. `CMPoint` is a value in that space, and
`CMTangentSpace` contains the numerical geometry at one point. A tangent
increment is an ordinary `gtsam::Vector`.

`IEConstraintManifold` retains the existing public name for this PR and now
denotes the runtime definition; numerical point values use `CMPoint`.

| Object | Owns |
| --- | --- |
| `IEConstraintManifold` | Local constraint definitions, slot types/dimensions, immutable conditioning, symbolic equality structure, constant affine equality geometry, and retraction policy |
| `CMPoint` | Its runtime type, ambient-key bindings, numerical ambient values, active boundary blocks, and derived geometry |
| `CMTangentSpace` | A read-only view of the equality basis, raw and whitened active rows, cone projection, and point-local metric caches |

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
not retained. Empty inequalities define an equality-only space. Bindings must be
complete and one-to-one, and a point must contain exactly the expected ambient
values with matching types and dimensions. A point retains its type and bindings
when retracted. Chart comparisons require the same type identity and bindings;
points from differently conditioned spaces cannot be compared accidentally.

Types are constructed with `std::make_shared` and held through
`std::shared_ptr<const IEConstraintManifold>`. Create values with `type->point(...)`;
`CMPoint` never constructs a type. Its default constructor exists only for GTSAM
value-container compatibility. Copies of points share immutable geometry.
`IEConstraintManifold::linearize` constructs geometry without changing the type or point;
`CMPoint::tangentSpace` accesses the cached geometry for that value.

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
`CMPoint`, including zero-dimensional points. GTSAM Values hold these points
through their dynamic-dimensional manifold traits. Normal CMC linearization no
longer constructs equality-only CM value objects.

G1's temporal ordering reads the slice from ambient bindings. Component-point
keys have no time encoding. Raw QP captures therefore use the new point keys;
recovery into ambient coordinates is unchanged.

## Conditioning and retraction

`type->condition(fixedLocalValues)` creates a distinct immutable type. It shares
the constraint definition, captures fixed numerical parameters, removes fixed
slots from the free layout, and chooses a fresh general equality chart. Fully
fixed equalities must be satisfied; remaining inequality block indices are
preserved. Original bindings and points remain valid and unchanged.

Equality preparation belongs to the type. `TangentSpaceBasisCreator::prepare`
uses only local slot dimensions and constraints to build variable layout,
independent-coordinate selection, and symbolic ordering. It performs no numerical
linearization and retains no sample point values. Existing equality-only CM
`create` behavior remains unchanged.

Each nonlinear point constructs its numerical basis from that prepared structure.
Under the explicit Euclidean `affineEqualities` contract, the type instead builds
its local numerical basis once on first use, using `std::call_once`. The fully
constructed basis is published read-only; a failed attempt can be retried. Ambient
bindings only wrap this local geometry. A legacy read-only adapter prevents old
mutable basis interfaces from changing geometry shared by other points.

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
or `CMPoint::createWithNewValues`. Active boundary geometry and projection metric
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

The internal [compiled adapter](internal/CompiledIEConstraintManifold.h) accepts a type T
satisfying `gtsam::traits<T>` plus an immutable recovery object providing
`fromAmbient`, `ambient`, recovery `jacobians`, inequality `boundaries`, and
explicit `boundary` movement. The native type owns Local and Retract; no
constraint factors are fabricated. Conditioning for a native type is expressed
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

CMC value APIs formerly named `IEConstraintManifold` and `IEManifoldValues` become
`CMPoint` and `CMPoints`. Construction choices move to `IEConstraintManifold::Params`.
Use `identifyConnectedComponents` and `optimize`; iteration state exposes `points`
and trial updates expose `newPoints`.

Equality-only CM-Opt remains unchanged. The internal legacy adapter isolates its
old `ConstraintManifold` value class while downstream code migrates toward this
single type/point architecture. Custom CMC basis/retraction policies must
implement `clone()` to capture immutable configuration and detach mutable
workspace state; legacy CM-only policies do not need this new hook.
New CMC public interfaces do not require that
legacy value type.

The global tangent-cone QP, GTSAM/ProxQP backends, LM damping, active-block
promotion, nonlinear boundary handling, callbacks, and history behavior are
unchanged. See [component coordinates](ComponentCoordinates.md) for that solver
contract. This refactor does not promise a speedup or redesign the QP algorithms.


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
