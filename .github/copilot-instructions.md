# GTDynamics repository guidance

Follow these conventions when writing code and reviewing pull requests. Keep
changes focused and follow the surrounding code. Prefer small functions with
clear responsibilities over long functions or unnecessary abstractions.

## C++ style

### Naming conventions

Imported from GTSAM's `.github/copilot-instructions.md`, “C++ style” and
“Wrappers and vendored code”:

- Classes and types are `UpperCamelCase`; methods and functions are
  `lowerCamelCase`.
- Public fields in structs use plain names without a trailing underscore.
- Apart from those naming conventions, follow Google C++ style.
- When a C++ function is exposed in a wrapper `.i` file, parameter names must
  match exactly between the header declaration and the `.i` declaration.
  Update both whenever a wrapped parameter name changes.

Additional GTDynamics convention: new C++ parameters and local variables use
`lowerCamelCase`, for example `contactBuffer`. Preserve these names in wrappers
and Python keyword calls; Python-defined APIs still follow the Python style
below. Preserve established identifiers outside the scope of a change.

### Implementation style

- Use meaningful variable names, for example `measurement` rather than `msm`;
  avoid abbreviations. Use `pim` for a `PreintegratedImuMeasurementsG` instance
  and `pims` for a collection.
- Prefer concise examples and the fewest helpers needed. Favor direct
  construction and small local functors or lambdas over adapter functions.
- Keep functions focused. When reviewing overly complex or long functions,
  recommend breaking them into smaller functions.
- Prefer fixed-size Eigen types such as `Matrix3`, `Matrix23`, and `Vector6`
  when dimensions are known at compile time, especially for Jacobians. Use
  dynamic `Matrix` and `Vector` when dimensions vary or the API requires them.
  Write directly to fixed-size Jacobians or use fixed-size temporaries.
- Use flat initializer lists for vectors and nested lists for matrices.

### Public APIs and documentation

- Document functions declared in headers with Doxygen-style `/** */` comments.
  Small functions such as getters can use single-line `///` comments. There is
  no need for `@brief`, `@param`, or similar tags when the prose is already clear.
- Use `///` for single-line documentation rather than a one-line `/** */`.
- Document public types and fields where their meaning is not obvious. State
  coordinate frames, transform direction, units, index/key conventions, input
  requirements, defaults, return semantics, and relevant exceptions. For
  factors, document the residual, noise scale, and optional Jacobian dimensions.
- Explain gauges, observability limitations, hard constraints versus soft
  priors, and the exact definitions of reported costs and timing scopes. Keep
  implementation comments focused on the algorithm and non-obvious choices.
- Review Windows exports for public out-of-line classes and free functions;
  use the export macro belonging to the library that owns their definitions.
  Inline functions, header-only classes, and unspecialized templates normally
  compile in consumers and do not need a macro merely to force DLL export.

### C++ tests

Adapted from GTSAM’s `.github/copilot-instructions.md`, “C++ test organization”.

* Avoid one large anonymous namespace collecting every helper at the top of a
  test file. Prefer small, named fixture-style namespaces containing both the
  helpers and tests that use them.
* Put at least one short `//` comment immediately above every `TEST()` explaining
  the behavior being verified.
* When using `/* ************************************************************************* */`
  dividers, put one immediately before each fixture namespace and one
  immediately after it closes. Do not put dividers between tests in the same
  fixture namespace:

  ```cpp
  /* ************************************************************************* */
  namespace my_fixture {

  // Verifies the expected behavior.
  TEST(Suite, Case) {}

  }  // namespace my_fixture
  /* ************************************************************************* */
  ```

## Python style

Use PEP 8 naming: `snake_case` for functions, variables, and modules;
`UpperCamelCase` for classes; and `UPPER_SNAKE_CASE` for constants. Preserve
names imposed by wrapped C++ APIs, serialized formats, or other external
interfaces. These Python formatter settings are a GTDynamics convention;
GTSAM's current repository instructions do not prescribe Black or a line length.

- Use four spaces for indentation, descriptive names, and small functions.
- Group imports into standard library, third-party packages, and local modules,
  separated by blank lines. Black formats imports but does not sort them.
- Document public functions and classes with docstrings, including units,
  array shapes, coordinate frames, and return conventions when relevant.
- Use Black's normal string normalization and wrapping. Avoid manual alignment
  or personal formatting that fights the formatter. Wrap prose comments and
  docstrings manually when needed; long strings and URLs may remain unbroken.
- Use the `py312` conda environment for Python commands and formatter runs.

### Python wrappers

- The `.i` files use **gtwrap**, not SWIG. Follow the existing grammar; write
  inheritance as `class Derived : Base`, without `public`.
- Keep wrapped declarations synchronized with their C++ headers, including
  parameter names, const qualifiers, and pointer/reference types. Update both
  files when a wrapped signature changes.
- Ensure default arguments work through actual Python calls. For example,
  Eigen Jacobian pointer defaults may need an overload without Jacobian
  arguments to expose the scalar-only call.

## Formatters and line lengths

The agreed settings are checked into the repository root:

| Language | Formatter | Configuration | Line-length target | Indentation |
| --- | --- | --- | ---: | --- |
| C++ | clang-format, Google style | `.clang-format` | 80 columns | 2 spaces, no tabs |
| Python | Black | `pyproject.toml`, `[tool.black]` | 88 columns | 4 spaces |

Use the checked-in configuration rather than editor defaults or per-command
line-length overrides. The configuration is verified with clang-format 20.1.8
and Black 24.10.0. Format the first-party source files you change; avoid
repository-wide formatting sweeps, generated code/data, and vendored libraries.
In a shared legacy file, format the edited range when formatting the whole file
would create unrelated changes. Do not run clang-format on gtwrap `.i` files.

From the repository root, with `py312` activated:

```bash
clang-format --style=file -i path/to/file.cpp path/to/file.h
python -m black path/to/file.py
```

For checks without modifying files:

```bash
clang-format --style=file --dry-run --Werror path/to/file.cpp path/to/file.h
python -m black --check --diff path/to/file.py
```

Line lengths are formatter targets rather than a reason to split unbreakable
identifiers or URLs. Keep formatter changes separate from behavioral changes
when practical.

## Browser apps and robot visualization

- Put browser apps under `apps/`. Register new apps in `apps/registry.json` and
  the npm workspaces in `apps/package.json`, and use the shared navigation.
- **Start robot viewers with `apps/shared/RobotViewport.js` and
  `apps/shared/CameraController.js`.** See `apps/shared/README.md` for a minimal
  app and the camera/resource ownership contract. Reuse their renderer, stage,
  orbit controls, resize handling, animation loop, fitting, and following.
  Add app-specific overlays and picking to the shared scene; keep simulation,
  recording loading, optimization, and UI state in the app. Prefer composition
  over a new viewer hierarchy or copies of another app's constructor.
- Use real robot MJCF geometry, meshes, and materials from the recording's
  model or pinned MuJoCo Menagerie assets. Avoid approximate link silhouettes.
  **Load `mujoco-js` when the browser needs FK or physics.** For saved link-pose
  estimates, render those transforms directly without loading MuJoCo or
  recomputing FK. This preserves infeasible optimizer iterates and keeps static
  replay startup small. Three.js renders both modes and their overlays.
- Reuse `apps/shared/mujocoRobot.js` to build visuals from compiled MuJoCo
  geometry and update body transforms. See
  `apps/contact-initialization/src/mujocoG1.js` for configuration replay and
  `apps/motion-planning/src/viewer.js` for physics playback.
- Bootstrap model assets through the existing scripts and `npm run models`.
  Reuse pinned checkouts, keep downloaded meshes and generated manifests out
  of Git, retain model licenses and provenance, and include the actual assets
  in the production build.
- Keep MuJoCo's Z-up coordinate system and metre units. Convert quaternion
  order explicitly: MuJoCo uses `wxyz`, while Three.js uses `xyzw`. Map recorded
  joints by name to `jnt_qposadr`; do not assume recording and model indices
  match. Check rendered FK against recorded body and contact frames.
- Choose the playback mode deliberately. Use `mj_kinematics` or `mj_forward`
  for configuration replay; use `mj_step` when simulating dynamics. Apply the
  intended root pose and measured joint configuration consistently to the
  robot, snapshots, reference overlays, and contacts.
- Use `viewport.start(update)` for live physics and `viewport.render()` after
  recorded updates. The app owns simulation time and stepping; the viewport
  owns the display loop. Do not add a second renderer, resize observer, or idle
  animation loop. Call `viewport.dispose()` on unmount, and separately delete
  app-owned MuJoCo data/models and abort pending loads. Shared mesh geometry
  must be marked and disposed only by its model/cache owner.
- Keep native optimization and browser visualization distinct. Static demos
  replay recorded native results and timings; label recorded and live modes
  accurately. Recorded playback must work without a native backend.
- Publish only assets consumed by the registered apps, including selectable
  replay bundles and required attribution/license files. Keep raw sensor inputs
  and intermediate exports in ignored caches. Extend the public-asset inventory
  test when adding a consumer; avoid duplicate metadata encodings and obsolete
  recording versions in branch history.
- Store app recordings and compiled display geometry under the LFS rules in
  `.gitattributes`. Keep small manifests, source files, and license notices in
  ordinary Git. Hydrate app assets before tests/builds (`git lfs pull
  --include="apps/**" --exclude=""`); Pages checkout must retain `lfs: true`.
- Resolve app assets relative to `import.meta.env.BASE_URL` or the app document.
  Verify both `/` and `/GTDynamics/` deployment paths, including direct app URLs.
- Abort pending asset loads on unmount and release MuJoCo data/models, Three.js
  resources, listeners, and animation loops. Snapshot clones can share mesh
  geometry, but only its owner should dispose of those shared buffers.

## Local development and validation

- Use the `py312` conda environment for all Python commands, including scripts
  invoked by npm. Activate it before running `npm run models`.
- In this local development environment, link GTDynamics against
  `~/git/gtsam/build` through `GTSAM_DIR`. Rebuild that GTSAM checkout when needed
  and use its matching Python package.
- Run relevant C++ tests from the workspace's `build/` directory with
  `make -j6 testXXX.run`, using escalated permissions. Replace `testXXX` with
  the affected test target.
- For app changes, run the relevant npm tests and `npm run build` from `apps/`.
  Inspect the actual browser rendering and affected interactions. For viewer
  changes, check playback, model switching, overlays, and app unload/reload.
- Documentation-only changes need `git diff --check`, without unrelated builds
  or test runs. Report existing skips and disabled tests separately from passes.
- Run GitHub CLI commands with escalated permissions on macOS for Keychain and
  network access. Prefer narrow reusable approvals such as `gh pr`, `gh run`,
  and `gh api`. A sandboxed authentication failure is not a reason to log in again.

## Pull requests

Explain what failed, how it was fixed, and the resulting behavior. Use past
tense for failures resolved by the PR. Identify the most important files to
review when useful, and include relevant validation and remaining limitations.
