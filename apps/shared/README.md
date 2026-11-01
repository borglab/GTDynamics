# Shared robot viewers

Use `RobotViewport.js` for new Three.js robot viewers. It owns the canvas,
Z-up camera, orbit controls, resizing, optional continuous animation, lighting
and stage setup, and resource cleanup. It does not load models or run physics.
`CameraController.js` provides model-independent framing and following.
Give the container a definite height and its canvas `width: 100%; height: 100%`.

```js
import { RobotViewport } from '../../shared/RobotViewport.js';
import { createRobotVisual, updateRobotBodies } from '../../shared/mujocoRobot.js';

const viewport = new RobotViewport(element);
viewport.addStage(); // lights, ground plane, grid; options customize appearance
const robot = createRobotVisual(model);
viewport.scene.add(robot.root);
viewport.cameraController.fit(worldBounds); // THREE.Box3, metres, Z-up

// Live MuJoCo: the app advances physics and supplies body poses.
viewport.start(now => {
  advanceSimulation(now);
  updateRobotBodies(robot.bodies, data);
});

// On unmount: stop animation, disconnect resize handling, release GPU resources.
viewport.dispose();
```

For **recorded playback**, omit `start()`. Apply saved world-space positions and
xyzw quaternions to `robot.bodies`, then call `viewport.render()`. Do not load
MuJoCo or recompute FK just to display already-computed link estimates. This
also preserves optimizer iterates whose link poses do not satisfy joint FK.

For **live FK or physics**, load `mujoco-js` in the app, map joints by name, call
`mj_kinematics`/`mj_forward` or `mj_step`, and use `updateRobotBodies`. The app owns
and deletes its MuJoCo model/data and controls its simulation timestep; the
viewport owns the display animation loop. The callback advances state before
controls and rendering update. Use `performance.now()` timestamps in milliseconds.

## Camera

- `cameraController.lookAt(target, offset)`: set a world target and metre-valued
  camera offset.
- `cameraController.follow(position, offset)`: start following a moving target.
- `cameraController.updateTarget(position)`: translate the camera as that target
  moves, preserving the user's orbit, pan, and zoom.
- `cameraController.fit(bounds, {view})`: fit a world `Box3`; named views are
  `perspective`, `top`, `front`, and `side`. Optional `direction`, `padding`, and
  `minDistance` customize framing. Fitting stops following and accounts for both
  horizontal and vertical field of view. Resize automatically refits the bounds.

The app owns paths, contact markers, picking, transform gizmos, and UI controls.
It can add them to `viewport.scene` and use `viewport.camera` and
`viewport.renderer.domElement`. Configure orbit limits with the constructor's
`controls` option. Continuous viewers can enable damping; replay viewers render
on data or control changes and need no idle animation loop.

## Geometry and ownership

`mujocoRobot.js` builds the real robot meshes from compiled MuJoCo geometry
arrays, whether those arrays come from the WASM model or offline JSON. Geometry
and physics models remain independent of the viewport.

`disposeObject3D` releases owned geometries, materials, and textures once per
call. A cached/shared geometry must set `geometry.userData.sharedMujoco = true`;
its cache/model owner releases it after all visual instances are gone. Clone
materials when instances need independent color or transparency. Dispose removed
app overlay groups when replacing them. Do not dispose shared mesh buffers from
an individual snapshot or viewer.

## Examples and checks

- `kinematics/scene.js`: saved body poses, follow/fit camera, reference and slip overlays.
- `../contact-initialization/src/scene.js`: MuJoCo FK with contact-graph overlays.
- `../motion-planning/src/viewer.js`: live physics, draggable targets, dark stage.

From `apps`, `npm run test:shared` checks camera fitting, following, resizing,
and resource ownership. Run the apps' browser checks after migration or display
changes, including both root and project-prefix URLs, portrait resizing,
playback, switching models/recordings, overlays, and leaving/reopening an app.
