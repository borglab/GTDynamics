/**
 * Live-setup editor of the SCATE UR10e Base app: draws a start and a goal
 * "shadow" arm and the moving obstacles into the replay viewer's scene, and
 * lets the user drag them. Grab any link of the active arm to pull that point
 * (inverse kinematics over the arm's joints, kinematics.js); grab an obstacle
 * to move where it starts, or its end marker to move where it is at the end
 * of the run. Drags move in the plane facing the camera through the grabbed
 * point; the camera orbits as usual when nothing is grabbed.
 */
import * as THREE from 'three';
import { inversePoint, linkTransforms, solvePointIK, sphereCenters, transformPoint } from './kinematics.js';

/** Colors of the shadow arms and obstacle handles. */
export const EDITOR_COLORS = { start: '#2e9e5b', goal: '#7b5cd6', obstacle: '#d9534f', end: '#f0ad4e' };

/**
 * @param context viewer.editorContext()
 * @param options.onChange called with the edited setup after every drag step
 * @returns {setRobot, setSetup, setMode, setActivePose, setShowSpheres, dispose}
 */
export function createSetupEditor(context, { onChange } = {}) {
  const { scene, camera, controls, domElement, meshForLink, render } = context;
  const group = new THREE.Group();
  group.visible = false;
  scene.add(group);

  let robot = null, geometries = {}, setup = null;
  let mode = 'pose';           // 'pose': drag the arms; 'obstacles': drag the obstacles
  let activePose = 'start';    // the arm a pose drag moves
  let showSpheres = false;
  const arms = {}, obstacleObjects = [];
  const materials = {};

  function material(key, opacity) {
    const name = `${key}:${opacity}`;
    materials[name] ??= new THREE.MeshStandardMaterial({
      color: EDITOR_COLORS[key], roughness: 0.6, transparent: opacity < 1, opacity, depthWrite: opacity >= 1,
    });
    return materials[name];
  }
  const lineMaterial = new THREE.LineDashedMaterial({ color: '#8a6d3b', dashSize: 0.03, gapSize: 0.02 });
  const sphereGeometry = new THREE.SphereGeometry(1, 24, 16);

  function disposeObjects() {
    group.traverse(object => {
      if (object.geometry && object.geometry !== sphereGeometry && !object.userData.shared) object.geometry.dispose();
    });
    group.clear();
    for (const key of Object.keys(arms)) delete arms[key];
    obstacleObjects.length = 0;
  }

  // One shadow arm: a group per link holding its meshes (tagged with the link
  // index for picking), and its collision spheres.
  function buildArm(key) {
    const arm = { key, links: [], spheres: [], root: new THREE.Group() };
    robot.links.forEach((link, index) => {
      const holder = new THREE.Group();
      const geometry = geometries[meshForLink[link.name]];
      const parts = geometry?.parts ? geometry.parts.map(part => part.geometry) : geometry ? [geometry] : [];
      for (const part of parts) {
        const mesh = new THREE.Mesh(part, material(key, 0.45));
        mesh.userData = { shared: true, arm: key, link: index };
        holder.add(mesh);
      }
      holder.matrixAutoUpdate = false;
      arm.links.push(holder);
      arm.root.add(holder);
    });
    for (const sphere of robot.spheres) {
      const mesh = new THREE.Mesh(sphereGeometry, material(key, 0.22));
      mesh.scale.setScalar(sphere.radius);
      mesh.userData = { arm: key, link: sphere.link };
      arm.spheres.push(mesh);
      arm.root.add(mesh);
    }
    group.add(arm.root);
    return arm;
  }

  function buildObstacles() {
    setup.obstacles.forEach((obstacle, index) => {
      const body = new THREE.Mesh(sphereGeometry, material('obstacle', 0.55));
      body.userData = { obstacle: index, handle: 'start' };
      const end = new THREE.Mesh(sphereGeometry, material('end', 0.9));
      end.userData = { obstacle: index, handle: 'end' };
      const ghost = new THREE.Mesh(sphereGeometry, material('end', 0.15));
      ghost.raycast = () => {};
      const lineGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
      const line = new THREE.Line(lineGeometry, lineMaterial);
      group.add(body, end, ghost, line);
      obstacleObjects.push({ body, end, ghost, line });
    });
  }

  const matrixOf = m => new THREE.Matrix4().set(...m);

  function poseArm(arm, q) {
    const transforms = linkTransforms(robot, q);
    arm.links.forEach((holder, l) => { holder.matrix.copy(matrixOf(transforms[l])); holder.matrixWorldNeedsUpdate = true; });
    sphereCenters(robot, q, transforms).forEach((sphere, s) => arm.spheres[s].position.set(...sphere.center));
  }

  function update() {
    if (!setup || !robot) return;
    for (const key of ['start', 'goal']) {
      const arm = arms[key];
      poseArm(arm, setup[key === 'start' ? 'q_start' : 'q_goal']);
      const active = mode === 'pose' && activePose === key;
      const opacity = mode === 'pose' ? (active ? 0.75 : 0.25) : (key === 'start' ? 0.45 : 0.18);
      arm.links.forEach(holder => holder.children.forEach(mesh => { mesh.material = material(key, opacity); }));
      arm.spheres.forEach(mesh => { mesh.visible = showSpheres; });
    }
    setup.obstacles.forEach((obstacle, index) => {
      const objects = obstacleObjects[index];
      objects.body.position.set(...obstacle.start);
      objects.body.scale.setScalar(obstacle.radius);
      objects.ghost.position.set(...obstacle.end);
      objects.ghost.scale.setScalar(obstacle.radius);
      objects.end.position.set(...obstacle.end);
      objects.end.scale.setScalar(Math.min(0.03, obstacle.radius * 0.5));
      objects.line.geometry.setFromPoints([new THREE.Vector3(...obstacle.start), new THREE.Vector3(...obstacle.end)]);
      objects.line.computeLineDistances();
      const editable = mode === 'obstacles';
      objects.body.material = material('obstacle', editable ? 0.7 : 0.45);
    });
    render();
  }

  // ---------- dragging ----------
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let drag = null;

  function rayFrom(event) {
    const box = domElement.getBoundingClientRect();
    pointer.set(((event.clientX - box.left) / box.width) * 2 - 1, -((event.clientY - box.top) / box.height) * 2 + 1);
    raycaster.setFromCamera(pointer, camera);
    return raycaster;
  }

  // What the pointer is over: an obstacle handle, or a link of the active arm.
  function pick(event) {
    if (!group.visible || !setup) return null;
    const ray = rayFrom(event);
    const targets = [];
    if (mode === 'obstacles') obstacleObjects.forEach(objects => targets.push(objects.end, objects.body));
    if (mode === 'pose') {
      const arm = arms[activePose];
      arm.links.forEach((holder, l) => { if (l > 0) holder.children.forEach(mesh => targets.push(mesh)); });
      arm.spheres.forEach((mesh, s) => { if (showSpheres && robot.spheres[s].link > 0) targets.push(mesh); });
    }
    group.updateMatrixWorld(true);
    const hit = ray.intersectObjects(targets, false)[0];
    return hit ? { object: hit.object, point: hit.point.clone() } : null;
  }

  function startDrag(event) {
    const hit = pick(event);
    if (!hit) return;
    event.preventDefault();
    event.stopImmediatePropagation();
    const normal = camera.getWorldDirection(new THREE.Vector3());
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, hit.point);
    const data = hit.object.userData;
    if (data.obstacle !== undefined) {
      const center = new THREE.Vector3(...setup.obstacles[data.obstacle][data.handle]);
      drag = { kind: 'obstacle', index: data.obstacle, handle: data.handle, plane, offset: center.sub(hit.point) };
    } else {
      const key = activePose === 'start' ? 'q_start' : 'q_goal';
      const transform = linkTransforms(robot, setup[key])[data.link];
      drag = { kind: 'arm', key, link: data.link, pointInLink: inversePoint(transform, hit.point.toArray()), plane };
    }
    controls.enabled = false;
    domElement.setPointerCapture?.(event.pointerId);
    domElement.style.cursor = 'grabbing';
  }

  const clampToWorkspace = (point, radius) => point.map((value, axis) =>
    Math.min(robot.workspace.max[axis] - radius, Math.max(robot.workspace.min[axis] + radius, value)));

  function moveDrag(event) {
    if (!drag) {
      domElement.style.cursor = pick(event) ? 'grab' : '';
      return;
    }
    const target = rayFrom(event).ray.intersectPlane(drag.plane, new THREE.Vector3());
    if (!target) return;
    if (drag.kind === 'obstacle') {
      const obstacle = setup.obstacles[drag.index];
      obstacle[drag.handle] = clampToWorkspace(target.add(drag.offset).toArray(), obstacle.radius);
    } else {
      setup[drag.key] = solvePointIK(robot, setup[drag.key], drag.link, drag.pointInLink, target.toArray(), { iterations: 12 });
    }
    update();
    onChange?.(setup);
  }

  function endDrag(event) {
    if (!drag) return;
    drag = null;
    controls.enabled = true;
    domElement.releasePointerCapture?.(event.pointerId);
    domElement.style.cursor = '';
    onChange?.(setup);
  }

  const events = new AbortController();
  // Capture phase, so a grab starts before the orbit controls see the pointer.
  domElement.addEventListener('pointerdown', startDrag, { capture: true, signal: events.signal });
  domElement.addEventListener('pointermove', moveDrag, { signal: events.signal });
  domElement.addEventListener('pointerup', endDrag, { signal: events.signal });
  domElement.addEventListener('pointercancel', endDrag, { signal: events.signal });

  return {
    /** Show the editor for a robot description and its meshes (by mesh name). */
    setRobot(description, meshes) {
      disposeObjects();
      robot = description; geometries = meshes;
      if (setup) { arms.start = buildArm('start'); arms.goal = buildArm('goal'); buildObstacles(); update(); }
    },
    /** The setup to draw and edit (edited in place): {q_start, q_goal, obstacles: [{start, end, radius}]}. */
    setSetup(next) {
      setup = next;
      if (!robot) return;
      disposeObjects();
      arms.start = buildArm('start'); arms.goal = buildArm('goal'); buildObstacles();
      update();
    },
    /** Redraw after the page changed the setup (sliders, radius). */
    refresh() { update(); },
    /** 'pose' (drag the arms), 'obstacles' (drag the obstacles) or null (hidden). */
    setMode(next) {
      mode = next;
      group.visible = !!next;
      if (!next) { drag = null; controls.enabled = true; domElement.style.cursor = ''; render(); return; }
      update();
    },
    /** Which arm a pose drag moves: 'start' or 'goal'. */
    setActivePose(key) { activePose = key; update(); },
    /** Draw the shadow arms' collision spheres (also grabbable). */
    setShowSpheres(show) { showSpheres = show; update(); },
    /** World position of a sphere of an arm, for tests and readouts. */
    toolPoint(key) {
      const tool = robot.spheres[robot.tool_sphere];
      return transformPoint(linkTransforms(robot, setup[key])[tool.link], tool.center);
    },
    dispose() {
      events.abort();
      disposeObjects();
      scene.remove(group);
      sphereGeometry.dispose();
      lineMaterial.dispose();
      Object.values(materials).forEach(m => m.dispose());
      controls.enabled = true;
    },
  };
}
