/**
 * URDF IK Explorer: the three.js scene and side-panel logic for the browser
 * worker and local GTDynamics backend. App.vue owns the markup.
 */
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';
import { api, latestOnly } from './api.js';
import { createBrowserBackend } from './browser-backend.js';
import { g1Path, loadG1Visual } from './g1-visual.js';
import {
  POSE_FIELDS, availableSchemes, endEffectorLabel, homeConfiguration, quaternionFromRpyDeg,
  rpyDegFromQuaternion, sliderRange, solveSummary,
} from './pose.js';
import { RobotViewport, disposeObject3D } from '../../shared/RobotViewport.js';
import { getTheme, onThemeChange } from '../../shared/theme.js';

const $ = (id) => document.getElementById(id);

function homeJoints(model, joints) {
  const q = homeConfiguration(joints);
  if (model === g1Path) {
    // A small knee bend gives the sole Jacobian a vertical motion direction.
    for (const side of ['left', 'right']) {
      q[`${side}_hip_pitch_joint`] = -0.15;
      q[`${side}_knee_joint`] = 0.30;
      q[`${side}_ankle_pitch_joint`] = -0.15;
    }
  }
  return q;
}

/**
 * Builds the scene inside #viewport and wires the controls rendered by App.vue.
 * hooks.backend enables the local Python probe on a development server.
 * hooks.onMode reports the selected solver before model loading begins.
 * Returns { dispose }.
 */
export function createExplorer(hooks = {}) {
  const lifetime = new AbortController();
  let modelLoad;
  let browserBackend;
  let schemes = [];
  const request = (path, body, signal = lifetime.signal) =>
    browserBackend ? browserBackend.request(path, body, signal) :
      api(path, body, (url, init) => fetch(url, { ...init, signal }));
  const state = {
    generation: 0,      // bumped on every model / end-effector change
    modelPath: null,
    urdfPrefix: 'files/urdfs/',
    info: null,         // /api/load response
    ee: null,
    q: {},              // joint name -> value
    eePose: null,       // achieved end-effector pose from the server
    robot: null,        // { root, update(configuration) }
    configuration: null,
    scale: 1,           // characteristic robot size in meters
    editing: false,
    missingMeshes: new Set(),
    visualError: '',
  };

  // ------------------------------------------------------------------ scene

  const viewport = new RobotViewport($('viewport'), {
    fov: 45, near: 0.005, far: 500,
    controls: { minDistance: 0.05, maxPolarAngle: Math.PI },
    ariaLabel: 'Robot viewer. Drag the orange target to move it. Double-click a link to select it as the end effector.',
  });
  const { scene, camera, renderer, controls: orbit, cameraController } = viewport;
  const { grid } = viewport.addStage({
    hemisphere: [0xf0f3ff, 0x40362c, 1.6],
    key: { color: 0xffffff, intensity: 1.6, position: [3, -2, 5] },
    fill: { color: 0xffffff, intensity: 0.5, position: [-3, 4, 2] },
    ground: null,
    grid: { size: 20, divisions: 100, colors: [0xb5c2b7, 0xd6ddd3] },
  });
  const gridHeight = grid.position.z;
  scene.add(new THREE.AxesHelper(0.3));
  cameraController.lookAt(new THREE.Vector3(0, 0, 0.5), new THREE.Vector3(2, -2.5, 1.1));

  function overlayAxes(length, opacity) {
    const axes = new THREE.AxesHelper(length);
    axes.material.depthTest = false;
    axes.material.transparent = true;
    axes.material.opacity = opacity;
    axes.renderOrder = 20;
    return axes;
  }

  // Target: the pose the user asks for (draggable handle + faded axes).
  const target = new THREE.Object3D();
  scene.add(target);
  const handleMaterial = new THREE.MeshStandardMaterial({
    color: 0xff8a1f, emissive: 0x803000, transparent: true, opacity: 0.85, depthTest: false,
  });
  const handle = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 16), handleMaterial);
  handle.renderOrder = 21;
  target.add(handle);
  const targetAxes = overlayAxes(1, 0.45);
  target.add(targetAxes);

  // Achieved end-effector pose (solid axes).
  const eeMarker = new THREE.Object3D();
  eeMarker.add(overlayAxes(1, 1));
  scene.add(eeMarker);

  // Red line from the achieved pose to an unreachable target.
  const gapLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineDashedMaterial({ color: 0xff5a52, dashSize: 0.02, gapSize: 0.015, depthTest: false }),
  );
  gapLine.renderOrder = 19;
  gapLine.visible = false;
  scene.add(gapLine);

  // Stick-figure skeleton from GTDynamics' link frames.
  const skeleton = new THREE.Group();
  const skeletonLines = new THREE.LineSegments(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ transparent: true, opacity: 0.8, depthTest: false }),
  );
  const skeletonPoints = new THREE.Points(
    new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ size: 6, sizeAttenuation: false, depthTest: false }),
  );
  skeletonLines.renderOrder = skeletonPoints.renderOrder = 18;
  skeleton.add(skeletonLines, skeletonPoints);
  scene.add(skeleton);
  handle.visible = eeMarker.visible = skeleton.visible = false;

  function updateSkeletonColor() {
    const bone = getTheme() === 'dark' ? 0x5ad1ff : 0x1479b8;
    skeletonLines.material.color.set(bone);
    skeletonPoints.material.color.set(bone);
    viewport.render();
  }
  updateSkeletonColor();
  const stopThemeSync = onThemeChange(updateSkeletonColor);

  const gizmo = new TransformControls(camera, renderer.domElement);
  gizmo.setSpace('local');
  gizmo.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; });
  gizmo.addEventListener('objectChange', () => onTargetMoved());
  gizmo.addEventListener('change', () => viewport.render());
  scene.add(gizmo);

  function setMarkerScale() {
    const s = state.scale;
    handle.scale.setScalar(0.035 * s);
    targetAxes.scale.setScalar(0.18 * s);
    eeMarker.children[0].scale.setScalar(0.14 * s);
    gizmo.setSize(0.9);
    gapLine.material.dashSize = 0.02 * s;
    gapLine.material.gapSize = 0.015 * s;
  }

  // ------------------------------------------------------------ server calls

  const requestIK = latestOnly(async (gen) => {
    if (gen !== state.generation || !state.info) return;
    const body = {
      model: state.modelPath,
      ee: state.ee,
      scheme: $('scheme').value,
      q: state.q,
      target: targetPoseJson(),
      orientation: $('orientation').checked,
      pos_tol: Number($('pos-tol').value) / 1000,
      rot_tol_deg: Number($('rot-tol').value),
      max_iters: Number($('max-iters').value) || null,
    };
    try {
      const res = await request('api/ik', body);
      if (gen === state.generation) applySolve(res, body.orientation);
    } catch (err) {
      if (gen === state.generation && !lifetime.signal.aborted) showError(err.message);
    }
  });

  const requestFK = latestOnly(async (gen) => {
    if (gen !== state.generation || !state.info) return;
    try {
      const res = await request('api/fk', { model: state.modelPath, ee: state.ee, q: state.q });
      if (gen !== state.generation) return;
      applyConfiguration(res);
      setTargetToCurrent();
      setStatus('idle', 'Forward kinematics', '');
    } catch (err) {
      if (gen === state.generation && !lifetime.signal.aborted) showError(err.message);
    }
  });

  // ------------------------------------------------------------ pose helpers

  function poseToObject(pose, obj) {
    obj.position.fromArray(pose.position);
    obj.quaternion.fromArray(pose.quat_xyzw);
  }

  function targetPoseJson() {
    target.updateMatrixWorld();
    return { position: target.position.toArray(), quat_xyzw: target.quaternion.toArray() };
  }

  function setTargetToCurrent() {
    if (!state.eePose) return;
    poseToObject(state.eePose, target);
    setSolveFeedback(false);
    updatePoseTable();
  }

  function onTargetMoved() {
    updatePoseTable();
    viewport.render();
    requestIK(state.generation);
  }

  // ------------------------------------------------------------- applying

  function applyConfiguration(res) {
    state.configuration = res;
    state.q = res.q;
    state.eePose = res.ee_pose;
    state.robot?.update(res);
    poseToObject(res.ee_pose, eeMarker);
    updateSkeleton(res.skeleton);
    updateJointWidgets(res.at_limits || []);
    updatePoseTable();
  }

  function setSolveFeedback(failed) {
    handleMaterial.color.set(failed ? 0xff4b45 : 0xff8a1f);
    handleMaterial.emissive.set(failed ? 0x801010 : 0x803000);
    gapLine.visible = failed;
    if (failed) {
      gapLine.geometry.setFromPoints([eeMarker.position.clone(), target.position.clone()]);
      gapLine.computeLineDistances();
    }
  }

  function applySolve(res, orientation) {
    applyConfiguration(res);
    setSolveFeedback(res.status !== 'solved');
    const { cls, label, meta, items } = solveSummary(res, orientation);
    setStatus(cls, label, meta, items);
  }

  function updateSkeleton(nodes) {
    const byName = Object.fromEntries(nodes.map((n) => [n.link, n]));
    const segments = [];
    for (const n of nodes) {
      if (n.parent && byName[n.parent]) segments.push(...byName[n.parent].position, ...n.position);
    }
    skeletonLines.geometry.dispose();
    skeletonLines.geometry = new THREE.BufferGeometry();
    skeletonLines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(segments, 3));
    skeletonPoints.geometry.dispose();
    skeletonPoints.geometry = new THREE.BufferGeometry();
    skeletonPoints.geometry.setAttribute(
      'position', new THREE.Float32BufferAttribute(nodes.flatMap((n) => n.position), 3));
  }

  // ------------------------------------------------------------- side panel

  function setStatus(cls, label, meta, items = []) {
    $('status').className = `status ${cls}`;
    $('status-badge').textContent = label;
    $('status-meta').textContent = meta;
    $('result-summary').textContent = [label, meta].filter(Boolean).join(' · ');
    const list = $('reasons');
    list.replaceChildren(...items.map(({ text, kind }) => {
      const li = document.createElement('li');
      li.className = kind;
      li.textContent = text;
      return li;
    }));
    viewport.render();
  }

  function showError(message) {
    setStatus('error', 'Error', '', [{ text: message, kind: 'reason' }]);
  }

  let toastTimer = null;
  function toast(message) {
    const el = $('toast');
    el.textContent = message;
    el.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { el.hidden = true; }, 2600);
  }

  function buildPoseTable() {
    $('pose-rows').replaceChildren(...POSE_FIELDS.map(([name, unit], i) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `<th scope="row">${name} (${unit})</th><td class="cur" id="cur-${i}">–</td>` +
        `<td><input type="number" id="tgt-${i}" aria-label="Target ${name} (${unit})" step="${i < 3 ? 0.01 : 1}"></td>`;
      return tr;
    }));
    for (let i = 0; i < 6; i++) {
      $(`tgt-${i}`).addEventListener('change', applyPoseInputs);
    }
  }

  function updatePoseTable() {
    if (state.eePose) {
      const cur = [...state.eePose.position, ...state.eePose.rpy_deg];
      cur.forEach((v, i) => { $(`cur-${i}`).textContent = v.toFixed(i < 3 ? 4 : 2); });
      const [qx, qy, qz, qw] = state.eePose.quat_xyzw;
      $('quat').textContent =
        `current quaternion (x y z w): ${[qx, qy, qz, qw].map((v) => v.toFixed(4)).join(' ')}`;
    }
    const tgt = [...target.position.toArray(), ...rpyDegFromQuaternion(target.quaternion)];
    tgt.forEach((v, i) => {
      const input = $(`tgt-${i}`);
      if (document.activeElement !== input) input.value = v.toFixed(i < 3 ? 4 : 2);
    });
  }

  function applyPoseInputs() {
    const v = POSE_FIELDS.map((_, i) => Number($(`tgt-${i}`).value));
    if (v.some((x) => !Number.isFinite(x))) return;
    target.position.set(v[0], v[1], v[2]);
    target.quaternion.copy(quaternionFromRpyDeg(v.slice(3)));
    onTargetMoved();
  }

  function buildJointWidgets() {
    const container = $('joints');
    container.replaceChildren();
    for (const joint of state.info.joints) {
      const value = state.q[joint.name] ?? 0;
      const [lo, hi] = sliderRange(joint, value);
      const unit = joint.type === 'prismatic' ? 'm' : 'rad';
      const div = document.createElement('div');
      div.className = 'joint';
      div.dataset.name = joint.name;
      div.innerHTML = `
        <div class="joint-head"><span></span><span class="val"></span></div>
        <input type="range" min="${lo}" max="${hi}" step="${(hi - lo) / 2000}">
        <div class="limits"><span>${joint.lower === null ? '−∞' : lo.toFixed(2)}</span>` +
        `<span>${joint.type}, ${unit}</span>` +
        `<span>${joint.upper === null ? '+∞' : hi.toFixed(2)}</span></div>`;
      const slider = div.querySelector('input');
      div.querySelector('.joint-head span').textContent = joint.name;
      slider.setAttribute('aria-label', `${joint.name} (${unit})`);
      slider.addEventListener('input', () => {
        state.q = { ...state.q, [joint.name]: Number(slider.value) };
        requestFK(state.generation);
      });
      container.append(div);
    }
  }

  function updateJointWidgets(atLimits) {
    const limited = new Set(atLimits);
    for (const div of $('joints').children) {
      const name = div.dataset.name;
      const value = state.q[name] ?? 0;
      const slider = div.querySelector('input');
      if (document.activeElement !== slider) slider.value = value;
      div.querySelector('.val').textContent = value.toFixed(3);
      div.classList.toggle('at-limit', limited.has(name));
    }
  }

  function showNotes(notes) {
    const el = $('notes');
    el.replaceChildren(...notes.map((text) => {
      const p = document.createElement('p');
      p.textContent = text;
      return p;
    }));
    el.hidden = notes.length === 0;
  }

  function currentNotes() {
    const notes = [...(state.info?.notes || [])];
    if (state.info) notes.unshift(`Fixed base link: ${state.info.base}`);
    if (state.visualError) notes.push(state.visualError);
    if (state.missingMeshes.size) {
      notes.push(`${state.missingMeshes.size} mesh file(s) referenced by the URDF are not in ` +
        'the repository, so those links show only the skeleton.');
    }
    return notes;
  }

  // ------------------------------------------------------------ interaction

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let drag = null;

  function castFrom(event) {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.setFromCamera(pointer, camera);
  }

  function overHandle(event) {
    if (!handle.visible) return false;
    castFrom(event);
    return raycaster.intersectObject(handle, false).length > 0;
  }

  function pickedLink(event) {
    castFrom(event);
    const root = state.robot?.root;
    if (root?.visible) {
      const hit = raycaster.intersectObject(root, true).find(({ object }) => {
        if (!object.isMesh) return false;
        for (let part = object; part; part = part.parent) {
          if (!part.visible) return false;
        }
        return true;
      });
      if (hit) {
        for (let part = hit.object; part; part = part.parent) {
          if (part.userData.ikLink) return part.userData.ikLink;
        }
        return;
      }
    }
    if (skeleton.visible) {
      raycaster.params.Points.threshold = 0.015 * state.scale;
      const [hit] = raycaster.intersectObject(skeletonPoints, false);
      if (hit) return state.configuration.skeleton[hit.index].link;
    }
  }

  // Capture phase so a grab on the handle pre-empts OrbitControls.
  renderer.domElement.addEventListener('pointerdown', (event) => {
    if (event.button !== 0 || !state.info || (gizmo.object && gizmo.axis)) return;
    if (!overHandle(event)) return;
    event.stopImmediatePropagation();
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(
      camera.getWorldDirection(new THREE.Vector3()), target.position);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(plane, hit)) return;
    drag = { plane, offset: target.position.clone().sub(hit), moved: false };
    renderer.domElement.setPointerCapture(event.pointerId);
    renderer.domElement.style.cursor = 'grabbing';
  }, { capture: true });

  renderer.domElement.addEventListener('pointermove', (event) => {
    if (!drag) {
      renderer.domElement.style.cursor = overHandle(event) ? 'grab' : '';
      return;
    }
    castFrom(event);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(drag.plane, hit)) return;
    target.position.copy(hit.add(drag.offset));
    drag.moved = true;
    onTargetMoved();
  });

  function endDrag(event) {
    if (!drag) return;
    renderer.domElement.releasePointerCapture(event.pointerId);
    renderer.domElement.style.cursor = '';
    if (drag.moved) onTargetMoved();
    drag = null;
  }
  renderer.domElement.addEventListener('pointerup', endDrag);
  renderer.domElement.addEventListener('pointercancel', endDrag);

  renderer.domElement.addEventListener('dblclick', (event) => {
    if (overHandle(event)) {
      setEditing(!state.editing);
    } else if (state.info && !gizmo.axis) {
      const name = pickedLink(event);
      if (state.info.end_effectors.some(frame => frame.name === name && frame.dof > 0)) {
        changeEndEffector(name);
      }
    }
  });

  function setEditing(on) {
    state.editing = on && !!state.info;
    $('pose-section').classList.toggle('editing', state.editing);
    if (state.editing) {
      if (!$('orientation').checked) {
        setOrientation(true);
        toast('Orientation constraint enabled so the pose can be edited');
      }
      gizmo.attach(target);
      gizmo.setMode('rotate');
      toast('Editing pose: drag the rings to rotate, W to move, E to rotate, Esc to finish');
    } else {
      gizmo.detach();
    }
  }

  function onKeyDown(event) {
    if (!state.editing) return;
    if (event.key === 'Escape') {
      setEditing(false);
      return;
    }
    if (event.target.matches('input, select, textarea')) return;
    if (event.key === 'w' || event.key === 'W') gizmo.setMode('translate');
    else if (event.key === 'e' || event.key === 'E') gizmo.setMode('rotate');
  }
  window.addEventListener('keydown', onKeyDown, { signal: lifetime.signal });

  // --------------------------------------------------------------- loading

  async function loadMesh(path, manager, done, signal, missingMeshes) {
    const ext = path.split(/[?#]/)[0].split('.').pop().toLowerCase();
    manager.itemStart(path);
    try {
      const url = manager.resolveURL(path);
      const response = await fetch(url, { signal });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      if (ext === 'stl') {
        const data = await response.arrayBuffer();
        signal.throwIfAborted();
        // URDFLoader assigns its own material to a Mesh returned by this callback.
        const mesh = new THREE.Mesh(new STLLoader(manager).parse(data));
        mesh.material.dispose();
        done(mesh);
      } else {
        const data = await response.text();
        signal.throwIfAborted();
        if (ext === 'dae') done(new ColladaLoader(manager).parse(data, THREE.LoaderUtils.extractUrlBase(url)).scene);
        else if (ext === 'obj') done(new OBJLoader(manager).parse(data));
        else throw new Error(`No loader for .${ext} meshes`);
      }
    } catch {
      if (!signal.aborted) missingMeshes.add(path);
    } finally {
      manager.itemEnd(path);
    }
  }

  async function loadVisual(modelPath, signal) {
    if (modelPath === g1Path) return loadG1Visual(signal);
    const robot = await new Promise((resolve) => {
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      const missingMeshes = state.missingMeshes;
      loader.fetchOptions = { signal };
      let url = state.urdfPrefix + modelPath;
      if (browserBackend) {
        url = browserBackend.urdfUrl(modelPath);
        loader.workingPath = './';
        manager.setURLModifier(path => browserBackend.resolveAsset(modelPath, path));
      } else {
        loader.packages = (pkg) => `pkg/${pkg}`;
      }
      loader.loadMeshCb = (path, manager, done) => loadMesh(path, manager, done, signal, missingMeshes);
      let parsed = null;
      manager.onLoad = () => resolve(parsed);
      loader.load(url, (result) => { parsed = result; }, undefined, () => resolve(null));
    });
    if (!robot) return null;
    for (const joint of Object.values(robot.joints)) joint.ignoreLimits = true;
    for (const [name, link] of Object.entries(robot.links)) link.userData.ikLink = name;
    return { root: robot, update: ({ q }) => robot.setJointValues(q) };
  }

  function fitView() {
    const box = new THREE.Box3();
    if (state.robot) box.expandByObject(state.robot.root);
    const pos = skeletonPoints.geometry.getAttribute('position');
    if (pos) {
      for (let i = 0; i < pos.count; i++) box.expandByPoint(new THREE.Vector3().fromBufferAttribute(pos, i));
    }
    if (eeMarker.visible) box.expandByPoint(eeMarker.position);
    if (box.isEmpty()) return;
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.2);
    state.scale = Math.min(Math.max(size, 0.25), 20);
    setMarkerScale();
    cameraController.fit(box, { direction: new THREE.Vector3(0.75, -1, 0.55), minDistance: 0.25 });
    viewport.render();
  }

  function clearRobot() {
    if (state.robot) {
      scene.remove(state.robot.root);
      disposeObject3D(state.robot.root);
    }
    state.robot = null;
    state.configuration = null;
    state.info = null;
    state.eePose = null;
    state.missingMeshes = new Set();
    state.visualError = '';
    grid.position.z = gridHeight;
    gapLine.visible = false;
    setEditing(false);
    showNotes([]);
    $('quat').textContent = '';
  }

  async function loadModel(modelPath) {
    modelLoad?.abort();
    modelLoad = new AbortController();
    const { signal } = modelLoad;
    const gen = ++state.generation;
    state.modelPath = modelPath;
    try { localStorage.setItem('urdf-ik-model', modelPath); } catch { /* optional */ }
    clearRobot();
    updateSchemes();
    $('ee').disabled = true;
    $('ee').replaceChildren();
    $('joints').replaceChildren();
    handle.visible = eeMarker.visible = skeleton.visible = false;
    setStatus('idle', 'Loading…', modelPath);

    const visual = loadVisual(modelPath, signal).then(robot => {
      if (!robot) return;
      if (signal.aborted) {
        disposeObject3D(robot.root);
        return;
      }
      if (state.configuration) robot.update(state.configuration);
      scene.add(robot.root);
      state.robot = robot;
      updateMeshVisibility();
      alignG1Grid();
      fitView();
      showNotes(currentNotes());
    }).catch(error => {
      if (signal.aborted) return;
      state.visualError = `Could not load robot meshes: ${error.message}`;
      showNotes(currentNotes());
    });
    let info = null;
    try {
      info = await request('api/load', { model: modelPath }, signal);
      if (modelPath === g1Path) {
        const stance = await request('api/fk', {
          model: modelPath, ee: info.default_ee, q: homeJoints(modelPath, info.joints),
        }, signal);
        Object.assign(info, stance);
      }
    } catch (err) {
      if (gen !== state.generation) return;
      await visual;
      if (signal.aborted) return;
      fitView();
      setStatus('error', 'Cannot solve IK for this model', '', [{
        text: modelPath === g1Path ? err.message : `${err.message} The URDF is shown for reference only.`,
        kind: 'reason',
      }]);
      showNotes(currentNotes());
      if (!browserBackend) refreshModelList();
      return;
    }
    if (gen !== state.generation) return;

    state.info = info;
    state.ee = info.default_ee;
    $('ee').replaceChildren(...info.end_effectors.map((e) => {
      const opt = document.createElement('option');
      opt.value = e.name;
      opt.textContent = endEffectorLabel(e);
      return opt;
    }));
    $('ee').value = state.ee;
    $('ee').disabled = false;
    setOrientation(eeDof() >= 6);

    applyConfiguration(info);
    updateMeshVisibility();
    alignG1Grid();
    buildJointWidgets();
    updateJointWidgets([]);
    setTargetToCurrent();
    handle.visible = eeMarker.visible = true;
    skeleton.visible = $('show-skeleton').checked;
    fitView();
    setStatus('idle', 'Ready', `${info.joints.length} joints`, [{
      text: 'Drag the orange handle to solve IK.', kind: 'info',
    }]);
    showNotes(currentNotes());
    if (!browserBackend) refreshModelList();

    await visual;
  }

  function alignG1Grid() {
    if (state.modelPath === g1Path && state.robot && state.configuration) {
      grid.position.z = new THREE.Box3().setFromObject(state.robot.root).min.z + gridHeight;
    }
  }

  function updateMeshVisibility() {
    if (state.robot) {
      state.robot.root.visible = $('show-meshes').checked &&
        (state.modelPath !== g1Path || !!state.configuration);
    }
  }

  function eeDof() {
    return state.info?.end_effectors.find((e) => e.name === state.ee)?.dof ?? 0;
  }

  function updateMethod() {
    hooks.onMethod?.($('scheme').value, $('orientation').checked);
  }

  function setOrientation(checked) {
    $('orientation').checked = checked;
    updateSchemes();
  }

  function describeScheme() {
    const scheme = schemes.find(s => s.id === $('scheme').value);
    if (!scheme) return;
    $('scheme-desc').textContent = scheme.description;
    $('max-iters').placeholder = `auto (${scheme.default_iters})`;
    updateMethod();
  }

  function updateSchemes() {
    const previous = $('scheme').value;
    const ee = state.info?.end_effectors.find(e => e.name === state.ee);
    const available = availableSchemes(schemes, ee, $('orientation').checked);
    $('scheme').replaceChildren(...available.map(s => {
      const option = document.createElement('option');
      option.value = s.id;
      option.textContent = s.label;
      return option;
    }));
    const selected = available.find(s => s.id === previous) ??
      (previous ? available.find(s => s.id === 'dls') : null) ?? available[0];
    if (selected) $('scheme').value = selected.id;
    describeScheme();
  }

  async function changeEndEffector(name) {
    if (name === state.ee) return;
    $('ee').value = name;
    const gen = ++state.generation;
    state.ee = name;
    setEditing(false);
    setOrientation(eeDof() >= 6);
    try {
      const res = await request('api/fk', { model: state.modelPath, ee: name, q: state.q });
      if (gen !== state.generation) return;
      applyConfiguration(res);
      setTargetToCurrent();
      setStatus('idle', 'Ready', `${eeDof()} DoF to ${name}`);
    } catch (err) {
      if (gen === state.generation && !lifetime.signal.aborted) showError(err.message);
    }
  }

  async function refreshModelList() {
    try {
      const { models, urdf_url_prefix: prefix } = await request('api/models');
      if (lifetime.signal.aborted) return [];
      if (prefix) state.urdfPrefix = prefix;
      const select = $('model');
      const selected = select.value || state.modelPath;
      select.replaceChildren(...models.map((m) => {
        const opt = document.createElement('option');
        opt.value = m.path;
        const name = m.path === g1Path ? 'Unitree G1' : m.path;
        opt.textContent = m.status === 'error' ? `${name}  (not loadable)` : name;
        return opt;
      }));
      if (selected) select.value = selected;
      return models;
    } catch (err) {
      if (!lifetime.signal.aborted) showError(`Could not list robot models: ${err.message}`);
      return [];
    }
  }

  // ------------------------------------------------------------------ init

  async function init() {
    buildPoseTable();
    if (hooks.backend) {
      try {
        schemes = await request('api/schemes');
      } catch {
        // A development server can still run the bundled models without Python.
      }
    }
    if (lifetime.signal.aborted) return;
    if (!schemes?.length) {
      browserBackend = createBrowserBackend();
      schemes = await request('api/schemes');
    }
    if (lifetime.signal.aborted) return;
    hooks.onMode?.(browserBackend ? 'browser' : 'native');
    updateSchemes();

    $('scheme').addEventListener('change', () => { describeScheme(); requestIK(state.generation); });
    $('orientation').addEventListener('change', () => { updateSchemes(); requestIK(state.generation); });
    for (const id of ['pos-tol', 'rot-tol', 'max-iters']) {
      $(id).addEventListener('change', () => requestIK(state.generation));
    }
    $('model').addEventListener('change', () => loadModel($('model').value));
    $('ee').addEventListener('change', () => changeEndEffector($('ee').value));
    $('solve').addEventListener('click', () => { applyPoseInputs(); });
    $('reset-target').addEventListener('click', () => {
      setTargetToCurrent();
      viewport.render();
    });
    $('home').addEventListener('click', () => {
      if (!state.info) return;
      state.q = homeJoints(state.modelPath, state.info.joints);
      requestFK(state.generation);
    });
    $('fit').addEventListener('click', fitView);
    $('show-skeleton').addEventListener('change', (e) => {
      skeleton.visible = e.target.checked && !!state.info;
      viewport.render();
    });
    $('show-meshes').addEventListener('change', (e) => {
      updateMeshVisibility();
      viewport.render();
    });

    const models = await refreshModelList();
    let initial = null;
    try { initial = localStorage.getItem('urdf-ik-model'); } catch { /* optional */ }
    const paths = models.map((m) => m.path);
    if (!paths.includes(initial)) {
      initial = paths.find((p) => p.startsWith('ur10e/')) || paths[0];
    }
    if (initial) {
      $('model').value = initial;
      loadModel(initial);
    }
  }

  init().catch((err) => {
    if (lifetime.signal.aborted) return;
    hooks.onMode?.('error');
    showError(err.message);
  });

  return {
    dispose() {
      state.generation++;
      lifetime.abort();
      modelLoad?.abort();
      browserBackend?.dispose();
      stopThemeSync();
      clearTimeout(toastTimer);
      scene.remove(gizmo);
      gizmo.dispose();
      viewport.dispose();
    },
  };
}
