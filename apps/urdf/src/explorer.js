/**
 * URDF IK Explorer: the three.js scene and side-panel logic, driving the
 * GTDynamics backend (../scripts/server.py). App.vue owns the markup and mounts
 * this with createExplorer().
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';
import { api, latestOnly } from './api.js';
import {
  POSE_FIELDS, endEffectorLabel, homeConfiguration, quaternionFromRpyDeg,
  rpyDegFromQuaternion, sliderRange, solveSummary,
} from './pose.js';

// URDFs are Z-up; make three.js follow.
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const $ = (id) => document.getElementById(id);

/**
 * Builds the scene inside #viewport and wires the controls rendered by App.vue.
 * hooks.backend: whether a backend can be reached (false for a static build,
 * which has no proxy to it). hooks.onOffline(message) runs when there is no
 * backend or it does not answer; hooks.onLive() runs once it lists its IK schemes.
 * Returns { dispose }.
 */
export function createExplorer(hooks = {}) {
  const state = {
    generation: 0,      // bumped on every model / end-effector change
    modelPath: null,
    urdfPrefix: 'files/urdfs/',
    info: null,         // /api/load response
    ee: null,
    q: {},              // joint name -> value
    eePose: null,       // achieved end-effector pose from the server
    robot: null,        // URDFRobot
    scale: 1,           // characteristic robot size in meters
    editing: false,
    failed: false,
    missingMeshes: new Set(),
  };

  // ------------------------------------------------------------------ scene

  const viewport = $('viewport');
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  viewport.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x20232a);
  scene.add(new THREE.HemisphereLight(0xf0f3ff, 0x40362c, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(3, -2, 5);
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0xffffff, 0.5);
  fill.position.set(-3, 4, 2);
  scene.add(fill);

  const grid = new THREE.GridHelper(10, 50, 0x5a606b, 0x363a42);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  const worldAxes = new THREE.AxesHelper(0.3);
  scene.add(worldAxes);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.005, 500);
  camera.position.set(2, -2.5, 1.6);
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.target.set(0, 0, 0.5);
  orbit.enableDamping = true;

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
    new THREE.LineBasicMaterial({ color: 0x5ad1ff, transparent: true, opacity: 0.8, depthTest: false }),
  );
  const skeletonPoints = new THREE.Points(
    new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ color: 0x5ad1ff, size: 6, sizeAttenuation: false, depthTest: false }),
  );
  skeletonLines.renderOrder = skeletonPoints.renderOrder = 18;
  skeleton.add(skeletonLines, skeletonPoints);
  scene.add(skeleton);

  const gizmo = new TransformControls(camera, renderer.domElement);
  gizmo.setSpace('local');
  gizmo.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; });
  gizmo.addEventListener('objectChange', () => onTargetMoved());
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

  const resizer = new ResizeObserver(() => {
    const { clientWidth: w, clientHeight: h } = viewport;
    renderer.setSize(w, h);
    camera.aspect = w / Math.max(h, 1);
    camera.updateProjectionMatrix();
  });
  resizer.observe(viewport);

  renderer.setAnimationLoop(() => {
    orbit.update();
    renderer.render(scene, camera);
  });

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
      const res = await api('api/ik', body);
      if (gen === state.generation) applySolve(res);
    } catch (err) {
      if (gen === state.generation) showError(err.message);
    }
  });

  const requestFK = latestOnly(async (gen) => {
    if (gen !== state.generation || !state.info) return;
    try {
      const res = await api('api/fk', { model: state.modelPath, ee: state.ee, q: state.q });
      if (gen !== state.generation) return;
      applyConfiguration(res);
      setTargetToCurrent();
      setStatus('idle', 'Forward kinematics', '');
    } catch (err) {
      if (gen === state.generation) showError(err.message);
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
    updatePoseTable();
  }

  function onTargetMoved() {
    updatePoseTable();
    requestIK(state.generation);
  }

  // ------------------------------------------------------------- applying

  function applyConfiguration(res) {
    state.q = res.q;
    state.eePose = res.ee_pose;
    if (state.robot) {
      for (const [name, value] of Object.entries(res.q)) {
        if (state.robot.joints[name]) state.robot.setJointValue(name, value);
      }
    }
    poseToObject(res.ee_pose, eeMarker);
    updateSkeleton(res.skeleton);
    updateJointWidgets(res.at_limits || []);
    updatePoseTable();
  }

  function applySolve(res) {
    applyConfiguration(res);
    state.failed = res.status !== 'solved';
    handleMaterial.color.set(state.failed ? 0xff4b45 : 0xff8a1f);
    handleMaterial.emissive.set(state.failed ? 0x801010 : 0x803000);

    gapLine.visible = state.failed;
    if (state.failed) {
      gapLine.geometry.setFromPoints([eeMarker.position.clone(), target.position.clone()]);
      gapLine.computeLineDistances();
    }

    const { cls, label, meta, items } = solveSummary(res, $('orientation').checked);
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
    const list = $('reasons');
    list.replaceChildren(...items.map(({ text, kind }) => {
      const li = document.createElement('li');
      li.className = kind;
      li.textContent = text;
      return li;
    }));
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
      tr.innerHTML = `<th>${name} (${unit})</th><td class="cur" id="cur-${i}">–</td>` +
        `<td><input type="number" id="tgt-${i}" step="${i < 3 ? 0.01 : 1}"></td>`;
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
        <div class="joint-head"><span>${joint.name}</span><span class="val"></span></div>
        <input type="range" min="${lo}" max="${hi}" step="${(hi - lo) / 2000}">
        <div class="limits"><span>${joint.lower === null ? '−∞' : lo.toFixed(2)}</span>` +
        `<span>${joint.type}, ${unit}</span>` +
        `<span>${joint.upper === null ? '+∞' : hi.toFixed(2)}</span></div>`;
      const slider = div.querySelector('input');
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
    if (overHandle(event)) setEditing(!state.editing);
  });

  function setEditing(on) {
    state.editing = on && !!state.info;
    $('pose-section').classList.toggle('editing', state.editing);
    if (state.editing) {
      if (!$('orientation').checked) {
        $('orientation').checked = true;
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
  window.addEventListener('keydown', onKeyDown);

  // --------------------------------------------------------------- loading

  function loadMesh(path, manager, done) {
    const ext = path.split(/[?#]/)[0].split('.').pop().toLowerCase();
    const fail = (err) => {
      state.missingMeshes.add(path);
      done(null, err);
    };
    if (ext === 'stl') {
      new STLLoader(manager).load(path, (geometry) => {
        done(new THREE.Mesh(geometry, new THREE.MeshPhongMaterial({ color: 0xb4b9c2 })));
      }, undefined, fail);
    } else if (ext === 'dae') {
      new ColladaLoader(manager).load(path, (collada) => done(collada.scene), undefined, fail);
    } else if (ext === 'obj') {
      new OBJLoader(manager).load(path, (obj) => done(obj), undefined, fail);
    } else {
      fail(new Error(`No loader for .${ext} meshes`));
    }
  }

  function loadVisual(modelPath, gen) {
    return new Promise((resolve) => {
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      loader.packages = (pkg) => `pkg/${pkg}`;
      loader.loadMeshCb = loadMesh;
      let robot = null;
      manager.onLoad = () => resolve(robot);
      manager.onError = () => {};
      loader.load(state.urdfPrefix + modelPath, (result) => {
        if (gen !== state.generation) return;
        robot = result;
        for (const joint of Object.values(robot.joints)) joint.ignoreLimits = true;
        robot.visible = $('show-meshes').checked;
        scene.add(robot);
        state.robot = robot;
      }, undefined, () => resolve(null));
    });
  }

  function fitView() {
    const box = new THREE.Box3();
    if (state.robot) box.expandByObject(state.robot);
    const pos = skeletonPoints.geometry.getAttribute('position');
    if (pos) {
      for (let i = 0; i < pos.count; i++) box.expandByPoint(new THREE.Vector3().fromBufferAttribute(pos, i));
    }
    box.expandByPoint(eeMarker.position);
    if (box.isEmpty()) return;
    const center = box.getCenter(new THREE.Vector3());
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.2);
    state.scale = Math.min(Math.max(size, 0.25), 20);
    setMarkerScale();
    orbit.target.copy(center);
    camera.position.copy(center).add(new THREE.Vector3(0.75, -1.0, 0.55).multiplyScalar(size));
    camera.near = size / 500;
    camera.far = size * 100;
    camera.updateProjectionMatrix();
    grid.scale.setScalar(Math.max(1, Math.ceil(size / 2)));
  }

  function clearRobot() {
    if (state.robot) {
      scene.remove(state.robot);
      state.robot.traverse((obj) => obj.geometry?.dispose());
    }
    state.robot = null;
    state.info = null;
    state.eePose = null;
    state.missingMeshes = new Set();
    gapLine.visible = false;
    setEditing(false);
  }

  async function loadModel(modelPath) {
    const gen = ++state.generation;
    state.modelPath = modelPath;
    try { localStorage.setItem('urdf-ik-model', modelPath); } catch { /* optional */ }
    clearRobot();
    setStatus('idle', 'Loading…', modelPath);
    $('ee').disabled = true;
    $('ee').replaceChildren();
    $('joints').replaceChildren();
    handle.visible = eeMarker.visible = skeleton.visible = false;

    const visual = loadVisual(modelPath, gen);
    let info = null;
    try {
      info = await api('api/load', { model: modelPath });
    } catch (err) {
      if (gen !== state.generation) return;
      await visual;
      fitView();
      setStatus('error', 'Cannot solve IK for this model', '', [{
        text: `${err.message} The URDF is shown for reference only.`, kind: 'reason',
      }]);
      showNotes(currentNotes());
      refreshModelList();
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
    $('orientation').checked = eeDof() >= 6;

    applyConfiguration(info);
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
    refreshModelList();

    await visual;
    if (gen !== state.generation) return;
    if (state.robot) {
      for (const [name, value] of Object.entries(state.q)) {
        if (state.robot.joints[name]) state.robot.setJointValue(name, value);
      }
    }
    fitView();
    showNotes(currentNotes());
  }

  function eeDof() {
    return state.info?.end_effectors.find((e) => e.name === state.ee)?.dof ?? 0;
  }

  async function changeEndEffector(name) {
    const gen = ++state.generation;
    state.ee = name;
    setEditing(false);
    $('orientation').checked = eeDof() >= 6;
    try {
      const res = await api('api/fk', { model: state.modelPath, ee: name, q: state.q });
      if (gen !== state.generation) return;
      applyConfiguration(res);
      setTargetToCurrent();
      setStatus('idle', 'Ready', `${eeDof()} DoF to ${name}`);
    } catch (err) {
      showError(err.message);
    }
  }

  async function refreshModelList() {
    try {
      const { models, urdf_url_prefix: prefix } = await api('api/models');
      state.urdfPrefix = prefix;
      const select = $('model');
      const selected = select.value || state.modelPath;
      select.replaceChildren(...models.map((m) => {
        const opt = document.createElement('option');
        opt.value = m.path;
        opt.textContent = m.status === 'error' ? `${m.path}  (not loadable)` : m.path;
        return opt;
      }));
      if (selected) select.value = selected;
      return models;
    } catch (err) {
      showError(`Cannot reach the server: ${err.message}`);
      return [];
    }
  }

  // ------------------------------------------------------------------ init

  async function init() {
    buildPoseTable();
    let schemes;
    try {
      if (hooks.backend === false) throw new Error('this static build has no backend');
      schemes = await api('api/schemes');
    } catch (err) {
      setStatus('error', 'Backend not running', '', [{
        text: 'IK runs on a local GTDynamics Python server. See the notice for how to start it.',
        kind: 'reason',
      }]);
      hooks.onOffline?.(err.message);
      return;
    }
    hooks.onLive?.();
    $('scheme').replaceChildren(...schemes.map((s) => {
      const opt = document.createElement('option');
      opt.value = s.id;
      opt.textContent = s.label;
      return opt;
    }));
    const describeScheme = () => {
      const s = schemes.find((x) => x.id === $('scheme').value);
      $('scheme-desc').textContent = s.description;
      $('max-iters').placeholder = `auto (${s.default_iters})`;
    };
    describeScheme();

    $('scheme').addEventListener('change', () => { describeScheme(); requestIK(state.generation); });
    for (const id of ['orientation', 'pos-tol', 'rot-tol', 'max-iters']) {
      $(id).addEventListener('change', () => requestIK(state.generation));
    }
    $('model').addEventListener('change', () => loadModel($('model').value));
    $('ee').addEventListener('change', () => changeEndEffector($('ee').value));
    $('solve').addEventListener('click', () => { applyPoseInputs(); });
    $('reset-target').addEventListener('click', () => {
      setTargetToCurrent();
      gapLine.visible = false;
    });
    $('home').addEventListener('click', () => {
      if (!state.info) return;
      state.q = homeConfiguration(state.info.joints);
      requestFK(state.generation);
    });
    $('fit').addEventListener('click', fitView);
    $('show-skeleton').addEventListener('change', (e) => { skeleton.visible = e.target.checked && !!state.info; });
    $('show-meshes').addEventListener('change', (e) => { if (state.robot) state.robot.visible = e.target.checked; });

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

  init().catch((err) => showError(err.message));

  return {
    dispose() {
      state.generation++;
      window.removeEventListener('keydown', onKeyDown);
      clearTimeout(toastTimer);
      resizer.disconnect();
      renderer.setAnimationLoop(null);
      gizmo.dispose();
      orbit.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    },
  };
}
