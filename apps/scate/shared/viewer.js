import * as THREE from 'three';
import { RobotViewport } from '../../shared/RobotViewport.js';
import { onThemeChange } from '../../shared/theme.js';

/** Mesh name (in meshes.json) of each UR link, shared by the UR5 and UR10e. */
export const UR_MESH_FOR_LINK = {
  base_link: "base", shoulder_link: "shoulder", upper_arm_link: "upperarm", forearm_link: "forearm",
  wrist_1_link: "wrist1", wrist_2_link: "wrist2", wrist_3_link: "wrist3",
};

/**
 * Replay saved world-frame link poses; no browser FK or native optimization.
 * @param root page element holding the viewer's controls
 * @param options.robotName robot name for the canvas label, e.g. "UR5"
 * @param options.meshForLink mesh name in meshes.json for each link name in the replay
 * @param options.onReplay called with each loaded replay
 * @param options.onTime called with the playback time
 */
export function createReplayViewer(root, { robotName = 'UR5', meshForLink = UR_MESH_FOR_LINK, onReplay, onTime } = {}) {
  let disposed = false;
  const loading = new AbortController(), events = new AbortController();
  const listen = (target, type, handler) => target.addEventListener(type, handler, { signal: events.signal });


  const $ = (id) => root.querySelector(`#${id}`);

  // ---------- theme ----------
  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const col = (name) => new THREE.Color(css(name) || "#888888");

  // ---------- binary STL ----------
  function parseSTL(buf) {
    const dv = new DataView(buf);
    const n = dv.getUint32(80, true);
    if (84 + n * 50 !== buf.byteLength) throw new Error("not a binary STL");
    const pos = new Float32Array(n * 9);
    for (let i = 0; i < n; i++) {
      const o = 84 + i * 50;
      for (let v = 0; v < 3; v++) {
        for (let a = 0; a < 3; a++) pos[i * 9 + v * 3 + a] = dv.getFloat32(o + 12 + v * 12 + a * 4, true);
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    // The UR5 STLs store all-zero facet normals, so derive them from the triangles.
    g.computeVertexNormals();
    return g;
  }

  // ---------- small math ----------
  const lerp = (a, b, s) => a + (b - a) * s;
  const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);
  function eigSym3(m) {  // Jacobi; m as 9 numbers row-major -> {vals, vecs (columns)}
    const a = [[m[0], m[1], m[2]], [m[3], m[4], m[5]], [m[6], m[7], m[8]]];
    const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (let sweep = 0; sweep < 50; sweep++) {
      let off = Math.abs(a[0][1]) + Math.abs(a[0][2]) + Math.abs(a[1][2]);
      if (off < 1e-18) break;
      for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
        if (Math.abs(a[p][q]) < 1e-20) continue;
        const th = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
        const c = Math.cos(th), s = Math.sin(th);
        for (let k = 0; k < 3; k++) {
          const akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq; v[k][q] = s * vkp + c * vkq;
        }
      }
    }
    return { vals: [a[0][0], a[1][1], a[2][2]], vecs: v };
  }

  // ---------- scene ----------
  const stage = $("stage");
  const viewport = new RobotViewport(stage, {
    background: '#eef1f3', fov: 40, near: .01, far: 50,
    controls: { minDistance: .1, maxDistance: 10, maxPolarAngle: Math.PI },
    ariaLabel: `${robotName} replay; drag to orbit, scroll to zoom`,
  });
  const { scene, camera, controls } = viewport;
  viewport.addStage({ ground: false, grid: {size: 2, divisions: 20, colors: ['#dde2e6', '#e3e7ea']} });
  scene.add(new THREE.AxesHelper(.15));

  const world = new THREE.Group();
  scene.add(world);

  // Materials (recolored on theme change).
  const M = {};
  function makeMaterials() {
    M.robot = new THREE.MeshStandardMaterial({ color: col("--robot"), roughness: 0.65, metalness: 0.1 });
    M.robotGhost = new THREE.MeshStandardMaterial({ color: col("--blind"), roughness: 0.7, transparent: true, opacity: 0.28, depthWrite: false });
    M.obstacle = new THREE.MeshStandardMaterial({ color: col("--obstacle"), roughness: 0.5 });
    M.shell = new THREE.MeshBasicMaterial({ color: col("--critical"), wireframe: true, transparent: true, opacity: 0.22 });
    M.est = new THREE.MeshBasicMaterial({ color: col("--online"), transparent: true, opacity: 0.22, depthWrite: false });
    M.estDot = new THREE.MeshBasicMaterial({ color: col("--online") });
    M.range = new THREE.MeshBasicMaterial({ color: col("--online"), wireframe: true, transparent: true, opacity: 0.12 });
    M.sphOk = new THREE.MeshStandardMaterial({ color: col("--online"), transparent: true, opacity: 0.18, depthWrite: false });
    M.sphOkBlind = new THREE.MeshStandardMaterial({ color: col("--blind"), transparent: true, opacity: 0.18, depthWrite: false });
    M.sphWarn = new THREE.MeshStandardMaterial({ color: col("--serious"), transparent: true, opacity: 0.55, depthWrite: false });
    M.sphBad = new THREE.MeshStandardMaterial({ color: col("--critical"), transparent: true, opacity: 0.7, depthWrite: false });
    M.lineOnline = new THREE.LineBasicMaterial({ color: col("--online") });
    M.lineBlind = new THREE.LineBasicMaterial({ color: col("--blind") });
    M.lineOnlineFaint = new THREE.LineBasicMaterial({ color: col("--online"), transparent: true, opacity: 0.25 });
    M.lineBlindFaint = new THREE.LineBasicMaterial({ color: col("--blind"), transparent: true, opacity: 0.25 });
    M.planOnline = new THREE.LineDashedMaterial({ color: col("--online"), dashSize: 0.015, gapSize: 0.01 });
    M.planOff = new THREE.LineDashedMaterial({ color: col("--muted"), dashSize: 0.02, gapSize: 0.012 });
  }

  // ---------- data ----------
  let D = null;        // replay
  let R = {};          // runs by name
  let meshGeo = {};    // STL geometry by mesh name
  let T = 0, Tend = 1; // time
  let playing = false, speed = 2;
  let objs = null;     // scene objects for the loaded replay

  // The arm's collision STLs (GTDynamics models/urdfs/<robot>/collision), base64 in one file.
  // A standalone copy written by plot_scate.py embeds the meshes and the run
  // as window.SCATE_MESHES / window.SCATE_REPLAY; the hosted page fetches them.
  async function loadMeshes() {
    try {
      let pack = window.SCATE_MESHES;
      if (!pack) {
        const r = await fetch(new URL("meshes.json", document.baseURI), { signal: loading.signal });
        if (!r.ok) throw new Error(`Meshes: HTTP ${r.status}`);
        pack = await r.json();
      }
      if (disposed) return;
      for (const [name, b64] of Object.entries(pack.meshes || {})) {
        const bin = atob(b64), bytes = new Uint8Array(bin.length);
        for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
        meshGeo[name] = parseSTL(bytes.buffer);
        meshGeo[name].userData.sharedMujoco = true;
      }
    } catch (e) { if (!disposed) throw e; }
  }

  function showError(msg) { const e = $("err"); e.textContent = msg; e.hidden = !msg; }

  function setReplay(d, imported = false) {
    if (!d || d.format !== "scate-replay" || !Array.isArray(d.runs) || !d.runs.length) {
      throw new Error("This file is not a SCATE replay (expected the replay.json written by scate_online).");
    }
    D = d; R = {};
    for (const r of d.runs) R[r.name] = r;
    Tend = (d.num_poses - 1) * d.dt;
    const demo = /^SANDBOX/i.test(d.source || "");
    const tag = $("srcTag");
    tag.textContent = demo ? "Demo data" : imported ? "Loaded replay" : "Bundled replay";
    tag.className = "tag" + (demo ? " demo" : "");
    tag.title = d.source || "";
    $("nmOnline").textContent = R.online ? R.online.label : "Online";
    $("nmBlind").textContent = R.blind ? R.blind.label : "Blind";
    $("blindRow").hidden = $("blindRead").hidden = !R.blind;
    $("note").textContent =
      "Clearance is the gap between the closest collision sphere and the true obstacle surface. Click or drag on the chart to scrub.";
    buildObjects();
    frameCamera();
    // Open at the most telling instant: where the blind run comes closest.
    const ref = R.blind || R.online;
    let best = 0, bestC = Infinity;
    ref.frames.forEach((f) => { if (f.clear !== null && f.clear < bestC) { bestC = f.clear; best = f.t; } });
    setTime(best);
    drawChart();
    onReplay?.(D);
    stage.dataset.ready = "true";
  }

  // Aim at everything the replay touches: every sphere center of every run, and the obstacle.
  function frameCamera() {
    const box = new THREE.Box3();
    for (const r of Object.values(R)) {
      for (let i = 0; i < r.frames.length; i += 5) r.frames[i].spheres.forEach((c) => box.expandByPoint(v3(c)));
    }
    box.expandByPoint(v3(D.obstacle.center));
    if (box.isEmpty()) return;
    viewport.cameraController.fit(box, { direction: new THREE.Vector3(.75, -.95, .6), padding: 1.05, minDistance: .1 });
  }

  function lineFrom(points, mat, dashed) {
    const g = new THREE.BufferGeometry().setFromPoints(points.map(v3));
    const l = new THREE.Line(g, mat);
    if (dashed) l.computeLineDistances();
    return l;
  }

  function buildArm(mat, sphMat, run) {
    const arm = { links: [], spheres: [], group: new THREE.Group(), sph: new THREE.Group(), sphMat };
    for (const name of D.links) {
      const holder = new THREE.Group();
      const geo = meshGeo[meshForLink[name]];
      if (geo) holder.add(new THREE.Mesh(geo, mat));
      arm.group.add(holder);
      arm.links.push(holder);
    }
    const unit = new THREE.SphereGeometry(1, 20, 14);
    D.sphere_radii.forEach((r) => {
      const s = new THREE.Mesh(unit, sphMat);
      s.scale.setScalar(r);
      arm.sph.add(s);
      arm.spheres.push(s);
    });
    world.add(arm.group, arm.sph);
    const tool = D.tool_sphere;
    if (tool >= 0) {
      arm.trailFull = lineFrom(run.frames.map((f) => f.spheres[tool]),
        run.name === "online" ? M.lineOnlineFaint : M.lineBlindFaint);
      const pts = new Float32Array(run.frames.length * 3);
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(pts, 3));
      arm.trail = new THREE.Line(g, run.name === "online" ? M.lineOnline : M.lineBlind);
      world.add(arm.trailFull, arm.trail);
    }
    return arm;
  }

  function clearWorld() {
    const geometries = new Set();
    world.traverse(object => { if (object.geometry && !object.geometry.userData.sharedMujoco) geometries.add(object.geometry); });
    geometries.forEach(geometry => geometry.dispose());
    world.clear();
  }

  function buildObjects() {
    clearWorld();
    objs = { arms: {} };
    const o = D.obstacle;
    const obs = new THREE.Mesh(new THREE.SphereGeometry(o.radius, 40, 28), M.obstacle);
    obs.position.copy(v3(o.center));
    const shell = new THREE.Mesh(new THREE.SphereGeometry(o.radius + D.obstacle_standoff, 28, 18), M.shell);
    shell.position.copy(obs.position);
    world.add(obs, shell);
    objs.shell = shell;

    const est = new THREE.Mesh(new THREE.SphereGeometry(1, 24, 16), M.est);
    const estDot = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 8), M.estDot);
    world.add(est, estDot);
    objs.est = est; objs.estDot = estDot;

    const range = new THREE.Mesh(new THREE.SphereGeometry(1, 28, 18), M.range);
    world.add(range);
    objs.range = range;

    if (R.online) objs.arms.online = buildArm(M.robot, M.sphOk, R.online);
    if (R.blind) objs.arms.blind = buildArm(R.online ? M.robotGhost : M.robot, M.sphOkBlind, R.blind);

    const off = (R.online || R.blind).steps[0];
    objs.offline = off && off.plan_tool.length > 1 ? lineFrom(off.plan_tool, M.planOff, true) : null;
    if (objs.offline) world.add(objs.offline);
    objs.plan = null;
    objs.planStep = -1;
    applyVisibility();
  }

  // ---------- per-frame state ----------
  function frameAt(run, t) {
    const F = run.frames;
    const step = F.length > 1 ? (F[F.length - 1].t - F[0].t) / (F.length - 1) : 1;
    const x = Math.max(0, Math.min(F.length - 1, (t - F[0].t) / step));
    const i = Math.min(F.length - 2, Math.floor(x));
    return { a: F[i], b: F[i + 1] || F[i], s: x - i, i: Math.round(x) };
  }

  function stepAt(run, t) {  // latest planner step whose plan is in force at time t
    const k = Math.min(run.steps.length - 1, Math.floor(t / D.dt + 1e-9));
    return run.steps[Math.max(0, k)];
  }

  function sphereStates(centers) {
    const O = D.obstacle.center, Rr = D.obstacle.radius, rad = D.sphere_radii;
    const st = rad.map((r, i) => {
      const c = centers[i];
      const gap = Math.hypot(c[0] - O[0], c[1] - O[1], c[2] - O[2]) - Rr - r;
      return gap < 0 ? 2 : gap < D.obstacle_standoff ? 1 : 0;
    });
    for (const [a, b] of D.pairs || []) {
      const ca = centers[a], cb = centers[b];
      const gap = Math.hypot(ca[0] - cb[0], ca[1] - cb[1], ca[2] - cb[2]) - rad[a] - rad[b];
      const s = gap < 0 ? 2 : gap < D.self_standoff ? 1 : 0;
      st[a] = Math.max(st[a], s); st[b] = Math.max(st[b], s);
    }
    return st;
  }

  const q0 = new THREE.Quaternion(), q1 = new THREE.Quaternion();
  function poseArm(arm, run, t) {
    const { a, b, s, i } = frameAt(run, t);
    arm.links.forEach((h, l) => {
      const A = a.links[l], B = b.links[l];
      if (!A) return;
      h.position.set(lerp(A[0], B[0], s), lerp(A[1], B[1], s), lerp(A[2], B[2], s));
      q0.set(A[3], A[4], A[5], A[6]); q1.set(B[3], B[4], B[5], B[6]);
      h.quaternion.copy(q0).slerp(q1, s);
    });
    const centers = a.spheres.map((c, k) => {
      const d = b.spheres[k];
      return [lerp(c[0], d[0], s), lerp(c[1], d[1], s), lerp(c[2], d[2], s)];
    });
    const st = sphereStates(centers);
    arm.spheres.forEach((m, k) => {
      m.position.set(centers[k][0], centers[k][1], centers[k][2]);
      m.material = st[k] === 2 ? M.sphBad : st[k] === 1 ? M.sphWarn : arm.sphMat;
    });
    if (arm.trail) {
      const pos = arm.trail.geometry.attributes.position;
      const tool = D.tool_sphere;
      const n = Math.min(run.frames.length, i + 1);
      for (let k = 0; k < n; k++) {
        const p = k === n - 1 ? centers[tool] : run.frames[k].spheres[tool];
        pos.setXYZ(k, p[0], p[1], p[2]);
      }
      pos.needsUpdate = true;
      arm.trail.geometry.setDrawRange(0, n);
    }
    return { clear: lerp(a.clear, b.clear, s), self: a.self === null ? null : lerp(a.self, b.self, s) };
  }

  const fmtCm = (m) => (m === null || m === undefined || !isFinite(m)) ? "–" : (100 * m).toFixed(1) + " cm";
  function chip(el, clear) {
    if (clear === null || !isFinite(clear)) { el.className = "chip idle"; el.textContent = "–"; return; }
    if (clear < 0) { el.className = "chip bad"; el.textContent = "collision"; }
    else if (clear < D.obstacle_standoff) { el.className = "chip warn"; el.textContent = "inside standoff"; }
    else { el.className = "chip ok"; el.textContent = "clear"; }
  }

  function setTime(t) {
    if (!D) return;
    T = Math.max(0, Math.min(Tend, t));
    const which = root.querySelector('input[name="runs"]:checked').value;
    let on = null, bl = null;
    if (R.online && objs.arms.online) on = poseArm(objs.arms.online, R.online, T);
    if (R.blind && objs.arms.blind) bl = poseArm(objs.arms.blind, R.blind, T);

    if (on) { $("rOnClear").textContent = fmtCm(on.clear); $("rOnSelf").textContent = fmtCm(on.self); chip($("chipOnline"), on.clear); }
    if (bl) { $("rBlClear").textContent = fmtCm(bl.clear); chip($("chipBlind"), bl.clear); }

    // Estimate, current plan, sensor range: the online run.
    const run = R.online;
    if (run) {
      const st = stepAt(run, T);
      $("rK").textContent = String(st.k);
      $("rMs").textContent = st.solve_ms.toFixed(1) + " ms" + (st.solves > 1 ? " (2 solves)" : "");
      const seenK = run.summary.first_detection;
      if (st.l) {
        const O = D.obstacle.center;
        const e = Math.hypot(st.l[0] - O[0], st.l[1] - O[1], st.l[2] - O[2]);
        $("rSeen").textContent = "seen at step " + seenK;
        $("rErr").textContent = fmtCm(e);
        objs.estDot.position.copy(v3(st.l));
        objs.est.position.copy(v3(st.l));
        if (st.l_cov) {
          const { vals, vecs } = eigSym3(st.l_cov);
          const sd = vals.map((x) => Math.sqrt(Math.max(x, 0)));
          $("rSig").textContent = fmtCm(3 * Math.max(...sd));
          const basis = new THREE.Matrix4().makeBasis(
            new THREE.Vector3(vecs[0][0], vecs[1][0], vecs[2][0]),
            new THREE.Vector3(vecs[0][1], vecs[1][1], vecs[2][1]),
            new THREE.Vector3(vecs[0][2], vecs[1][2], vecs[2][2]));
          objs.est.quaternion.setFromRotationMatrix(basis);
          objs.est.scale.set(3 * sd[0], 3 * sd[1], 3 * sd[2]);
        } else {
          $("rSig").textContent = "–";
          objs.est.scale.setScalar(0.0001);
        }
        objs.est.userData.known = true;
      } else {
        $("rSeen").textContent = seenK < 0 ? "never seen" : "not yet seen";
        $("rErr").textContent = "–"; $("rSig").textContent = "–";
        objs.est.userData.known = false;
      }
      if (objs.planStep !== st.k) {
        if (objs.plan) { world.remove(objs.plan); objs.plan.geometry.dispose(); }
        objs.plan = st.plan_tool.length > 1 ? lineFrom(st.plan_tool, M.planOnline, true) : null;
        if (objs.plan) world.add(objs.plan);
        objs.planStep = st.k;
      }
      const lk = objs.arms.online.links[objs.arms.online.links.length - 1];
      objs.range.position.copy(lk.position);
      objs.range.scale.setScalar(Math.max(run.sensor_range, 1e-4));
    }
    applyVisibility();
    $("clock").textContent = "t = " + T.toFixed(2) + " s";
    $("timeTxt").textContent = T.toFixed(2) + " / " + Tend.toFixed(2) + " s";
    $("scrub").value = String(Math.round(1000 * T / Tend));
    movePlayhead();
    viewport.render();
    onTime?.(T);
  }

  function applyVisibility() {
    if (!objs) return;
    const which = root.querySelector('input[name="runs"]:checked').value;
    const showOn = which !== "blind", showBl = which !== "online";
    const mesh = $("lyMesh").checked, sph = $("lySph").checked, trail = $("lyTrail").checked;
    for (const [name, arm] of Object.entries(objs.arms)) {
      const vis = name === "online" ? showOn : showBl;
      arm.group.visible = vis && mesh;
      arm.sph.visible = vis && sph;
      if (arm.trail) { arm.trail.visible = vis && trail; arm.trailFull.visible = vis && trail; }
      if (name === "blind") {
        const ghost = showOn && R.online;
        arm.links.forEach((h) => h.children.forEach((m) => { m.material = ghost ? M.robotGhost : M.robot; }));
      }
    }
    const onVis = showOn && !!R.online;
    if (objs.plan) objs.plan.visible = onVis && $("lyPlan").checked;
    if (objs.offline) objs.offline.visible = $("lyOff").checked;
    const known = objs.est.userData.known;
    objs.est.visible = objs.estDot.visible = onVis && known && $("lyEst").checked;
    objs.range.visible = onVis && $("lyRange").checked && R.online && R.online.sensor_range > 0;
    objs.shell.visible = $("lyStand").checked;
  }

  // ---------- clearance chart ----------
  const SVGNS = "http://www.w3.org/2000/svg";
  let chartScale = null;
  function el(tag, attrs, parent) {
    const e = document.createElementNS(SVGNS, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(e);
    return e;
  }
  function niceStep(span) {
    const raw = span / 4, p = Math.pow(10, Math.floor(Math.log10(raw)));
    for (const m of [1, 2, 2.5, 5, 10]) if (raw <= m * p) return m * p;
    return 10 * p;
  }
  function drawChart() {
    const svg = $("chartSvg");
    svg.innerHTML = "";
    if (!D) return;
    const W = svg.clientWidth || 800, H = svg.clientHeight || 170;
    svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
    const m = { l: 44, r: 12, t: 10, b: 24 };
    const runs = Object.values(R);
    let lo = 0, hi = D.obstacle_standoff;
    runs.forEach((r) => r.frames.forEach((f) => { if (f.clear !== null) { lo = Math.min(lo, f.clear); hi = Math.max(hi, f.clear); } }));
    lo *= 100; hi *= 100;
    const st = niceStep(hi - lo);
    lo = Math.floor(lo / st) * st; hi = Math.ceil(hi / st) * st;
    const x = (t) => m.l + (W - m.l - m.r) * t / Tend;
    const y = (cm) => m.t + (H - m.t - m.b) * (hi - cm) / (hi - lo);
    chartScale = { x, y, m, W, H };
    const ink2 = css("--ink-2"), muted = css("--muted"), grid = css("--grid"), crit = css("--critical");
    // collision band
    if (lo < 0) el("rect", { x: m.l, y: y(0), width: W - m.l - m.r, height: y(lo) - y(0), fill: crit, "fill-opacity": 0.08 }, svg);
    for (let v = lo; v <= hi + 1e-9; v += st) {
      el("line", { x1: m.l, x2: W - m.r, y1: y(v), y2: y(v), stroke: grid, "stroke-width": 1 }, svg);
      const tx = el("text", { x: m.l - 6, y: y(v) + 4, "text-anchor": "end", "font-size": 11, fill: muted, "font-family": "IBM Plex Mono, monospace" }, svg);
      tx.textContent = (Math.round(v * 10) / 10) + "";
    }
    const yl = el("text", { x: 4, y: m.t + 8, "font-size": 11, fill: muted }, svg); yl.textContent = "cm";
    for (let t = 0; t <= Tend + 1e-9; t += niceStep(Tend)) {
      const tx = el("text", { x: x(t), y: H - 6, "text-anchor": "middle", "font-size": 11, fill: muted, "font-family": "IBM Plex Mono, monospace" }, svg);
      tx.textContent = t + " s";
    }
    el("line", { x1: m.l, x2: W - m.r, y1: y(0), y2: y(0), stroke: crit, "stroke-width": 1 }, svg);
    el("line", { x1: m.l, x2: W - m.r, y1: y(100 * D.obstacle_standoff), y2: y(100 * D.obstacle_standoff), stroke: muted, "stroke-width": 1, "stroke-dasharray": "4 3" }, svg);
    const sl = el("text", { x: m.l + 4, y: y(100 * D.obstacle_standoff) - 4, "text-anchor": "start", "font-size": 11, fill: ink2 }, svg);
    sl.textContent = "standoff " + (100 * D.obstacle_standoff).toFixed(0) + " cm";
    if (R.online && R.online.summary.first_detection >= 0) {
      const td = R.online.summary.first_detection * D.dt;
      el("line", { x1: x(td), x2: x(td), y1: m.t, y2: H - m.b, stroke: css("--online"), "stroke-width": 1, "stroke-dasharray": "2 3" }, svg);
      const dt = el("text", { x: x(td) + 4, y: m.t + 10, "font-size": 11, fill: ink2 }, svg);
      dt.textContent = "obstacle first seen";
    }
    for (const r of runs) {
      const d = r.frames.filter((f) => f.clear !== null).map((f, i) => (i ? "L" : "M") + x(f.t).toFixed(1) + " " + y(100 * f.clear).toFixed(1)).join("");
      el("path", { d, fill: "none", stroke: css(r.name === "online" ? "--online" : "--blind"), "stroke-width": 2, "stroke-linejoin": "round", "stroke-linecap": "round" }, svg);
      // direct label at the end of each line
      const last = r.frames[r.frames.length - 1];
      const lab = el("text", { x: x(last.t) - 4, y: y(100 * last.clear) + (r.name === "online" ? -6 : 14), "text-anchor": "end", "font-size": 11, fill: ink2 }, svg);
      lab.textContent = r.name === "online" ? "online" : "blind";
    }
    el("line", { id: "playhead", x1: x(T), x2: x(T), y1: m.t, y2: H - m.b, stroke: css("--ink"), "stroke-width": 1.5 }, svg);
  }
  function movePlayhead() {
    const p = $("playhead");
    if (p && chartScale) { const xx = chartScale.x(T); p.setAttribute("x1", xx); p.setAttribute("x2", xx); }
  }
  function chartTime(ev) {
    const svg = $("chartSvg"), r = svg.getBoundingClientRect();
    const { m, W } = chartScale;
    const px = (ev.clientX - r.left) * (W / r.width);
    return Math.max(0, Math.min(Tend, (px - m.l) / (W - m.l - m.r) * Tend));
  }
  function showTip(ev) {
    if (!chartScale || !D) return;
    const t = chartTime(ev), tip = $("tip");
    tip.replaceChildren();
    const heading = document.createElement('strong');
    heading.textContent = `t = ${t.toFixed(2)} s`;
    tip.append(heading);
    for (const run of Object.values(R)) {
      const { a, b, s } = frameAt(run, t);
      const row = document.createElement('div');
      row.textContent = `${run.label}: ${fmtCm(lerp(a.clear, b.clear, s))}`;
      tip.append(row);
    }
    tip.hidden = false;
    const box = $("chart").getBoundingClientRect();
    let left = ev.clientX - box.left + 12;
    if (left + tip.offsetWidth > box.width) left = ev.clientX - box.left - tip.offsetWidth - 12;
    tip.style.left = Math.max(0, left) + "px";
    tip.style.top = "6px";
  }

  // ---------- render loop ----------
  let last = 0;
  function tick(now) {
    const dt = Math.min(.1, (now - last) / 1000); last = now;
    if (playing && D) {
      let t = T + dt * speed;
      if (t >= Tend) { t = Tend; setPlaying(false); }
      setTime(t);
    }
  }
  function setPlaying(p) {
    playing = p;
    if (p) { last = performance.now(); viewport.start(tick); }
    else viewport.stop();
    $("play").textContent = p ? "❚❚ Pause" : "▶ Play";
    $("play").setAttribute("aria-label", p ? "Pause" : "Play");
  }

  function applyTheme() {
    const colors = { robot: '--robot', robotGhost: '--blind', obstacle: '--obstacle', shell: '--critical',
      est: '--online', estDot: '--online', range: '--online', sphOk: '--online', sphOkBlind: '--blind',
      sphWarn: '--serious', sphBad: '--critical', lineOnline: '--online', lineBlind: '--blind',
      lineOnlineFaint: '--online', lineBlindFaint: '--blind', planOnline: '--online', planOff: '--muted' };
    for (const [name, variable] of Object.entries(colors)) M[name].color.copy(col(variable));
    if (D) { setTime(T); drawChart(); }
  }

  // ---------- wiring ----------
  listen($("play"), "click", () => { if (T >= Tend) setTime(0); setPlaying(!playing); });
  listen($("restart"), "click", () => { setTime(0); setPlaying(true); });
  listen($("speed"), "change", (e) => { speed = parseFloat(e.target.value); });
  listen($("scrub"), "input", (e) => { setPlaying(false); setTime(Tend * e.target.value / 1000); });
  root.querySelectorAll('input[name="runs"], .layers input').forEach((i) => listen(i, "change", () => setTime(T)));
  const svg = $("chartSvg");
  let dragging = false;
  listen(svg, "pointerdown", (e) => { if (!chartScale) return; dragging = true; svg.setPointerCapture(e.pointerId); setPlaying(false); setTime(chartTime(e)); });
  listen(svg, "pointermove", (e) => { showTip(e); if (dragging) setTime(chartTime(e)); });
  listen(svg, "pointerup", () => { dragging = false; });
  listen(svg, "pointerleave", () => { $("tip").hidden = true; });
  listen(document, "keydown", (e) => {
    if (e.target.closest('input, select, textarea, button, a, [role="button"]')) return;
    if (e.code === "Space") { e.preventDefault(); $("play").click(); }
    if (e.code === "ArrowRight") setTime(T + (D ? D.dt / 4 : 0));
    if (e.code === "ArrowLeft") setTime(T - (D ? D.dt / 4 : 0));
  });

  function loadText(text) {
    if (disposed) return;
    try { showError(""); setPlaying(false); setReplay(JSON.parse(text), true); }
    catch (err) { showError("Could not load that file: " + err.message); }
  }
  listen($("file"), "change", (e) => {
    const f = e.target.files && e.target.files[0];
    if (f) f.text().then(loadText);
    e.target.value = "";
  });
  let dragDepth = 0;
  listen(window, "dragenter", (e) => { e.preventDefault(); dragDepth++; $("drop").hidden = false; });
  listen(window, "dragleave", () => { dragDepth = Math.max(0, dragDepth - 1); if (!dragDepth) $("drop").hidden = true; });
  listen(window, "dragover", (e) => e.preventDefault());
  listen(window, "drop", (e) => {
    e.preventDefault(); dragDepth = 0; $("drop").hidden = true;
    const f = e.dataTransfer.files && e.dataTransfer.files[0];
    if (f) f.text().then(loadText);
  });

  const chartObserver = new ResizeObserver(() => { if (D) drawChart(); });
  chartObserver.observe($("chart"));
  const stopTheme = onThemeChange(applyTheme);
  makeMaterials();
  (async () => {
    try {
      await loadMeshes();
      if (disposed) return;
      const data = window.SCATE_REPLAY || await fetch(new URL('replay.json', document.baseURI), { signal: loading.signal }).then(response => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return response.json();
      });
      if (!disposed) setReplay(data);
    } catch (error) {
      if (!disposed) showError(`Could not load the replay: ${error.message}`);
    }
  })();
  return {
    selectTime(time) { if (D) { setPlaying(false); setTime(time); } },
    dispose() {
      disposed = true; loading.abort(); events.abort(); stopTheme(); chartObserver.disconnect();
      clearWorld(); scene.remove(world); viewport.dispose();
      Object.values(meshGeo).forEach(geometry => geometry.dispose());
      Object.values(M).forEach(material => material.dispose());
    },
  };
}
