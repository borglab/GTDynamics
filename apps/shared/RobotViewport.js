import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { CameraController } from './CameraController.js';

/** Dispose owned geometry/materials once; shared model geometry belongs to its owner. */
export function disposeObject3D(root) {
  const geometries = new Set(), materials = new Set(), textures = new Set();
  root.traverse(object => {
    if (object.geometry && !object.geometry.userData.sharedMujoco) geometries.add(object.geometry);
    for (const material of [object.material].flat()) if (material) materials.add(material);
  });
  for (const material of materials) {
    for (const value of Object.values(material)) if (value?.isTexture) textures.add(value);
    material.dispose();
  }
  textures.forEach(texture => texture.dispose());
  geometries.forEach(geometry => geometry.dispose());
}

/**
 * Shared Z-up Three.js viewport. Apps add robot visuals/overlays to scene and
 * supply body transforms themselves. No MuJoCo dependency or implicit physics.
 * Call start(callback) for continuous simulation, or render() after replay updates.
 */
export class RobotViewport {
  constructor(container, {
    background = '#f1f3ef', fog, fov = 35, near = .02, far = 80,
    pixelRatio = 2, shadows = false, exposure, controls = {},
    ariaLabel = 'Robot viewer; drag to orbit, scroll to zoom',
  } = {}) {
    this.container = container;
    this.disposed = false;
    this.lightBackground = background;
    this.stages = [];
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(background);
    if (fog) this.scene.fog = new THREE.Fog(background, ...fog);
    this.camera = new THREE.PerspectiveCamera(fov, 1, near, far);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, pixelRatio));
    this.renderer.outputEncoding = THREE.sRGBEncoding;
    if (exposure !== undefined) {
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = exposure;
    }
    this.renderer.shadowMap.enabled = shadows;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.domElement.setAttribute('aria-label', ariaLabel);
    this.renderer.domElement.style.display = 'block';
    container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    Object.assign(this.controls, { minDistance: .5, maxDistance: 20, maxPolarAngle: Math.PI * .49 }, controls);
    this.cameraController = new CameraController(this.camera, this.controls);
    this.controlsChanged = () => { if (!this.animation) this.render(); };
    this.controls.addEventListener('change', this.controlsChanged);
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this.themeChanged = () => this.applyTheme();
    window.addEventListener('gtdynamics-theme-change', this.themeChanged);
    this.applyTheme();
  }

  /** Add a standard lit stage; customize colors/dimensions without copying setup. */
  addStage({
    hemisphere = ['#f7fbff', '#a4b0a3', .65],
    key = { color: '#fff8e9', intensity: 1.1, position: [0, -3, 7] },
    fill = { color: '#d6e9ed', intensity: .35, position: [-4, 3, 4] },
    shadow = { left: -6, right: 6, bottom: -5, top: 5, near: .1, far: 20, bias: -.0005, normalBias: .015 },
    ground = { size: 70, color: '#edf0e9', z: -.01 },
    grid = { size: 30, divisions: 120, colors: ['#b5c2b7', '#d6ddd3'], z: -.007, opacity: .6 },
  } = {}) {
    this.scene.add(new THREE.HemisphereLight(...hemisphere));
    const light = new THREE.DirectionalLight(key.color, key.intensity);
    light.position.fromArray(key.position);
    light.castShadow = this.renderer.shadowMap.enabled;
    light.shadow.mapSize.set(2048, 2048);
    const { bias, normalBias, ...bounds } = shadow;
    Object.assign(light.shadow.camera, bounds);
    light.shadow.bias = bias ?? -.0005;
    light.shadow.normalBias = normalBias ?? 0;
    this.scene.add(light);
    if (fill) {
      const light = new THREE.DirectionalLight(fill.color, fill.intensity);
      light.position.fromArray(fill.position); this.scene.add(light);
    }
    let floor, lines;
    if (ground) {
      floor = new THREE.Mesh(new THREE.PlaneGeometry(ground.size, ground.size),
        new THREE.MeshStandardMaterial({ color: ground.color, roughness: 1 }));
      floor.position.z = ground.z; floor.receiveShadow = true; this.scene.add(floor);
    }
    if (grid) {
      lines = new THREE.GridHelper(grid.size, grid.divisions, ...grid.colors);
      lines.rotation.x = Math.PI / 2; lines.position.z = grid.z ?? 0;
      if (grid.opacity !== undefined) { lines.material.transparent = true; lines.material.opacity = grid.opacity; }
      this.scene.add(lines);
    }
    const stage = { ground: floor, grid: lines };
    this.stages.push({ ...stage, groundColor: ground?.color,
      gridColors: lines ? Float32Array.from(lines.geometry.attributes.color.array) : null,
      majorColor: grid ? new THREE.Color(grid.colors[0]) : null });
    this.applyTheme();
    return stage;
  }

  /** Recolor stage surfaces while preserving the camera, models and overlays. */
  applyTheme() {
    const dark = document.documentElement.dataset.theme === 'dark';
    const css = getComputedStyle(document.documentElement);
    const color = (name, fallback) => css.getPropertyValue(`--theme-${name}`).trim() || fallback;
    this.scene.background.set(dark ? color('scene', '#0c1218') : this.lightBackground);
    if (this.scene.fog) this.scene.fog.color.copy(this.scene.background);
    for (const stage of this.stages) {
      if (stage.ground) {
        stage.ground.material.color.set(dark ? color('ground', '#111b23') : stage.groundColor);
        if (dark) stage.ground.material.color.convertSRGBToLinear();
      }
      if (stage.grid) {
        const colors = stage.grid.geometry.attributes.color;
        if (!dark) colors.array.set(stage.gridColors);
        else {
          const major = new THREE.Color(color('grid-major', '#425660')).convertSRGBToLinear();
          const minor = new THREE.Color(color('grid-minor', '#293941')).convertSRGBToLinear();
          for (let i = 0; i < colors.count; i++) {
            const original = stage.gridColors.subarray(i * 3, i * 3 + 3);
            const isMajor = Math.abs(original[0] - stage.majorColor.r) < 1e-5 && Math.abs(original[1] - stage.majorColor.g) < 1e-5;
            const value = isMajor ? major : minor;
            colors.setXYZ(i, value.r, value.g, value.b);
          }
        }
        colors.needsUpdate = true;
      }
    }
    if (!this.animation) this.render();
  }

  resize() {
    if (this.disposed) return;
    const width = Math.max(1, this.container.clientWidth), height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height; this.camera.updateProjectionMatrix();
    this.cameraController.refit();
    this.renderer.setSize(width, height, false);
    this.onResize?.();
    if (!this.animation) this.render();
  }

  /** Callback receives performance.now()-style milliseconds and advances app state. */
  start(update) {
    this.animation = update;
    this.renderer.setAnimationLoop(now => {
      if (this.disposed) return;
      update(now); this.controls.update(); this.render();
    });
  }

  /** Pause continuous updates; controls and explicit render calls remain active. */
  stop() {
    this.renderer.setAnimationLoop(null);
    this.animation = null;
    this.render();
  }

  render() { if (!this.disposed) this.renderer.render(this.scene, this.camera); }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.renderer.setAnimationLoop(null);
    this.animation = null;
    this.resizeObserver.disconnect();
    window.removeEventListener('gtdynamics-theme-change', this.themeChanged);
    this.controls.removeEventListener('change', this.controlsChanged);
    this.controls.dispose();
    disposeObject3D(this.scene);
    this.renderer.dispose(); this.renderer.domElement.remove();
  }
}
