import * as THREE from 'three';

/** Build Z-up robot visuals directly from a compiled MuJoCo model. */
export function createRobotVisual(model) {
  const bodies = Array.from({ length: model.nbody }, () => new THREE.Group());
  const root = new THREE.Group();
  root.add(...bodies);
  const meshes = new Map();
  for (let g = 0; g < model.ngeom; g += 1) {
    // Menagerie uses group 2 for visuals and group 3 for collision geometry.
    if (model.geom_group[g] >= 3) continue;
    const type = model.geom_type[g];
    const [x, y, z] = model.geom_size.slice(3 * g, 3 * g + 3);
    let geometry;
    if (type === 7) {
      const id = model.geom_dataid[g];
      if (!meshes.has(id)) {
        const mesh = new THREE.BufferGeometry();
        mesh.setAttribute('position', new THREE.BufferAttribute(new Float32Array(model.mesh_vert.slice(
          3 * model.mesh_vertadr[id], 3 * (model.mesh_vertadr[id] + model.mesh_vertnum[id]))), 3));
        mesh.setIndex(Array.from(model.mesh_face.slice(
          3 * model.mesh_faceadr[id], 3 * (model.mesh_faceadr[id] + model.mesh_facenum[id]))));
        mesh.computeVertexNormals();
        meshes.set(id, mesh);
      }
      geometry = meshes.get(id);
    } else if (type === 2 || type === 4) {
      geometry = new THREE.SphereGeometry(type === 2 ? x : 1, 20, 16);
      if (type === 4) geometry.scale(x, y, z);
    } else if (type === 3 || type === 5) {
      geometry = type === 3 ? new THREE.CapsuleGeometry(x, 2 * y, 8, 16)
        : new THREE.CylinderGeometry(x, x, 2 * y, 20);
      geometry.rotateX(Math.PI / 2);
    } else if (type === 6) {
      geometry = new THREE.BoxGeometry(2 * x, 2 * y, 2 * z);
    } else continue;
    const materialId = model.geom_matid[g];
    const rgba = materialId >= 0 ? model.mat_rgba.slice(4 * materialId, 4 * materialId + 4)
      : model.geom_rgba.slice(4 * g, 4 * g + 4);
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]).convertSRGBToLinear(),
      opacity: rgba[3], transparent: rgba[3] < 1, metalness: 0.12, roughness: 0.48,
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.fromArray(model.geom_pos, 3 * g);
    mesh.quaternion.set(model.geom_quat[4 * g + 1], model.geom_quat[4 * g + 2],
      model.geom_quat[4 * g + 3], model.geom_quat[4 * g]);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    bodies[model.geom_bodyid[g]].add(mesh);
  }
  return { root, bodies };
}

/** Copy MuJoCo body transforms without changing coordinate conventions. */
export function updateRobotBodies(bodies, data) {
  bodies.forEach((body, i) => {
    body.position.fromArray(data.xpos, 3 * i);
    body.quaternion.set(data.xquat[4 * i + 1], data.xquat[4 * i + 2],
      data.xquat[4 * i + 3], data.xquat[4 * i]);
  });
}
