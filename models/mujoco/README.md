# MuJoCo models

G1 uses `mujoco-menagerie==2026.10.6`, entry `unitree_g1/g1_mjlab`,
`mjlab==1.6.0`, and `mujoco==3.11.0`. Activate Python 3.12, install a suitable
PyTorch build, then run from the repository root:

```sh
python -m pip install -r examples/g1_mjlab/requirements-model.txt
python scripts/bootstrap_mujoco_models.py
```

`examples/g1_mjlab/g1_model.py` obtains `MjSpec` and mesh bytes through the
Menagerie Python API, supplies that spec to MJLab's public G1 factory, and uses
MJLab's scene exporter. No source checkouts or raw GitHub model downloads are
required. Simulation options are explicitly copied to the exported scene.

Generated XML, assets, binary cache, contact-frame metadata and a fingerprinted
provenance manifest live in `models/mujoco/.cache/g1-mjlab`. The model link and
browser assets link point there. The BSD model license accompanies the assets.
One sole frame per foot is derived independently from its foot site and capsule
lower envelope. It uses body axes (+X forward, +Y left, +Z up).

G1 kinematics, IMU aiding, SSIO import and browser export use this canonical
scene. Existing dynamics benchmark binaries/results retain their original
model and oracle settings for reproducibility. The older point-initialization
benchmark remains a point-contact client; its viewer now uses the canonical G1.
Motion Planning's Panda bootstrap is independent and unchanged.
