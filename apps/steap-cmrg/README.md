# STEAP Assembly

Two gantry-mounted ABB IRB6620 arms of the BAR Lab CMRG build a post-and-lintel frame from 2×4 stock staged far from the build site on the same table. Each maneuver is planned offline with GPMP2 (RRT-Connect seed, learned dress-cable factors), then executed with simulated disturbances and replanned by STEAP: measurement priors fused in an ISAM2 Bayes tree. The browser replays the recording.

## Run

```bash
cd apps
npm ci
npm run dev        # http://127.0.0.1:3000/ then open the STEAP Assembly tab
# or only this app:
cd steap-cmrg && npm run dev   # http://127.0.0.1:3010/
```

## Regenerate the recording

Requires the GTDynamics Python wrapper (GPMP2, SDF and cable bindings), `numpy`, `scipy` and `trimesh`.

```bash
python scripts/export_recording.py --preset portal   # portal | double-portal | skewed-portal
```

Options: `--seed`, `--gantry-sigma-mm`, `--arm-sigma-deg` (execution disturbance), `--lm-iterations`, `--max-segments`. A full build takes tens of minutes.

## Files

- `scripts/`: `cell.py` (joints, tools, FK), `collision.py` (sphere models), `scene.py` (floor/wood SDFs), `structure.py` (specs, 2×4 members, stock), `rrt.py`, `planner.py` (GPMP2), `steap.py` (ISAM2 execution), `sequence.py` (maneuvers), `export_recording.py`, `urdf_display.py` (browser geometry).
- `public/`: `model.json.gz` (display geometry), `recording-<preset>.json.gz`, `presets.json`, `MODEL_NOTICE.txt`. Large files are in Git LFS.
- Assets come from `models/urdfs/bar_lab_tools.urdf`, `models/urdfs/bar_*`, `models/cable/`.
