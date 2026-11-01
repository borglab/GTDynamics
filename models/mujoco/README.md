# MuJoCo models

Large model XML and meshes are downloaded, not committed. Run from the repository
root:

```bash
python scripts/bootstrap_mujoco_models.py
```

This fetches `unitree_g1` from MuJoCo Menagerie commit
`a03e87bf13502b0b48ebbf2808928fd96ebf9cf3` into `.cache/mujoco_menagerie/` in this
directory. It creates `unitree_g1` here and the Contact Initialization app's asset
link, both pointing to that single checkout. The retargeted-motion importer also
uses this model. XML and meshes are used without modification; the built website
includes the model license.

Motion Planning bootstraps its Panda model with its own script. The SSIO
Kinematics and Contact vs IK recordings use the MJLab G1 model captured with
their offline data, rather than this Menagerie G1.
