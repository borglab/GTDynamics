# CMRG dress-cable models

Learned cable models for the two BAR Lab CMRG arms, used by `gtd.MLP`, `gtd.NNCableSpline` and `gtd.RobotNNCableModel`.

- `cable_model_arm1_gripper.txt`: arm 1 with the timber gripper.
- `cable_model_arm2_nailer.txt`: arm 2 with the timber nailer.

Each file is an endpoint-constrained MLP (input: joints 2-6 of the arm, 16 Chebyshev nodes, 14 interior nodes x 3) with the `link_3` and `link_6` attachment points in each link's frame (metres). Cable collision radius in the paper planner is 3 in / 2.

The anchors stored in these files are not used by the STEAP Assembly app. Like the BAR Lab Grasshopper NN cable component, it uses the anchors and exit normals from the canvas (`CABLE_ANCHORS` and `CABLE_NORMALS` in `apps/steap-cmrg/scripts/cell.py`), which place the link 6 end in the holder's eyelet.

Source: BAR-robot-cell, `procedural/cable_tests/data/` (branch `cable_nn_paper`). Retrain and provenance details belong to that repository.
