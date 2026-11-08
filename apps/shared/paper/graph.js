/** Small explanatory motifs, not exports of the complete optimization graphs. */
export function graphSlices(frame, samples) {
  const count = Math.min(3, Math.max(1, samples));
  const first = Math.max(0, Math.min(Math.round(frame) - 1, samples - count));
  return Array.from({ length: count }, (_, i) => first + i);
}

export function paperGraph(kind, indices) {
  const nodes = [], factors = [];
  const x = i => 140 + 270 * i;
  const node = (id, label, px, y, slice) => nodes.push({ id, label, x: px, y, slice });
  const factor = (code, title, keys, px, y) => factors.push({
    id: `f${factors.length}`, code, title, keys, x: px, y,
  });

  if (kind === 'urdf-pose' || kind === 'urdf-point') {
    const poseGoal = kind === 'urdf-pose';
    node('parent', 'Xₚ', 225, 102);
    node('child', 'X꜀', 660, 102);
    node('joint', 'qⱼ', 440, 190);
    factor('B', 'Fixed-link anchor, shown on a fixed parent', ['parent'], 100, 102);
    factor('K', 'Joint pose factor on the parent, child, and joint coordinate', ['parent', 'child', 'joint'], 440, 102);
    factor(poseGoal ? 'T' : 'P', poseGoal ? 'Full pose prior on the tool’s carrier link' : 'Point goal on the tool’s carrier link', ['child'], 790, 102);
    factor('Q', 'Warm-start joint prior', ['joint'], 320, 190);
    if (poseGoal) factor('L', 'Soft joint-limit penalty', ['joint'], 555, 190);
    return {
      nodes, factors, height: 235,
      caption: `One joint is shown with parent and child COM poses Xₚ and X꜀ and joint coordinate qⱼ. B applies only to fixed links; the target factor attaches only to the selected tool’s carrier link. ${poseGoal ? 'T is the full pose prior, and L is the joint-limit penalty. K and B include the equality residuals that the default LM solver adds as penalties.' : 'P is the position-only goal. This graph has no joint-limit factor.'} Every square is a finite-weight cost in the illustrated solve.`,
    };
  }

  if (kind === 'scate') {
    node('l', 'ℓ', 820, 110);
    indices.forEach((k, i) => {
      node(`q${i}`, `q${k}`, x(i), 120, k);
      node(`v${i}`, `v${k}`, x(i), 235, k);
      factor('Mq/Lq', 'Separate encoder and joint-limit unary factors', [`q${i}`], x(i) - 70, 75);
      factor('Mv/Lv', 'Separate velocity-observation and velocity-limit unary factors', [`v${i}`], x(i) - 70, 270);
      factor('Ml', 'Tool-frame landmark observation when detected', [`q${i}`, 'l'], x(i) + 45, 40);
      if (i + 1 < indices.length) {
        node(`u${i}`, `u${k}`, x(i) + 135, 355, k);
        factor('Lu/E', 'Separate acceleration-limit and optional effort unary factors', [`u${i}`], x(i) + 135, 415);
        factor('D', 'Double integrator on two states and one acceleration control', [`q${i}`, `v${i}`, `q${i + 1}`, `v${i + 1}`, `u${i}`], x(i) + 135, 230);
        factor('O/S', 'GP obstacle and self-collision factors on a future interval', [`q${i}`, `v${i}`, `q${i + 1}`, `v${i + 1}`], x(i) + 135, 155);
      }
    });
    return { nodes, factors, height: 450, caption: 'q and v are six-dimensional joint states; u is a six-dimensional acceleration control; ℓ is one world-frame obstacle centre. All factors are soft. Mq/Lq, Mv/Lv, Lu/E and O/S group separate factors with matching scopes. Ml exists only after a detection; O/S applies only to future intervals, and Lu/E is removed after execution. O uses a fixed estimated centre and has no edge to ℓ. Endpoint priors are omitted.' };
  }

  if (kind === 'labels') {
    indices.forEach((k, i) => {
      node(`l${i}`, `L${k}`, x(i), 100, k);
      node(`r${i}`, `R${k}`, x(i), 250, k);
      factor('V', 'Unary left-foot velocity measurement', [`l${i}`], x(i), 45);
      factor('V', 'Unary right-foot velocity measurement', [`r${i}`], x(i), 310);
      factor('C/H', 'Soft support coupling / optional hard support mask', [`l${i}`, `r${i}`], x(i), 175);
      if (i) {
        factor('D', 'Left-foot two-displacement measurement', [`l${i - 1}`, `l${i}`], x(i) - 135, 100);
        factor('D', 'Right-foot two-displacement measurement', [`r${i - 1}`, `r${i}`], x(i) - 135, 250);
        factor('A/E', 'Four-variable transition / optional hard exchange mask', [`l${i - 1}`, `r${i - 1}`, `l${i}`, `r${i}`], x(i) - 135, 175);
      }
    });
    if (indices.length === 3) {
      factor('ψ', 'Left-foot three-slice duration prior', ['l0', 'l1', 'l2'], 410, 15);
      factor('ψ', 'Right-foot three-slice duration prior', ['r0', 'r1', 'r2'], 410, 365);
    }
    return { nodes, factors, height: 405, caption: 'L and R are binary stance/swing variables, not three-valued support labels. C/H and A/E group separate factors with identical scopes. D observes two displacements; ψ spans three slices. The uniform initial pair prior is omitted.' };
  }

  if (kind === 'dynamics') {
    indices.forEach((k, i) => {
      node(`z${i}`, `Z${k}`, x(i), 50, k);
      node(`a${i}`, `a${k}`, x(i) - 55, 170, k);
      node(`w${i}`, `W${k}`, x(i) + 55, 170, k);
      node(`t${i}`, `τ${k}`, x(i) - 50, 310, k);
      node(`c${i}`, `λ${k}`, x(i) + 50, 310, k);
      factor('F', 'ConstVarFactor: fixed configurations and velocities', [`z${i}`], x(i) - 70, 50);
      factor('A', 'Root and joint acceleration priors', [`a${i}`], x(i) - 100, 170);
      factor('K', 'Articulated acceleration equalities', [`z${i}`, `a${i}`], x(i) - 35, 100);
      factor('N', 'Exact Newton–Euler balance and joint-wrench equalities', [`z${i}`, `a${i}`, `w${i}`, `t${i}`], x(i) + 25, 115);
      factor('M', 'Sole-to-link wrench mapping', [`w${i}`, `c${i}`], x(i) + 55, 245);
      factor('H', 'Rectangular contact-wrench inequalities', [`c${i}`], x(i) + 115, 310);
      factor('R', 'Effort / wrench magnitude priors (separate unary factors)', [`t${i}`, `c${i}`], x(i), 365);
      if (i) {
        factor('Δτ', 'Torque temporal regularization', [`t${i - 1}`, `t${i}`], x(i) - 170, 260);
        factor('Δλ', 'Wrench temporal regularization', [`c${i - 1}`, `c${i}`], x(i) - 100, 345);
      }
    });
    return { nodes, factors, height: 405, caption: 'Each circle groups variables of one type in a slice: Z fixed configurations/velocities, a link and joint accelerations, W internal link wrenches, τ actuator torques, λ the two sole wrenches. A groups acceleration observations; K groups articulated acceleration equalities; N groups exact balance and joint-wrench constraints. R groups separate unary priors. The motif shows continuing stance: Δλ connections end at lift-off. Inactive-contact zero constraints and optional actuator bounds are omitted.' };
  }

  if (kind === 'kinodynamics') {
    node('b', 'b', 820, 65);
    node('c', 'C', 820, 510);
    indices.forEach((k, i) => {
      node(`z${i}`, `Z${k}`, x(i), 145, k);
      node(`a${i}`, `a${k}`, x(i) - 45, 280, k);
      node(`w${i}`, `W${k}`, x(i) + 45, 375, k);
      node(`l${i}`, `λ${k}`, x(i), 475, k);
      factor('K', 'Exact articulated pose/velocity and body-to-world velocity equalities', [`z${i}`], x(i) - 60, 85);
      factor('J', 'Unary encoder measurements', [`z${i}`], x(i) + 65, 145);
      factor('F', 'Sole-pose observation to uninterrupted-stance landmark', [`z${i}`, 'c'], x(i) + 90, 230);
      factor('S', 'Zero sole-origin velocity cost in stance', [`z${i}`], x(i) - 70, 190);
      factor('A', 'Broad root/joint acceleration priors', [`a${i}`], x(i) - 95, 280);
      factor('N', 'Exact acceleration kinematics, Newton–Euler balance and actuator-wrench equalities', [`z${i}`, `a${i}`, `w${i}`, `l${i}`], x(i), 335);
      factor('H', 'Exact stance wrench inequalities or zero swing wrench', [`l${i}`], x(i) - 70, 475);
      factor('R', 'Separate unary torque and sole-wrench regularization', [`w${i}`, `l${i}`], x(i) + 65, 450);
      if (i) {
        factor('I', 'Gal3 IMU factor: two root poses, two world velocities, shared bias', [`z${i - 1}`, `z${i}`, 'b'], x(i) - 135, 65);
        factor('T', 'Separate configuration and velocity trapezoidal collocation factors', [`z${i - 1}`, `z${i}`, `a${i - 1}`, `a${i}`], x(i) - 135, 240);
        factor('Δτ', 'Consecutive actuator torque differences', [`w${i - 1}`, `w${i}`], x(i) - 90, 400);
        factor('Δλ', 'Sole-wrench differences within a continuing stance', [`l${i - 1}`, `l${i}`], x(i) - 135, 520);
      }
    });
    factor('Pb', 'Shared sensor-bias prior', ['b'], 820, 15);
    return { nodes, factors, height: 565, caption: 'Z groups free link COM poses, joint angles, body twists, joint rates and world root velocity; a groups link/joint accelerations; W groups internal reaction wrenches and actuator torques; λ groups both sole wrenches. C groups the current stance landmarks; b is one shared IMU bias. K, N and H are hard constraints; J, F, S, A, R, I, T, Δτ, Δλ and Pb are soft factors. T and R group separate native factors; each IMU factor still has two poses and two velocities inside the Z groups. This motif shows continuing double stance; contact observations and wrench-rate edges end at lift-off. The translation/yaw gauge costs and initial-velocity prior are omitted.' };
  }

  if (kind === 'planning') {
    indices.forEach((k, i) => {
      node(`q${i}`, `q${k}`, x(i), 80, k);
      node(`v${i}`, `q̇${k}`, x(i), 195, k);
      node(`t${i}`, `T${k}`, x(i), 310, k);
      node(`w${i}`, `V${k}`, x(i), 425, k);
      factor('Kq', 'Position kinematics', [`q${i}`, `t${i}`], x(i) - 65, 195);
      factor('Kv', 'Velocity kinematics', [`q${i}`, `v${i}`, `t${i}`, `w${i}`], x(i) + 65, 250);
      factor('P', 'Posture and joint-limit unary costs', [`q${i}`], x(i), 25);
      if (i) {
        factor('Iq', 'Four-variable joint trapezoidal integration', [`q${i - 1}`, `v${i - 1}`, `q${i}`, `v${i}`], x(i) - 135, 130);
        factor('IT', 'Four-variable pose/twist trapezoidal integration', [`t${i - 1}`, `w${i - 1}`, `t${i}`, `w${i}`], x(i) - 135, 365);
        factor('S', 'Joint-velocity smoothness', [`v${i - 1}`, `v${i}`], x(i) - 135, 210);
      }
    });
    return { nodes, factors, height: 465, caption: 'Circles group joint or link variables within one time slice. Kq and Kv denote the within-slice position/velocity factors; Iq and IT are temporal integration factors. P groups separate unary costs. Endpoint hand-pose and rest priors attach only at the global endpoints and are omitted from this local motif. All are soft factors in this LM example.' };
  }

  // Pose initialization and kinematic smoothing share stance landmarks.
  node('cl', 'Cᴸ', 280, 345);
  node('cr', 'Cᴿ', 560, 345);
  const smoothing = kind === 'smoothing';
  if (smoothing) node('b', 'b', 800, 260);
  indices.forEach((k, i) => {
    node(`z${i}`, `${smoothing ? 'Z' : 'X'}${k}`, x(i), 95, k);
    factor('F', 'Left sole-pose observation', [`z${i}`, 'cl'], x(i) - 55, 235);
    factor('F', 'Right sole-pose observation', [`z${i}`, 'cr'], x(i) + 55, 270);
    factor(smoothing ? 'KJU' : 'U', smoothing ? 'Kinematic equalities, encoder measurements, optional upright prior' : 'Optional upright prior', [`z${i}`], x(i), 30);
    if (smoothing) {
      node(`v${i}`, `v${k}`, x(i), 175, k);
      if (i) factor('I', 'Five-variable Gal3 IMU factor', [`z${i - 1}`, `v${i - 1}`, `z${i}`, `v${i}`, 'b'], x(i) - 135, 135);
    } else if (i) {
      factor('M', 'Optional weak root-motion prior', [`z${i - 1}`, `z${i}`], x(i) - 135, 95);
    }
  });
  factor('H', 'Optional left contact-height prior', ['cl'], 280, 400);
  factor('H', 'Optional right contact-height prior', ['cr'], 560, 400);
  if (smoothing) factor('Pb', 'Shared IMU bias prior', ['b'], 800, 325);
  return { nodes, factors, height: 440, caption: `${smoothing ? 'Z groups all link poses and joint angles; v is world COM velocity; b is the shared IMU bias. KJU groups separate kinematic, encoder, and optional upright factors.' : 'X is a root COM pose. U and M are optional upright and motion priors.'} Cᴸ and Cᴿ are world sole-pose landmarks. This local motif shows a double-stance segment; lift-off ends a landmark. Gauge, initial-velocity, optional normal, and final-position priors are omitted.` };
}
