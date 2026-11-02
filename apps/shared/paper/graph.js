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
