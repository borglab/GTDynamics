"""Paper-style definitions of the support factors, typeset by MathJax."""

from __future__ import annotations

import json
import gzip
import math
from pathlib import Path
import subprocess
from typing import Any


def write_factor_report(path: Path, parameters: dict[str, Any]) -> None:
    """Render definitions and parameter examples, without recording-level data."""
    names = (
        "supportSigma",
        "swingSigma",
        "velocityScale",
        "minimumProbability",
        "transition",
        "sampleIntervalSeconds",
        "initialPrior",
        "noSupportWeight",
        "singleSliceWeight",
        "enforceWalkingSequence",
    )
    params = {name: parameters[name] for name in names}
    sigma_s, sigma_w = params["supportSigma"], params["swingSigma"]
    rows = []
    for distance_mm in (0, 3, 10, 20):
        distance = distance_mm / 1000
        narrow = (
            math.sqrt(2 / math.pi)
            / sigma_s
            * math.exp(-0.5 * (distance / sigma_s) ** 2)
        )
        broad = (
            math.sqrt(2 / math.pi)
            / sigma_w
            * math.exp(-0.5 * (distance / sigma_w) ** 2)
        )
        rows.append(
            f"<tr><td>{distance_mm}</td><td>{narrow:.6g}</td><td>{broad:.6g}</td><td>{narrow/broad:.6g}</td></tr>"
        )
    velocity_rows = []
    for velocity in (-0.5, -0.3, -0.1, 0, 0.1, 0.3, 0.5):
        support = min(
            1 - params["minimumProbability"],
            max(
                params["minimumProbability"],
                1 / (1 + math.exp(velocity / params["velocityScale"])),
            ),
        )
        velocity_rows.append(
            f"<tr><td>{velocity:+.1f}</td><td>{support:.6f}</td><td>{1-support:.6f}</td></tr>"
        )
    pair_logs = [
        -0.5 * math.log(a * b) - 0.25 * (0.020 / a) ** 2
        for a in (sigma_w, sigma_s)
        for b in (sigma_w, sigma_s)
    ]
    pair = [math.exp(value - max(pair_logs)) for value in pair_logs]
    # Describe the actual published default, without retaining per-factor data.
    replay_path = (
        Path(__file__).resolve().parents[2]
        / "kinematics-inspector/public/variants/height-imu-loop/final.json.gz"
    )
    replay = json.loads(gzip.decompress(replay_path.read_bytes()))
    run = replay["runs"]["contact"]
    imu_group = next(
        group for group in replay["graph"]["factorGroups"] if group["id"] == "imu"
    )
    imu_error = run["final"]["factorErrors"]["imu"]
    kinematic_model = {
        "source": "Kinematics default: contact initialization, height prior, IMU and loop closure",
        "sourceSlices": replay["contactPhaseInference"]["sourceSlices"],
        "phases": len(replay["contactPhaseInference"]["phases"]),
        "doubleStancePhases": sum(
            len(p["contacts"]) == 2 for p in replay["contactPhaseInference"]["phases"]
        ),
        "slices": replay["samples"],
        "stanceLandmarks": replay["graph"]["stanceLandmarks"],
        "contactFactors": next(
            g["count"] for g in replay["graph"]["factorGroups"] if g["id"] == "contact"
        ),
        "maxIterations": replay["maxIterations"],
        "iterations": run["solver"]["iterations"],
        "factorGroups": replay["graph"]["factorGroups"],
        "imuCost": imu_error,
        "imuDimensions": imu_group["residualDimension"],
        "imuWhitenedRms": math.sqrt(2 * imu_error / imu_group["residualDimension"]),
        "totalCost": run["final"]["metrics"]["cost"],
        "referencePositionRmsMeters": run["final"]["metrics"][
            "reference_root_translation_rms_m"
        ],
        "accelerometerNoiseDensity": replay["imuAiding"]["accelerometerNoiseDensity"],
        "gyroscopeNoiseDensityDegrees": math.degrees(
            replay["imuAiding"]["gyroscopeNoiseDensity"]
        ),
        "imuBiasPriorSigmas": replay["imuAiding"]["biasPriorSigmas"],
    }
    substitutions = {
        "__KINEMATIC_MODEL__": json.dumps(
            kinematic_model, separators=(",", ":"), allow_nan=False
        ),
        "__SOURCE_SLICES__": f"{kinematic_model['sourceSlices']:,}",
        "__PHASES__": str(kinematic_model["phases"]),
        "__DOUBLE_PHASES__": str(kinematic_model["doubleStancePhases"]),
        "__SLICES__": str(kinematic_model["slices"]),
        "__LANDMARKS__": str(kinematic_model["stanceLandmarks"]),
        "__CONTACT_FACTORS__": str(kinematic_model["contactFactors"]),
        "__MAX_ITERATIONS__": str(kinematic_model["maxIterations"]),
        "__IMU_COST__": f"{imu_error:,.2f}",
        "__IMU_RMS__": f"{kinematic_model['imuWhitenedRms']:.3f}",
        "__TOTAL_COST__": f"{kinematic_model['totalCost']:,.2f}",
        "__REFERENCE_RMS__": f"{kinematic_model['referencePositionRmsMeters']:.2f}",
        "__GYRO_DENSITY__": f"{kinematic_model['gyroscopeNoiseDensityDegrees']:.3f}",
        "__PAIR_EXAMPLE__": r" \\ ".join(
            " & ".join(f"{value:.6g}" for value in pair[start : start + 2])
            for start in (0, 2)
        ),
        "__MODEL__": json.dumps(params, separators=(",", ":"), allow_nan=False),
        "__SIGMA_S__": f"{sigma_s:.3f}",
        "__SIGMA_W__": f"{sigma_w:.3f}",
        "__VELOCITY_SCALE__": f"{params['velocityScale']:g}",
        "__FLOOR__": f"{params['minimumProbability']:g}",
        "__DURATION__": f"{params['singleSliceWeight']:g}",
        "__DURATION_COST__": f"{-math.log(params['singleSliceWeight']):.6f}",
        "__NO_SUPPORT__": f"{params['noSupportWeight']:g}",
        "__DT__": f"{params['sampleIntervalSeconds']:g}",
        "__TRANSITION__": r" \\ ".join(
            " & ".join(f"{v:.4f}" for v in row) for row in params["transition"]
        ),
        "__DISTANCE_ROWS__": "".join(rows),
        "__VELOCITY_ROWS__": "".join(velocity_rows),
    }
    html = TEMPLATE
    for key, value in substitutions.items():
        html = html.replace(key, value)
    result = subprocess.run(
        ["node", str(Path(__file__).with_name("typeset_report.mjs"))],
        input=html,
        stdout=subprocess.PIPE,
        text=True,
        check=True,
    )
    path.write_text(result.stdout)


TEMPLATE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 contact factors: support inference, initialization, and kinematic smoothing</title>
<style>
figure{margin:26px 0;break-inside:avoid}figure img{display:block;width:100%;height:auto}figcaption{font:14px/1.5 system-ui,sans-serif;margin-top:10px}body{max-width:980px;margin:44px auto;padding:0 24px;color:#171717;background:white;font:18px/1.65 Georgia,"Times New Roman",serif}h1{font-size:30px;line-height:1.25;text-align:center;font-weight:normal}h2{font-size:22px;line-height:1.3;margin-top:32px}header p{text-align:center;font:14px/1.5 system-ui,sans-serif;color:#555}.abstract{font-size:17px;margin:28px 16px}table{margin:20px auto;border-collapse:collapse;font-size:16px;font-variant-numeric:tabular-nums}th,td{padding:7px 18px;text-align:right;border-bottom:1px solid #ccc}thead{border-top:2px solid #555;border-bottom:1px solid #555}caption{text-align:left;margin-bottom:8px;font-size:15px}mjx-container[display="true"]{overflow-x:auto;overflow-y:hidden;padding:4px 0}footer{margin-top:36px;border-top:1px solid #bbb;font-size:13px;line-height:1.5;color:#555}a{color:#264d72}@media print{body{margin:0;max-width:none;padding:0;font-size:11pt}h1{font-size:20pt}h2{font-size:14pt;break-after:avoid}table{break-inside:avoid}mjx-container{break-inside:avoid}}
</style></head><body><article>
<header><h1>G1 contact factors: support inference, initialization, and kinematic smoothing</h1><p>GTDynamics · Model definition and implemented parameter values</p></header>
<p class="abstract"><b>Abstract.</b> We infer binary contact states for each foot at each kinematic slice. Each foot has a binary displacement factor conditioned on two successive measured displacements, and a unary factor conditioned on pelvis-relative forward velocity. A within-slice factor favors having at least one foot in stance, while a four-variable factor models temporal transitions. A three-slice prior discourages isolated stance or swing classifications. Exact max-product gives a joint MAP sequence. Parameters are specified assumptions, not learned from contact annotations.</p>

<h2>1. Slices, variables, and measurements</h2>
<p>Index \(k=0,\ldots,N-1\) denotes a <b>kinematic slice</b>. Each slice has two binary variables \(c_{\ell,k},c_{r,k}\in\{0,1\}\), where 0 means swing (W) and 1 means stance (S). Neither variable represents an interval. Samples are separated by \(\Delta t=__DT__\,\mathrm{s}\). Native keys are \(2k\) for the left foot and \(2k+1\) for the right foot.</p>
<p>For foot \(f\), joint-encoder FK composed with the SuperOdom LiDAR-inertial pelvis reference gives the world sole-origin position \(p^W_{f,k}\). Define the incoming displacement</p>
\[
 d_{f,k}=\lVert p^W_{f,k}-p^W_{f,k-1}\rVert_2,\qquad k=0,\ldots,N-1.
\tag{1}
\]
<p>A preceding pose \(p^W_{f,-1}\) is supplied to compute the first displacement; it has no contact variable in this graph. Distances are in metres, not velocities. A displacement factor on slices \(k,k+1\) observes both \(d_{f,k}\) and \(d_{f,k+1}\), requiring three consecutive observed poses. Associating incoming motion with the ending slice is a measurement convention; touchdown or lift-off inside a sampling interval can cause timing ambiguity.</p>
<p>The other measurement, \(v_{f,k}\), is the signed forward velocity of the sole origin relative to the pelvis, expressed in pelvis axes (+X forward, +Y left, +Z up), in m/s. It uses measured joint velocities, not a LiDAR trajectory derivative.</p>

<h2>2. Factorization, support coupling, and transitions</h2>
<figure id="support-factor-graph"><img src="__SUPPORT_GRAPH__" alt="fgz factor graph: two binary foot variables per slice with displacement, velocity, support, transition, and duration factors"><figcaption>Figure 1. Three slices of the discrete support graph, rendered with fgz. Circles are variables; squares are factors. For readability CH denotes the product of the separate support coupling C and optional hard support factor H; AE denotes the product of the separate four-variable transition A and optional hard exchange mask E. D, V and ψ are displacement, velocity and duration factors. Removing the optional hard factors removes H and E only.</figcaption></figure>
<p>Let \(z_k=(c_{\ell,k},c_{r,k})\), a shorthand for two variables rather than an additional variable. With measurements fixed,</p>
\[
 P(c\mid d,v)=\frac{1}{Z(d,v)}\phi_0(z_0)
 \prod_{k=0}^{N-1}\left[C(z_k)\prod_f V_{f,k}(c_{f,k})\right]
 \prod_{k=0}^{N-2}\left[A(z_k,z_{k+1})\prod_f D_{f,k}(c_{f,k},c_{f,k+1})\right]
 \prod_{k=1}^{N-2}\prod_f \psi(c_{f,k-1},c_{f,k},c_{f,k+1}).
\tag{2}
\]
<p>The initial pair prior is uniform, \(\phi_0=1/4\). A binary factor at each slice penalizes simultaneous swing:</p>
\[
 C(c_\ell,c_r)=\begin{cases}\eta,&(c_\ell,c_r)=(0,0),\\1,&\text{otherwise},\end{cases}
 \qquad \eta=__NO_SUPPORT__.
\]
<p>Thus if one foot is in swing, stance of the other receives \(1/\eta=100\) times the coupling weight. If one foot is already in stance, the factor is neutral about the other. Without the optional hard constraints, both-swing remains possible; it means neither foot is classified in stance, not verified flight.</p>
<p>The temporal factor connects all four variables at two adjacent slices. Its rows and columns use joint order (WW, WS, SW, SS), with left foot first:</p>
\[
 A=\begin{pmatrix}__TRANSITION__\end{pmatrix}.
\tag{3}
\]
<p>The default is the product of independent per-foot transition probabilities: 0.97 to persist and 0.03 to switch at 50 Hz. Thus zero, one, or two foot switches receive 0.9409, 0.0291, or 0.0009, respectively. The API accepts a general positive row-stochastic 4 by 4 matrix. This is a transition potential inside the conditional model; additional measurements and coupling change posterior transition probabilities.</p>

<p>A duration prior connects three consecutive slices of one foot:</p>
\[
 \psi(a,b,c)=\begin{cases}\rho,&a=c\ne b,\\1,&\text{otherwise},\end{cases}
 \qquad \rho=__DURATION__.
\tag{3a}
\]
<p>In lexicographic W/S order (WWW, WWS, WSW, WSS, SWW, SWS, SSW, SSS), its eight entries are
\([1,1,\rho,1,1,\rho,1,1]\). WSW is an isolated stance slice; SWS is an isolated swing slice. Each receives an extra negative-log cost of \(-\log\rho=__DURATION_COST__\), in addition to the existing transition costs. Every other triple, including a sustained switch, receives weight 1.</p>
<p>At 50 Hz this discourages one-slice (20 ms) classifications. It is a soft prior, not a hard minimum duration: strong measurements can override it. It does not penalize a two-slice event or an event at the unobserved sequence boundary. Context slices protect the displayed boundaries. There are two factors per interior slice, one per foot. The weight is exposed as <code>ContactSupportParams.singleSliceWeight</code>, with valid range (0,1]; setting it to 1 makes the prior neutral. All four app graph variants use the prior.</p>

<p><b>Optional hard walking constraints (enabled by default).</b> The native function <code>contactWalkingConstraints(N)</code> returns additional factors; it does not modify any existing factor. A binary factor within each slice is</p>
\[
 H(c_\ell,c_r)=\begin{cases}0,&c_\ell=c_r=0,\\1,&\text{otherwise}.\end{cases}
\tag{3b}
\]
<p>A four-variable factor across adjacent slices excludes direct left-only/right-only exchanges. In joint configuration order (WW, WS, SW, SS), its table is</p>
\[
 E=\begin{pmatrix}1&1&1&1\\1&1&0&1\\1&0&1&1\\1&1&1&1\end{pmatrix}.
\tag{3c}
\]
<p>The within-slice factors independently exclude WW. The strict model is obtained by adding these factors to the graph:</p>
\[
 P_{\mathrm{strict}}(c\mid d,v)\propto P(c\mid d,v)
 \prod_{k=0}^{N-1}H(c_{\ell,k},c_{r,k})
 \prod_{k=0}^{N-2}E(z_k,z_{k+1}).
\tag{3d}
\]
<p>The graph multiplies all factors automatically. Forbidden entries are exactly zero and never receive the measurement likelihood floor. Left-only support can therefore change to right-only support only through at least one double-stance slice, and vice versa. This is a constraint on slice labels, not a prescribed physical duration. Continuous support and double-stance exchanges are assumptions imposed on the inference, not conclusions established by the measurements.</p>
<p>The app checkbox <em>Require support and double-stance transitions</em> is checked initially. Turning it off selects the original unconstrained solution, retaining all soft factors. Both constraint settings are exported with and without velocity aiding; switching either option performs no browser inference.</p>

<h2>3. Binary displacement measurement factor</h2>
<p>We use a half-normal density on the nonnegative displacement magnitude:</p>
\[
 h(d;\sigma)=\sqrt{\frac{2}{\pi}}\frac{1}{\sigma}
 \exp\!\left(-\frac{d^2}{2\sigma^2}\right),\quad d\ge0.
\tag{4}
\]
<p>The two scales are \(\sigma_s=__SIGMA_S__\,\mathrm{m}\) and \(\sigma_w=__SIGMA_W__\,\mathrm{m}\). For a particular foot and interval, the symbols used in the factor table mean precisely</p>
\[
 \boxed{S_{f,k}:=h(d_{f,k};\sigma_s),\qquad W_{f,k}:=h(d_{f,k};\sigma_w).}
\tag{5}
\]
<p><b>These are two density values evaluated at the same observed displacement.</b> \(S_{f,k}\) is the narrow, near-stationary likelihood; \(W_{f,k}\) is the broad motion likelihood. Neither is a binary contact indicator nor a probability of stance or swing. Both have units \(\mathrm{m}^{-1}\), so values greater than one are valid. The \(1/\sigma\) term is essential: near zero displacement it favors the narrow density. The broad density also permits small displacement; it does not require a nonzero swing speed.</p>
<table><caption>Table 1. Likelihood values for illustrative displacements. These are evaluations of Eq. (4), not recorded samples.</caption><thead><tr><th>\(d\) (mm)</th><th>\(S(d)\) (\(\mathrm{m}^{-1}\))</th><th>\(W(d)\) (\(\mathrm{m}^{-1}\))</th><th>\(S(d)/W(d)\)</th></tr></thead><tbody>__DISTANCE_ROWS__</tbody></table>
<p>For each foot separately, the binary displacement factor on \((c_{f,k},c_{f,k+1})\) uses the two observations \((d_{f,k},d_{f,k+1})\). Its four entries are</p>
\[
 D_{f,k}=\begin{array}{c|cc}
 & W_{k+1}&S_{k+1}\\\hline
 W_k&W_{f,k}^{w_k}W_{f,k+1}^{w_{k+1}}&W_{f,k}^{w_k}S_{f,k+1}^{w_{k+1}}\\
 S_k&S_{f,k}^{w_k}W_{f,k+1}^{w_{k+1}}&S_{f,k}^{w_k}S_{f,k+1}^{w_{k+1}}
 \end{array}.
\tag{6}
\]
<p>Large/large displacements favor swing/swing; large/small favors touchdown; small/large favors lift-off; small/small favors continued stance. These are soft density comparisons. Changing state no longer grants both measurements the broad likelihood automatically.</p>
<p>Interior measurements appear in two adjacent factors. To count each once, set</p>
\[
 w_k=\begin{cases}1,&k=0\text{ or }k=N-1,\\1/2,&\text{otherwise}.\end{cases}
\tag{7}
\]
<p>This initial binary potential is deliberately <b>separable</b>: it is a weighted product of two single-displacement densities. The product over all displacement factors is exactly one density per foot per slice, up to state-independent scaling and numerical floors. No additional correlation between consecutive displacements is assumed. The binary factor scopes permit a future joint CRF potential; temporal dependence currently comes from Eq. (3). Shared observations are not independent new evidence.</p>
<p>For example, at an interior edge with 20 mm followed by 0 mm, the rescaled table in W/S order is approximately \(\left[\begin{smallmatrix}__PAIR_EXAMPLE__\end{smallmatrix}\right]\). The favored entry is W→S. These values use the scales above and square-root overlap weights.</p>
<p>Each table is divided by its largest entry, preserving MAP. The shared half-normal prefactor cancels. Log ratios are floored at −700 and extreme \(d/\sigma\) ratios capped at \(10^6\) before squaring. The normalization \(1/\sigma\) is retained. The half-normal is an assumed scalar magnitude density, not the radial distribution of a three-dimensional Gaussian.</p>

<h2>4. Pelvis-relative foot velocity</h2>
<p>Let \(B\) denote the pelvis frame, \(R_{WB}\) its orientation, and \(p^W_B\) its origin. The sole origin expressed in pelvis coordinates is \(r^B_f=R_{WB}^{\mathsf T}(p^W_f-p^W_B)\). Its time derivative is</p>
\[
 \dot r^B_f=R_{WB}^{\mathsf T}\!\left[
 (J^W_f-J^W_B)\dot q
 -(J^W_{\omega B}\dot q)\times(p^W_f-p^W_B)
 \right],\qquad v_f=e_x^{\mathsf T}\dot r^B_f.
\tag{8}
\]
<p>Here \(J^W_f\) is the translational Jacobian of the sole origin, \(J^W_B\) is the pelvis-origin translational Jacobian, and \(J^W_{\omega B}\) is the pelvis angular-velocity Jacobian. MuJoCo evaluates these Jacobians using the canonical model and the fixed body-to-sole offset. The rotational transport term is required when expressing relative motion in rotating pelvis axes.</p>
<p>Measured joint angles and rates are independently interpolated to the sample timestamps and mapped by joint name. Free-base generalized velocities are set to zero: common base motion cancels in Eq. (8), leaving joint-induced motion. Consequently this cue is invariant to the world pose and does not use LiDAR velocity.</p>

<h2>5. Two unary velocity measurement factors</h2>
<p>Define a bounded stance compatibility from signed forward velocity:</p>
\[
 c(v)=\operatorname{clip}\!\left(
 \frac{1}{1+\exp(v/s_v)},\epsilon,1-\epsilon\right),
 \qquad s_v=__VELOCITY_SCALE__\,\mathrm{m\,s^{-1}},\quad\epsilon=__FLOOR__.
\tag{9}
\]
<p>Each foot has its own unary factor on its own binary variable:</p>
\[
 V_{f,k}(c_{f,k})=\begin{cases}1-c(v_{f,k}),&c_{f,k}=0,\\c(v_{f,k}),&c_{f,k}=1.\end{cases}
\tag{10}
\]
<p>It is independent of the other foot's state. Forward motion favors swing, backward motion favors stance, and zero velocity gives neutral weights [0.5, 0.5]. These compatibility weights are not calibrated contact posteriors.</p>
<table><caption>Table 2. Velocity-factor weights. The probability floor limits the stance-to-swing odds to between 1:49 and 49:1.</caption><thead><tr><th>\(v\) (m/s)</th><th>Stance weight \(c(v)\)</th><th>Swing weight \(1-c(v)\)</th></tr></thead><tbody>__VELOCITY_ROWS__</tbody></table>

<h2>6. Inference and scope</h2>
<p>GTSAM's <code>DiscreteFactorGraph.optimize</code> with chronological left/right elimination performs exact max-product and traceback, yielding the joint MAP assignment rather than independent marginal modes. There are \(2N\) binary variables. The graph has one initial pair prior, \(N\) support couplings, \(N-1\) four-variable transitions, \(2(N-1)\) binary displacement factors, \(2(N-2)\) three-slice duration factors, and optionally \(2N\) velocity unaries: \(8N-6\) factors with velocities or \(6N-6\) without.</p>
<p>The three SSIO snippets each display 1,000 slices, with ten context slices at each end during inference and one additional pose for the first incoming displacement. Thus \(N=1020\), giving 2,040 binary variables and 8,154 factors (6,114 without velocities). The default-on hard constraints add \(2N-1=2039\) factors, giving 10,193 with velocities or 8,153 without. Unchecking them restores the counts above. Export is offline. The app maps configurations to L, R, double stance, or neither; neither gives two white swing markers.</p>
<p>The direction cue assumes ordinary forward walking and may conflict with backward walking, moving support surfaces, or external leg motion. Translation-only measurements do not identify pure pivots. Displacement and joint-velocity cues share kinematic information and are correlated; their product is a heuristic model rather than calibrated confidence. The parameters above are fixed assumptions, not estimates from force or contact labels.</p>

<h2>7. One sole frame per foot and contact-selected estimation slices</h2>
<p>The G1 pipeline uses the canonical Menagerie <code>g1_mjlab</code> model, compiled through the MuJoCo and MJLab Python APIs. Each foot has one fixed sole frame on its ankle-roll link. Privileged displacement observations above use the origin of that frame; initialization and smoothing use its full pose. Individual sole-corner positions used by the viewer are slip diagnostics and are not additional optimization contact variables.</p>
<p>The full SSIO clip contains __SOURCE_SLICES__ source slices at 50 Hz. Max-product with joint-velocity factors and the hard walking constraints gives __PHASES__ phases: left stance (red), right stance (green), or double stance (yellow). There are __DOUBLE_PHASES__ double-stance phases. A short yellow phase, with sample count times 20 ms at most 200 ms, contributes one central slice. A longer one contributes endpoints buffered inward by one source slice and the fewest evenly spaced interior slices needed to keep each within-phase interval at most 200 ms. This shared policy is the default in all G1 estimation apps; both initializers and smoothing use the same actual timestamps. Duplicate indices are removed. The current result is __SLICES__ estimation slices.</p>
<p>The original phase sequence determines stance identity even when single-stance intervals have no retained solve slice. A foot's landmark persists while it remains in stance; lift-off followed by recontact creates a new landmark. Thus sparse neighboring solver indices do not accidentally join two different footholds. Initialization and full-state smoothing use exactly the same retained source indices and timestamps. Interpolation is used only for visualization.</p>

<h2>8. Shared pose landmarks in initialization and CM-Opt</h2>
<figure id="stance-factor-graph"><img src="__LANDMARK_GRAPH__" alt="fgz kinematic smoothing graph with robot states, shared sole-pose landmarks, velocity variables, Gal3 IMU factors, and one shared bias"><figcaption>Figure 2. A schematic segment of the continuous smoothing graph. Z<sub>k</sub> groups all link COM poses and joint angles at selected slice k; v<sub>k</sub> is pelvis COM world velocity. F is a <code>ContactPoseFactor</code>. The left landmark persists through all three slices; the right foot lifts between slices 1 and 2, so recontact uses a new landmark. I is a five-variable Gal3 IMU factor touching the root-pose component of Z, both velocities, and shared bias b. KJU folds the local kinematic equalities, joint measurements, and optional root-upright factors into one drawing symbol; these remain separate factors in the implementation. H is an optional landmark-height prior. P, P<sub>v</sub>, P<sub>b</sub>, and L are the initial root, initial velocity, shared bias, and optional final-position priors. No adjacent no-slip factors are present.</figcaption></figure>
<p>Let X<sub>k</sub> be the world pose of the pelvis COM, C<sub>s</sub> one world sole-pose landmark for uninterrupted stance s, and L<sub>f,k</sub> the world COM pose of the foot link. All are SE(3) poses; v<sub>k</sub> is a world-frame linear velocity in m/s. The fixed COM-to-sole transform is A<sub>f</sub>. Initialization freezes encoder FK as a root-relative observation B<sub>f,k</sub>. The predicted sole pose is</p>
\[
 \widehat C_{f,k}=\begin{cases}X_k B_{f,k},&\text{contact initialization},\
 L_{f,k}A_f,&\text{full-state smoothing}.
 \end{cases}
 \tag{11}
\]
<p>Both graphs use the same native <code>ContactPoseFactor</code>, with residual and cost</p>
\[
 r_{f,k}=\operatorname{Local}_{\widehat C_{f,k}}(C_{s(f,k)}),\qquad
 E_{f,k}=\tfrac12 r_{f,k}^{\mathsf T}\Sigma_C^{-1}r_{f,k}.
 \tag{12}
\]
<p>Local denotes GTSAM's Pose3 local-coordinate chart, ordered rotation then translation. The per-axis standard deviations are <b>90° rotation and 10 mm translation</b>; degree-valued configuration is converted to radians at the API boundary. These are observation-to-landmark noise scales, not displacement noise per adjacent time interval. Each landmark is a free six-dimensional variable. There are __LANDMARKS__ landmarks and __CONTACT_FACTORS__ observations in the full-trajectory example. Smoothing seeds each landmark from its first predicted sole pose, without using later reference poses.</p>
<p>CM-Opt enforces the articulated link/joint kinematic equalities at every retained slice. Encoder angles have independent measurement priors (standard deviation approximately 1.719°). Contact factors touch the estimated foot-link poses, so joint-angle corrections affect their residuals through those equalities. The fixed first root pose in initialization and the tight first-root prior in smoothing set the world gauge. A common vertical shift, computed from measured FK at the first initialized slice, sets its mean sole height to zero.</p>
<table><caption>Table 3. Priors in the current initializer and smoother.</caption><thead><tr><th>Prior</th><th>Standard deviation</th><th>Initializer</th><th>Smoother default</th></tr></thead><tbody>
<tr><td>Root upright</td><td>15°</td><td>On</td><td>Off; selectable</td></tr>
<tr><td>Stance landmark height at world Z=0</td><td>0.10 m</td><td>On</td><td>On; selectable</td></tr>
<tr><td>Weak identity root motion</td><td>180° / 10 m</td><td>On</td><td>Absent</td></tr>
<tr><td>Sole-normal alignment</td><td>—</td><td>Off</td><td>Absent</td></tr>
</tbody></table>
<p>Root upright uses the residual R<sub>k</sub>e<sub>z</sub>−e<sub>z</sub> and leaves heading and translation free; its angular standard deviation is a small-angle scale. Height is one scalar residual C<sub>s,z</sub> per landmark and leaves XY and orientation free. The upright starting-value policy is separate from these priors. Contact vs IK uses this same landmark smoothing graph for both initializations and retains ten smoothing iterations. Kinematics offers all four upright/height combinations, with IMU and loop closure independently selectable, and allows __MAX_ITERATIONS__ iterations.</p>

<h2>9. IMU aiding, error units, and reference alignment</h2>
<p>The Gal3 factor on each retained interval has keys (X<sub>k</sub>,v<sub>k</sub>,X<sub>k+1</sub>,v<sub>k+1</sub>,b). One constant sensor-axis bias b is shared throughout the clip, without bias random walk. Each pim integrates native IMU timestamps between the exact selected boundaries, including fractional endpoint intervals. The 420 intervals range from 0.06 to 25.58 s. The root COM pose is world_T_COM, mapping COM coordinates into world coordinates, and the velocity is world linear velocity of that COM. The COM-to-IMU transform rotates measurements and applies GTSAM's centripetal lever-arm correction; the angular-acceleration lever-arm term is omitted. Gravity is [0,0,−9.81] m/s².</p>
<table><caption>Table 4. IMU assumptions. Strengths are provisional, manually chosen values.</caption><thead><tr><th>Quantity</th><th>Setting per axis</th><th>Origin</th></tr></thead><tbody>
<tr><td>Accelerometer white-noise density</td><td>0.1 m/s²/√Hz</td><td>Provisional</td></tr>
<tr><td>Gyro white-noise density</td><td>__GYRO_DENSITY__ °/s/√Hz</td><td>Provisional</td></tr>
<tr><td>Integration covariance</td><td>10⁻⁸ I</td><td>Provisional</td></tr>
<tr><td>Accelerometer bias prior</td><td>Zero mean; σ=0.5 m/s²</td><td>Manual prior</td></tr>
<tr><td>Gyro bias prior</td><td>Stationary mean; σ=0.00573 °/s</td><td>Mean of 5,000 samples in 0–25 s; manual strength</td></tr>
<tr><td>Initial COM velocity</td><td>Zero mean; σ=3 m/s</td><td>Broad manual prior</td></tr>
<tr><td>Final-position return prior</td><td>XY: 0.30 m; Z: 0.05 m</td><td>Manual return-to-start assumption; orientation free</td></tr>
</tbody></table>
<p>Measurement densities are continuous-time quantities; each pim propagates them into its correlated nine-dimensional covariance using the actual sample intervals. The cost displayed by the apps is half the squared whitened residual, summed by factor group. Hard kinematic equalities are reported separately and excluded from that sum. In the current default 50-iteration IMU-plus-loop-closure run, total cost is __TOTAL_COST__, of which IMU cost is __IMU_COST__ across 420 nine-dimensional factors. Its RMS whitened IMU residual is __IMU_RMS__. These strengths have not been calibrated from sensor specifications or an Allan-variance analysis.</p>
<p>The default still reaches the iteration cap, with __REFERENCE_RMS__ m root-COM reference-position RMS. The reference is SuperOdom LiDAR-inertial odometry. The optional <em>Aligned reference (SE(3))</em> display fits one proper rigid transform to paired pelvis body positions with equal weight per retained slice and fixed unit scale. It changes the reference overlay, world-coordinate curves, and displayed pelvis-reference RMS; estimated states, factor errors, and foot-in-pelvis coordinates are unaffected. Per-iteration scalar error histories are retained without retaining intermediate full-trajectory robot poses.</p>

</article><footer><p>Reference application: SSIO / Open-IO G1 clip 10871_humanoid_20260520_223451, by Yuheng Qiu and the SSIO authors / Open-IO, source revision db621c57833e08a812b2ec8491aa0ca81f0ecb33. Source motion: <a href="https://creativecommons.org/licenses/by/4.0/">CC BY 4.0</a>. Derived quantities use joint FK, recorded joint rates, and the SuperOdom LiDAR-inertial reference; the reference is not independent contact ground truth.</p><p>Typeset with MathJax 3.2.2 during export. Equations and fgz diagrams are embedded as SVG for offline viewing and printing. Diagrams use fgz revision 51fdc784d8217542ae2b34179ce698e397972441; editable .fgz sources accompany the report generator.</p></footer>
<script id="kinematic-model" type="application/json">__KINEMATIC_MODEL__</script>
<script id="factor-model" type="application/json">__MODEL__</script>
</body></html>"""
