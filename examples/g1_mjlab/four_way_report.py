# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file four_way_report.py
# @brief Render a self-contained four-way CMC friction comparison report.
# @author Frank Dellaert

"""Render complete repeated measurements, physical validation, and CPU/GPU timing."""

import argparse
import base64
import html
import gzip
import io
import json
from pathlib import Path
from statistics import median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LABELS = {
    "pyramidal": "Pyramidal inequalities",
    "elliptic_norm": "Elliptic norm inequality",
    "signed_square": "Signed-square inequality",
    "friction_factor": "ContactDynamicsFrictionConeFactor",
}
SHORT = ["Pyramidal", "Elliptic norm", "Signed-square", "Penalty factor"]


def number(value):
    return "—" if value is None else f"{value:.4g}"


def embedded(fig, alt):
    stream = io.BytesIO()
    fig.savefig(stream, format="png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    return f'<img alt="{html.escape(alt)}" src="data:image/png;base64,{base64.b64encode(stream.getvalue()).decode()}">'


def state_status(runs):
    if any("error" in r for r in runs):
        return "error"
    return "valid" if all(r["validated"] for r in runs) else "invalid"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path)
    args = parser.parse_args()
    data = args.data
    def read(name):
        path = data / name
        if path.exists():
            return json.loads(path.read_text())
        with gzip.open(data / (name + ".gz"), "rt") as stream:
            return json.load(stream)
    config, rows = read("four-way-config.json"), read("four-way-results.json")
    references, validation = read("four-way-references.json"), read("four-way-validation.json")
    indices, repeats = config["indices"], config["repeats"]
    if config["schema"] != "gtdynamics-four-way-v1" or not config.get("completed_utc"):
        raise ValueError("The four-way benchmark has not completed")
    if [r["index"] for r in rows] != indices or len(set(indices)) != len(indices):
        raise ValueError("Incomplete or duplicated sample set")
    if set(config["variants"]) != set(LABELS):
        raise ValueError("Expected exactly the four documented formulations")
    for row in rows:
        if set(row["runs"]) != set(LABELS) or any(len(rr) != repeats for rr in row["runs"].values()):
            raise ValueError("Incomplete formulation or repetition set")
    if not validation["passed"] or {r["formulation"] for r in validation["cases"]} != set(LABELS):
        raise ValueError("Independent physics checks must cover all four formulations")
    for cone in ("pyramidal", "elliptic"):
        if [r["index"] for r in references[cone]] != indices:
            raise ValueError("Reference states do not match comparison states")

    summary = {"configuration": config, "physics_validation": validation, "groups": {}, "references": {}}
    table = []
    for variant, label in LABELS.items():
        valid = [r for r in rows if state_status(r["runs"][variant]) == "valid"]
        complete = [r for r in rows if state_status(r["runs"][variant]) != "error"]
        returned = [x for r in rows for x in r["runs"][variant] if "error" not in x]
        def milliseconds(selected):
            return 1000 * median(median(x["dynamics_seconds"] for x in r["runs"][variant]) for r in selected) if selected else None
        group = {
            "validated": len(valid), "returned_but_invalid": len(complete) - len(valid),
            "errors_or_timeouts": len(rows) - len(complete),
            "median_returned_ms": milliseconds(complete), "median_validated_ms": milliseconds(valid),
            "validated_indices": [r["index"] for r in valid],
            "all_repetitions_same_status": all(len({("error" if "error" in x else "valid" if x["validated"] else "invalid") for x in r["runs"][variant]}) == 1 for r in rows),
            "maximum_returned": {k: max((x[k] for x in returned), default=None) for k in config["thresholds"]},
        }
        summary["groups"][variant] = group
        table.append(f'<tr><th>{label}</th><td>{len(valid)}/{len(rows)}</td><td>{group["returned_but_invalid"]}</td><td>{group["errors_or_timeouts"]}</td><td>{number(group["median_returned_ms"])}</td><td>{number(group["median_validated_ms"])}</td></tr>')
    fig, ax = plt.subplots(figsize=(10, 4.2), layout="constrained")
    passed = [summary["groups"][k]["validated"] for k in LABELS]
    invalid = [summary["groups"][k]["returned_but_invalid"] for k in LABELS]
    errors = [summary["groups"][k]["errors_or_timeouts"] for k in LABELS]
    ax.bar(SHORT, passed, color="#338366", label="Validated")
    ax.bar(SHORT, invalid, bottom=passed, color="#d8a13c", label="Returned but invalid")
    ax.bar(SHORT, errors, bottom=[a+b for a,b in zip(passed,invalid)], color="#b65956", label="Error / timeout")
    ax.set(ylabel="Recorded states", ylim=(0, len(rows) * 1.2))
    ax.legend(ncol=3, loc="upper center")
    reliability_plot = embedded(fig, "Validated, invalid, and rejected states for each formulation")
    fig, ax = plt.subplots(figsize=(10, 4.6), layout="constrained")
    for color, (variant, label) in zip(("#207b6b", "#336bb3", "#9159a4", "#bb772c"), LABELS.items()):
        for status, marker in (("valid", "o"), ("invalid", "x")):
            selected = [r for r in rows if state_status(r["runs"][variant]) == status]
            ax.scatter([r["index"] for r in selected],
                       [1000 * median(x["dynamics_seconds"] for x in r["runs"][variant]) for r in selected],
                       color=color, marker=marker, label=label if status == "valid" else None, s=26)
    ax.set(xlabel="Recorded state index", ylabel="Full GTDynamics solve (ms, logarithmic)", yscale="log")
    ax.grid(alpha=.2)
    ax.legend(fontsize=8)
    timing_plot = embedded(fig, "Per-state median dynamics latency; crosses indicate invalid returns")

    reference_table = []
    for cone, rr in references.items():
        native = 1000 * median(r["native"]["seconds"] for r in rr)
        reference_table.append(f'<tr><th>{cone}</th><td>MuJoCo CPU</td><td>{number(native)}</td><td>—</td><td>—</td><td>Reference</td></tr>')
        group = {"native_median_ms": native}
        if config["warp_evaluated"]:
            warp = read(f"four-way-warp-{cone}.json")
            group.update({"warp_single_median_ms": 1000 * median(r["warp"]["seconds"] for r in rr),
                          "warp_batch_ms": 1000 * warp["batch"]["seconds"],
                          "warp_batch_us_per_world": 1e6 * warp["batch"]["seconds_per_world"],
                          "warp_validated_accelerations": sum(r["warp_acceleration_max_abs"] < 1e-3 for r in rr),
                          "warp_max_acceleration_error": max(r["warp_acceleration_max_abs"] for r in rr),
                          "device": warp["device"]})
            reference_table.append(f'<tr><th>{cone}</th><td>MJLab/Warp ({html.escape(warp["device"])})</td><td>{number(group["warp_single_median_ms"])}</td><td>{number(group["warp_batch_ms"])}</td><td>{number(group["warp_batch_us_per_world"])}</td><td>{group["warp_validated_accelerations"]}/{len(rr)}; max error {number(group["warp_max_acceleration_error"])}</td></tr>')
        summary["references"][cone] = group

    cases, failures = [], []
    for row in rows:
        cells = []
        for variant in LABELS:
            rr = row["runs"][variant]
            status = state_status(rr)
            completed = [r for r in rr if "error" not in r]
            ms = 1000 * median(r["dynamics_seconds"] for r in completed) if completed else None
            detail = "; ".join(f'{k}: {number(max(r[k] for r in completed))}' for k in config["thresholds"]) if completed else "No returned solution"
            cells.append(f'<td class="{status}" title="{html.escape(detail)}">{status}<br>{number(ms)} ms</td>')
            for repeat, r in enumerate(rr):
                if "error" in r:
                    failures.append(f'<li>State {row["index"]}, {LABELS[variant]}, repeat {repeat+1}: <code>{html.escape(r["error"])}</code></li>')
        cases.append(f'<tr><th>{row["index"]}</th>'+"".join(cells)+"</tr>")
    errors_table = []
    for variant, label in LABELS.items():
        group = summary["groups"][variant]
        errors_table.append(f'<tr><th>{label}</th>'+"".join(f'<td>{number(group["maximum_returned"][k])}</td>' for k in config["thresholds"])+"</tr>")
    (data / "four-way-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    sigma = config["friction_factor_sigma"]
    physics = validation["maximum_errors"]
    doc = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 CMC dynamics: four friction formulations</title>
<style>body{{font:16px/1.6 system-ui,sans-serif;max-width:1200px;margin:38px auto;padding:0 22px;color:#203044;background:#fafcfe}}h1,h2{{line-height:1.2}}h1{{font-size:2.1rem}}h2{{margin-top:2.4rem}}table{{border-collapse:collapse;width:100%;font-size:14px;background:white}}th,td{{border-bottom:1px solid #dce4eb;padding:9px;text-align:right}}th:first-child{{text-align:left}}thead{{background:#edf2f7}}img{{max-width:100%;height:auto}}.scroll{{overflow-x:auto}}.valid{{color:#237357}}.invalid{{color:#916311}}.error{{color:#a13d3d}}.note{{padding:14px 18px;background:#edf3f8;border-left:4px solid #527ca0}}code,pre{{overflow-wrap:anywhere;white-space:pre-wrap}}small{{color:#526579}}</style></head><body>
<h1>G1 CMC dynamics: four friction formulations</h1>
<p>{len(rows)} recorded states × {repeats} repetitions × four formulations. All runs use the full GTDynamics link/joint constraint graph and the updated <code>ReducedALRetractor</code>. Run completed {html.escape(config['completed_utc'])}.</p>
<p>QP backend: <strong>{html.escape(config['solver_build']['qp_backend'])}</strong>, operating in equality-reduced coordinates for both tangent-cone projection and IQP step refinement.</p>
<p class="note">CMC discovers the connected equality component, retains its constant affine basis, and caches its projection metric. Retraction skips AL when the trial or equality projection already satisfies every inequality and required blocking face. Other trials use reduced AL. MuJoCo inertia matrices and contact Jacobians are used only to validate outputs; they are never supplied to the GTDynamics solver.</p>
<h2>Reliability and full dynamics timing</h2>
<div class="scroll"><table><thead><tr><th>Formulation</th><th>Validated states</th><th>Returned but invalid</th><th>Errors / timeouts</th><th>Returned median ms</th><th>Validated median ms</th></tr></thead><tbody>{''.join(table)}</tbody></table></div>
<p>A state is validated only if all {repeats} repetitions pass. Each state contributes its median latency. Returned medians include incorrect solutions; validated medians use different successful-state subsets and are not direct speedup ratios. Errors and {config['timeout_seconds']:g}-second timeouts are excluded from latency medians.</p>
{reliability_plot}
{timing_plot}
<p>Points are validated states; crosses are returned but invalid states. Rejected states are omitted from the latency plot. Timings include GTDynamics model construction/FK, factor assembly, feasible initialization, manifold construction, and CMC optimization; JSON, process startup, and shared collision generation are excluded.</p>
<h2>What differs between the four cases?</h2>
<ul><li><strong>Pyramidal:</strong> four nonnegative edge coefficients, using the shared direct pyramidal inequalities and MuJoCo's matching pyramidal contact law.</li>
<li><strong>Elliptic norm:</strong> <code>hypot(t₁/μ₁,t₂/μ₂) − n ≤ 0</code>, plus a normal nonnegativity row. The norm is nonsmooth at zero tangential force.</li>
<li><strong>Signed-square:</strong> <code>t₁² + (μ₁/μ₂)² t₂² − μ₁² n|n| ≤ 0</code>. It has the same elliptic feasible set and excludes negative normals; its gradient vanishes at the zero-force apex.</li>
<li><strong>Penalty factor:</strong> the actual <code>ContactDynamicsFrictionConeFactor</code> on the body pose and contact wrench, with the known pose fixed through <code>ConstVarGraph</code>. It adds <code>0.5 [max(0, |t|² − μ²n²) / {sigma:g}]²</code> to the physical objective. A separate normal inequality excludes pulling. No hard tangential-cone inequality is added.</li></ul>
<p>The penalty case changes the optimization problem. Finite penalty weight can admit friction violations even when its own optimizer converges. Sigma {sigma:g} matches the standard dynamics setting; these measurements do not select an optimal penalty weight. The runner checks isotropic friction and vertical contact normals for this factor. Elliptic variants share identical physical inputs, objective before the penalty, and initial forces. Pyramidal forces start at zero; elliptic forces start at 1 N normal force.</p>
<h2>Fresh MuJoCo CPU and MJLab/Warp timings</h2>
<div class="scroll"><table><thead><tr><th>Cone</th><th>Engine</th><th>Single state ms</th><th>{len(rows)}-world batch ms</th><th>Batch μs/world</th><th>Acceleration agreement with CPU</th></tr></thead><tbody>{''.join(reference_table)}</tbody></table></div>
<p>All reference timings were measured in this run: {config['timing_warmup']} warmups and {config['timing_repeats']} repetitions, resetting <code>qacc_warmstart</code> each time. CPU measurements time complete <code>mj_forward</code>. GPU measurements time synchronized CUDA-graph <code>mujoco_warp.forward</code>, excluding compilation, allocation, and state transfers. Batch throughput is not single-state latency. GPU validation here checks acceleration only; GTDynamics acceptance additionally checks constraints and stationarity.</p>
<h2>Independent physical acceptance</h2>
<p>Every returned result is checked against its matching MuJoCo cone. Required: acceleration max error &lt; 1e−3, equality L2 residual &lt; 1e−5, canonical physical cone violation L2 &lt; 1e−4, and projected-gradient infinity norm &lt; 1e−3. The cone check uses the force itself, so the differently scaled signed-square residual and the penalty's missing hard cone cannot hide a violation. Stationarity is checked against the original physical constrained objective.</p>
<table><thead><tr><th>Formulation</th><th>Max acceleration error</th><th>Max equality L2</th><th>Max physical cone L2</th><th>Max projected gradient</th></tr></thead><tbody>{''.join(errors_table)}</tbody></table>
<p>These maxima include invalid returned solutions. Separately, all {len(validation['cases'])} prescribed-force checks passed across the four formulations, three states, and zero/random/reference forces. They verify poses, twists, acceleration maps, bias acceleration, armature/contact dynamics, force ordering, the physical objective, and the actual penalty value. Maximum dynamics discrepancy: {physics['dynamics']:.3g}; maximum physical-objective discrepancy: {physics['objective']:.3g}. Prescribed reference forces never initialize comparison solves.</p>
<h2>Every recorded state</h2>
<p>Hover over a cell to see its worst physical acceptance metrics. A cell containing any failed repetition is marked error, even if another repetition returned.</p>
<div class="scroll"><table><thead><tr><th>State</th>{''.join(f'<th>{x}</th>' for x in SHORT)}</tr></thead><tbody>{''.join(cases)}</tbody></table></div>
<details><summary>Solver failures ({len(failures)} repeated runs)</summary><ul>{''.join(failures) or '<li>None</li>'}</ul></details>
<h2>Build and reproducibility</h2>
<p>GTSAM develop commit <code>{html.escape(config['gtsam_head'])}</code>; source dirty: {config['gtsam_dirty']}. GTDynamics base <code>{html.escape(config['gtdynamics_head'])}</code>; uncommitted comparison edits at measurement: {config['gtdynamics_dirty']}. Exact source, binary, library, and input hashes are recorded locally with the measurements.</p>
<pre>{html.escape(json.dumps(config['solver_build'], indent=2))}</pre>
<p><small>Solver SHA-256: {config['solver_sha256']}<br>GTDynamics library SHA-256: {config['libraries']['libgtdynamics.so']['sha256']}<br>GTSAM library SHA-256: {config['libraries']['libgtsam.so.4']['sha256']}<br>{html.escape(config['platform'])}</small></p>
<p><a href="../README.md">Reproduction instructions</a> · <a href="../four_way_compare.py">Comparison runner</a> · <a href="../full_dynamics.cpp">Full constraint graph</a> · <a href="https://github.com/borglab/GTDynamics/pull/449">Generic retractor PR</a></p>
</body></html>'''
    (data / "report.html").write_text(doc)
    print(json.dumps(summary["groups"], indent=2))
    print(data / "report.html")


if __name__ == "__main__":
    main()
