#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file report.py
# @brief Compare CMC benchmark outputs and generate a self-contained HTML report.
# @author GTDynamics contributors
"""Generate an offline report with per-case quality, timing, and downloadable data."""
from __future__ import annotations

import argparse
from collections import defaultdict
import html
import json
import math
from pathlib import Path
import statistics
from typing import Any

import numpy as np


def read_runs(folder: Path) -> dict[str, list[dict[str, Any]]]:
    """Read measured trials, retaining failures and incomplete result records."""
    cases = defaultdict(list)
    ledger = folder / "runs.jsonl"
    if not ledger.exists():
        return cases
    runs = [json.loads(line) for line in ledger.read_text().splitlines()]
    measured_cases = {run["case"] for run in runs if not run["warmup"]}
    for run in runs:
        if run["warmup"] and (run["case"] in measured_cases or run["status"] == 0):
            continue
        path = folder / run["case"] / str(run["repeat"])
        record = dict(run, directory=str(path))
        try:
            if (path / "result.jsonl").exists():
                stages = [
                    json.loads(line)
                    for line in (path / "result.jsonl").read_text().splitlines()
                ]
                if not stages:
                    raise ValueError("no completed solve")
                record["result"] = stages[-1]
                record["stages"] = stages
                record["time"] = sum(stage["optimization_seconds"] for stage in stages)
            elif (path / "summary.json").exists():
                result = json.loads((path / "summary.json").read_text())
                record["result"] = result
                record["time"] = result.get(
                    "optimizationSeconds", result.get("dynamics_seconds")
                )
            else:
                result = json.loads((path / "stdout.log").read_text())
                if "error" in result:
                    raise ValueError(result["error"])
                record["result"] = result
                record["time"] = result["seconds"]
        except (ValueError, KeyError, OSError) as error:
            record["failure"] = (
                f"no completed solve within {run['seconds']:.1f} seconds"
                if run["status"] == "timeout"
                else str(error)
            )
            # The final exception line carries more information than invalid JSON.
            for name in ("stderr.log", "stdout.log"):
                log = path / name
                if log.exists():
                    lines = log.read_text(errors="replace").splitlines()
                    exceptions = [
                        line
                        for line in lines
                        if any(
                            word in line
                            for word in ("Error:", "what():", "terminate called")
                        )
                    ]
                    if exceptions:
                        record["failure"] = exceptions[-1][-1000:]
        cases[run["case"]].append(record)
    return cases


def fields(result: dict[str, Any]) -> dict[str, Any]:
    """Normalize existing native summaries without changing timing scopes."""

    def get(*names: str) -> Any:
        return next((result[name] for name in names if name in result), None)

    return dict(
        cost=get("cost", "objective", "objectiveFinal"),
        equality=get("equality_l2", "equalityL2"),
        inequality=get("inequality_l2", "inequalityL2"),
        iterations=get("iterations"),
        inner_iterations=get("inner_iterations"),
        equality_max=get("equality_max", "max_force_residual_n"),
        inequality_max=get("inequality_max"),
        feasible=get("feasible"),
        maximum_moment_residual=get("max_moment_residual_nm"),
        penalty_cost=get("penalty_cost"),
        initial_cost=get("objectiveInitial"),
        initial_equality=get("initialEqualityL2"),
        variable_dimension=get("variableDimension"),
        reduced_dimension=get("reducedDimension"),
        equality_dimension=get("equalityDimension"),
        inequality_dimension=get("inequalityDimension"),
    )


def compare_arrays(before: dict, after: dict) -> dict:
    """Compare saved solutions and every factor cost, including intermediate stages."""
    pairs = []
    before_stages = before.get("stages", [before["result"]])
    after_stages = after.get("stages", [after["result"]])
    if len(before_stages) != len(after_stages):
        return dict(equal=False, close=False, reason="stage count changed")
    for stage_before, stage_after in zip(before_stages, after_stages):
        quality_before, quality_after = fields(stage_before), fields(stage_after)
        for key, value in quality_before.items():
            if value is not None:
                pairs.append((key, np.asarray(value), np.asarray(quality_after[key])))
        for key in (
            "coordinates_from_initial",
            "factor_costs",
            "force",
            "acceleration",
            "errorHistory",
            "errorNames",
            "estimatedBias",
            "groundTranslation",
            "initializationSlices",
        ):
            if key in stage_before:
                pairs.append(
                    (key, np.asarray(stage_before[key]), np.asarray(stage_after[key]))
                )
    for filename in ("dynamics.npz", "kinodynamics.npz"):
        path_before = Path(before["directory"]) / filename
        path_after = Path(after["directory"]) / filename
        if path_before.exists() and path_after.exists():
            with np.load(path_before) as a, np.load(path_after) as b:
                if set(a.files) != set(b.files):
                    return dict(equal=False, close=False, reason="array names changed")
                pairs.extend((name, a[name], b[name]) for name in a.files)
    exact = True
    close = True
    maximum = 0.0
    worst = None
    for name, a, b in pairs:
        if a.shape != b.shape:
            return dict(equal=False, close=False, reason=f"{name}: shape changed")
        same = np.array_equal(a, b)
        exact &= same
        if a.dtype.kind in "iufcb" and b.dtype.kind in "iufcb":
            if not np.isfinite(a).all() or not np.isfinite(b).all():
                return dict(
                    equal=False, close=False, reason=f"{name}: nonfinite values"
                )
            delta = float(np.max(np.abs(a.astype(float) - b.astype(float)), initial=0))
            close &= bool(np.allclose(a, b, rtol=1e-8, atol=1e-8))
            if delta > maximum:
                maximum, worst = delta, name
        else:
            close &= same
    return dict(
        equal=bool(exact),
        close=bool(close),
        max_absolute=maximum,
        worst_array=worst,
        arrays=len(pairs),
    )


def summarize(name: str, before: list[dict], after: list[dict]) -> dict:
    """Report all outcomes and the spread of successful solver measurements."""
    record: dict[str, Any] = dict(
        case=name,
        before_status=[r["status"] for r in before],
        after_status=[r["status"] for r in after],
        warmup_only=bool(before or after) and all(r["warmup"] for r in before + after),
    )
    for label, runs in (("before", before), ("after", after)):
        good = [run for run in runs if run["status"] == 0 and "failure" not in run]
        record[label + "_successful"] = len(good)
        record[label + "_failures"] = list(
            dict.fromkeys(
                f"{run['status']}: {run.get('failure', '')}".rstrip(": ")
                for run in runs
                if run not in good
            )
        )
        if good:
            timings = [run["time"] for run in good]
            record[label + "_seconds"] = dict(
                median=statistics.median(timings),
                minimum=min(timings),
                maximum=max(timings),
                samples=timings,
            )
            record[label + "_quality"] = fields(good[-1]["result"])
    after_by_repeat = {run["repeat"]: run for run in after}
    paired = [
        (a, after_by_repeat[a["repeat"]])
        for a in before
        if a["repeat"] in after_by_repeat
        for b in [after_by_repeat[a["repeat"]]]
        if a["status"] == b["status"] == 0 and "failure" not in a and "failure" not in b
    ]
    if paired:
        comparisons = [compare_arrays(a, b) for a, b in paired]
        record["solutions"] = dict(
            exact=all(c["equal"] for c in comparisons),
            close=all(c["close"] for c in comparisons),
            max_absolute=max(
                (
                    c["max_absolute"]
                    for c in comparisons
                    if c.get("max_absolute") is not None
                ),
                default=None,
            ),
            details=comparisons,
        )
        record["time_ratio"] = (
            record["after_seconds"]["median"] / record["before_seconds"]["median"]
        )
        record["paired_time_ratios"] = [b["time"] / a["time"] for a, b in paired]
        record["paired_geometric_ratio"] = math.exp(
            statistics.mean(math.log(ratio) for ratio in record["paired_time_ratios"])
        )
    return record


def number(value: Any) -> str:
    """Format a nullable scalar for a compact technical table."""
    return "—" if value is None else f"{value:.6g}"


def main() -> None:
    """Build a standalone HTML artifact and companion machine-readable summary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, nargs="+", required=True)
    parser.add_argument("--after", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--notes", type=Path)
    parser.add_argument("--g1-requests", type=Path)
    parser.add_argument("--recheck", type=Path, action="append", default=[])
    parser.add_argument("--provenance", type=Path)
    args = parser.parse_args()
    before = {
        key: value for folder in args.before for key, value in read_runs(folder).items()
    }
    after = {
        key: value for folder in args.after for key, value in read_runs(folder).items()
    }
    expected = set(before) | set(after)
    if args.g1_requests:
        from run import cases as native_cases
        from g1_cases import walking_cases

        expected.update(case["name"] for case in native_cases())
        expected.update(case["name"] for case in walking_cases())
        expected.update(
            case["name"] for case in json.loads(args.g1_requests.read_text())
        )
    comparisons = [
        summarize(name, before.get(name, []), after.get(name, []))
        for name in sorted(expected)
    ]
    successful = [c for c in comparisons if "solutions" in c]
    exact = sum(c["solutions"]["exact"] for c in successful)
    close = sum(c["solutions"]["close"] for c in successful)
    notes = args.notes.read_text() if args.notes else ""
    data = dict(
        baseline="f71c907d",
        candidate="component-ownership refactor",
        cases=comparisons,
        notes=notes,
        rechecks=[json.loads(path.read_text()) for path in args.recheck],
        provenance=json.loads(args.provenance.read_text()) if args.provenance else {},
    )
    args.output.with_suffix(".json").write_text(
        json.dumps(data, indent=2, allow_nan=False) + "\n"
    )
    rows = []
    for case in comparisons:
        b = case.get("before_quality", {})
        a = case.get("after_quality", {})
        status = (
            (
                "exact"
                if case["solutions"]["exact"]
                else "within tolerance" if case["solutions"]["close"] else "DIFF"
            )
            if "solutions" in case
            else "failed / incomplete"
        )
        rows.append(
            "<tr>"
            + "".join(
                f"<td>{html.escape(str(item))}</td>"
                for item in [
                    case["case"],
                    f"{case['before_successful']}/5 → {case['after_successful']}/5",
                    (
                        (
                            "paired warmup timeout"
                            if case["before_status"]
                            == case["after_status"]
                            == ["timeout"]
                            else "warmup failure"
                        )
                        if case["warmup_only"]
                        else status
                    ),
                    number(case.get("before_seconds", {}).get("median")),
                    number(case.get("after_seconds", {}).get("median")),
                    number(case.get("time_ratio")),
                    number(b.get("cost")),
                    number(a.get("cost")),
                    number(b.get("equality")),
                    number(a.get("equality")),
                    number(b.get("inequality")),
                    number(a.get("inequality")),
                    f"{b.get('iterations','—')} → {a.get('iterations','—')}",
                    number(case.get("solutions", {}).get("max_absolute")),
                ]
            )
            + "</tr>"
        )
    failures = "".join(
        f"<li><b>{html.escape(c['case'])}</b>: before {html.escape('; '.join(c['before_failures']))}; after {html.escape('; '.join(c['after_failures']))}</li>"
        for c in comparisons
        if c["before_failures"] or c["after_failures"]
    )
    encoded = json.dumps(data, allow_nan=False).replace("</", "<\\/")
    page = """<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>CMC-Opt component refactor</title>
<style>body{font:16px/1.6 system-ui,sans-serif;background:#f7f8f4;color:#21342e;margin:0 auto;padding:40px;max-width:1400px}h1,h2{line-height:1.2}p{max-width:1000px}code,pre{font-family:ui-monospace,monospace}pre{background:white;padding:20px;overflow:auto;border:1px solid #cdd8d1}table{border-collapse:collapse;font-size:12px;white-space:nowrap;width:100%}th,td{padding:8px;border-bottom:1px solid #cdd8d1;text-align:right}th:first-child,td:first-child{text-align:left}th{background:#e8eee8;position:sticky;top:0}button,input{font:inherit;padding:8px;margin:8px 0}section.scroll{overflow:auto;max-height:700px}li{margin-bottom:8px}.equation{font:20px Georgia,serif;padding:10px 24px;background:white}a{color:#176451}</style>
<h1>CMC-Opt component refactor</h1>
<p>The refactor makes each component responsible for its constraint geometry and leaves coupled objective optimization in the optimizer. The baseline is <code>f71c907d</code>.</p>
<h2>Architecture</h2>
<p>The checkpoint already used local null-space elimination for the global linear solve. It also kept a global container of component pointers, a second map-owning coordinate class, and mappings between flat and component constraint indices. This change removes that bookkeeping layer. Connected components are still discovered once at problem construction; local linearizations are refreshed when the nonlinear state changes.</p>
<p><code>IEConstraintManifold</code> owns its equality basis and one immutable <code>IEManifoldLinearization</code>. That local object supplies cone projection, directional feasibility, selected-face charts, boundary intersections, and multiplier candidates. <code>IELMWorkingSet</code> contains stateless coordination functions. It owns no constraints.</p>
<p>The refactor removes <code>TangentConeLinearization</code>, the global map-owning <code>ComponentTangentSpace</code>, <code>IndexSetMapTranslator</code>, and compatibility factor exports. <code>WorkingFace</code> represents one component. Gaussian cost transformation and recovery are free functions. Original constraint indices survive from manifold construction through retraction.</p>
<h2>Like an undergrad</h2>
<p>A constrained robot problem contains a soft objective and equations or inequalities that must hold. CMC groups variables connected by hard constraints into components. Costs can still connect different components, so their best motions must be chosen together.</p>
<div class="equation">minimize F(x) = ½ ‖r(x)‖², subject to h<sub>i</sub>(x<sub>i</sub>) = 0 and g<sub>i</sub>(x<sub>i</sub>) ≤ 0.</div>
<p>At the current estimate, the equality basis B<sub>i</sub> describes directions that satisfy the linearized equality equations. Active inequality derivatives restrict these directions to a local tangent cone. The gradient projection suggests which boundaries block downhill motion. LM then solves a damped quadratic approximation of the coupled objective.</p>
<div class="equation">Δx<sub>i</sub> = B<sub>i</sub> δ<sub>i</sub>, &nbsp; A<sub>i</sub>δ<sub>i</sub> ≤ 0, &nbsp; A<sub>i</sub> = Dg<sub>i</sub>(x<sub>i</sub>)B<sub>i</sub>.</div>
<p>A working set temporarily treats selected boundaries as equalities. A second basis N<sub>i</sub> spans their null space. Substitution turns that working-set subproblem into an ordinary least-squares problem in η. The two bases have different jobs: B belongs to the equality manifold; N changes when the selected inequality faces change.</p>
<div class="equation">A<sub>i,W</sub>N<sub>i</sub> = 0, &nbsp; δ<sub>i</sub> = N<sub>i</sub>η<sub>i</sub>, &nbsp; Δx<sub>i</sub> = B<sub>i</sub>N<sub>i</sub>η<sub>i</sub>.</div>
<p>Every feasible working-face direction is representable by Nη. Therefore minimizing the same quadratic after substitution gives the same minimizer in the original coordinates. Damping is substituted too: its term is ‖DNη‖². Cross-component cost blocks remain in the objective, retaining all coupling.</p>
<div class="equation">min<sub>δ : A<sub>W</sub>δ=0</sub> ½‖Jδ − b‖² + λ/2 ‖Dδ‖²<br>= min<sub>η</sub> ½‖JNη − b‖² + λ/2 ‖DNη‖².</div>
<p>The optimizer adds violated boundaries and optionally refines the working set. Each component finds its next boundary and solves its local multiplier equation A<sub>i,W</sub><sup>T</sup>μ<sub>i</sub> = −∇<sub>i</sub>q. That gradient comes from the full coupled objective. The optimizer selects the earliest entering boundary or most negative multiplier across components. Finally each component retracts its step onto the nonlinear manifold, and LM accepts or rejects the resulting cost change.</p>
<p>The Cartesian product of the local cones describes all feasible directions; the code queries the components directly. The mathematics, row order, rank detection, tolerances, objective compaction, damping, and retraction policies are preserved. This refactor changes ownership and indexing; it does not replace the algorithms inside local projection or nonlinear retraction.</p>
<h2>Validation and timing</h2>
<p>Native tests compare selected-face solves against independent constrained QR and compare coupled QP refinement with exhaustive working-set enumeration. They cover dependent and opposing rows, zero-DOF components, scaling, damping, sparse kernels, state sharing, and nonlinear geometry refresh. Existing manifold and specialized-retractor tests remain in the suite.</p>
<p>Each successful case has one warmup and five measured runs. A pair that times out in both warmups is recorded as censored; further identical timeouts are skipped. Earlier timeout batches retain their measured failures. Native script timings cover optimization; G1 full dynamics uses its native solver scope; fixed dynamics includes native construction and solve; kinodynamics reports native optimization. All comparisons use the same scope within a case. Native full-dynamics objectives include their existing linear terms and may be negative. Exact denotes saved numerical arrays matching element-for-element; within tolerance uses absolute and relative tolerances of 10⁻⁸. Completed means the process returned a readable solution; feasibility and convergence must also be judged from the residuals and stopping conditions. Failures remain visible.</p>
"""
    page = page.replace(
        "<h2>Architecture</h2>",
        f"<p><b>{len(comparisons)} benchmark cases:</b> {len(successful)} have paired completed solutions; {exact} match exactly. The remaining {len(comparisons) - len(successful)} cases have no paired completed solution; their failure and timeout outcomes are listed below.</p><h2>Architecture</h2>",
        1,
    )
    page += "<p>The six geometry and working-set source files shrink from 806 to 629 lines (22%), with additional removals from optimizer state and graph utilities.</p>"
    page += "<table><thead><tr><th>Benchmark family</th><th>Paired completed cases</th><th>Geometric mean after / before</th></tr></thead><tbody>"
    for title, prefixes in (
        ("Original native examples", ("rss", "yetong")),
        ("G1 dynamics snapshots", ("full-",)),
        ("G1 walking trajectories", ("fixed-", "kinodynamics-")),
    ):
        group = [case for case in comparisons if case["case"].startswith(prefixes)]
        completed = [
            case
            for case in group
            if min(case["before_successful"], case["after_successful"]) == 5
            and "time_ratio" in case
        ]
        ratio = (
            math.exp(
                statistics.mean(math.log(case["time_ratio"]) for case in completed)
            )
            if completed
            else None
        )
        page += f"<tr><td>{title}</td><td>{len(completed)} / {len(group)}</td><td>{number(ratio)}</td></tr>"
    page += "</tbody></table><p>Each case contributes equally within its family. Ratios below one indicate shorter runtime. Censored and incomplete cases are excluded from these aggregates; all outcomes are listed below.</p>"
    page += "<p>Detailed observations:</p><pre>" + html.escape(notes) + "</pre>"
    page += """<button id="download-report">Download report (HTML)</button> <button id="download">Download comparison data (JSON)</button><br><input id="filter" placeholder="Filter cases"><section class="scroll"><table id="cases"><thead><tr>"""
    page += "".join(
        f"<th>{title}</th>"
        for title in [
            "Case",
            "Completed trials",
            "Solution",
            "Before s",
            "After s",
            "After / before",
            "Before cost",
            "After cost",
            "Before eq L2",
            "After eq L2",
            "Before ineq L2",
            "After ineq L2",
            "Iterations",
            "Max array Δ",
        ]
    )
    page += (
        "</tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table></section><h2>Failures and incomplete cases</h2><ul>"
        + failures
        + "</ul>"
    )
    if data["rechecks"]:
        initial_ratios = {case["case"]: case.get("time_ratio") for case in comparisons}
        page += "<h2>Timing rechecks</h2><p>Cases above the 5% timing threshold receive a fresh alternating batch. Both batches are included in the download.</p><table><tr><th>Case</th><th>Initial after / before</th><th>Recheck after / before</th><th>Solution</th></tr>"
        for batch in data["rechecks"]:
            for case in batch["cases"]:
                page += (
                    "<tr>"
                    + "".join(
                        f"<td>{html.escape(str(value))}</td>"
                        for value in [
                            case["case"],
                            number(initial_ratios.get(case["case"])),
                            number(case.get("time_ratio")),
                            (
                                "exact"
                                if case.get("solutions", {}).get("exact")
                                else "inspect data"
                            ),
                        ]
                    )
                    + "</tr>"
                )
        page += "</table>"
    page += "<h2>Reproduction</h2><p>The drivers and case inventory are in <code>examples/cmc_benchmarks</code>. The JSON download includes timing samples, outcome counts, residuals and array comparisons. Large raw arrays and logs remain in the benchmark output directories.</p>"
    page += (
        '<script id="data" type="application/json">'
        + encoded
        + """</script><script>const data=document.getElementById('data').textContent;const reportHtml='<!doctype html>'+document.documentElement.outerHTML;document.getElementById('download-report').onclick=()=>{const url=URL.createObjectURL(new Blob([reportHtml],{type:'text/html'}));const a=document.createElement('a');a.href=url;a.download='ComponentRefactorReport.html';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};document.getElementById('download').onclick=()=>{const url=URL.createObjectURL(new Blob([data],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='cmc-component-refactor.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};document.getElementById('filter').oninput=e=>{for(const row of document.querySelectorAll('#cases tbody tr'))row.hidden=!row.textContent.toLowerCase().includes(e.target.value.toLowerCase())};</script></html>"""
    )
    args.output.write_text(page)
    print(
        json.dumps(
            dict(
                cases=len(comparisons), paired=len(successful), exact=exact, close=close
            )
        )
    )


if __name__ == "__main__":
    main()
