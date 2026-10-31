# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file timing.py
# @brief Warm forward-dynamics timings; model setup and state transfers are excluded.
# @author Frank Dellaert

"""Warm forward-dynamics timings; model setup and state transfers are excluded."""

from time import perf_counter_ns

import mujoco
import numpy as np


def statistics(seconds):
    return {
        "seconds": float(np.median(seconds)),
        "p10_seconds": float(np.percentile(seconds, 10)),
        "p90_seconds": float(np.percentile(seconds, 90)),
        "repeats": len(seconds),
    }


def native_forward(model, data, warmup, repeats):
    """Time complete CPU forward calls with a zero solver warm start each time."""
    times = []
    for iteration in range(warmup + repeats):
        data.qacc_warmstart[:] = 0
        start = perf_counter_ns()
        mujoco.mj_forward(model, data)
        elapsed = (perf_counter_ns() - start) * 1e-9
        if iteration >= warmup:
            times.append(elapsed)
    return statistics(times)


def warp_forward(model, walk, indices, warmup, repeats):
    """Measure MJLab's CUDA-graph forward path at one and N recorded worlds.

    Synchronized wall time includes graph launch and completion. CUDA events
    also record device elapsed time. Each graph resets qacc_warmstart so repeats
    do not reuse a converged solver initial guess. No simulation step is taken.
    """
    import mujoco_warp as mjw
    import warp as wp

    def set_states(data, selected):
        for name in ("qpos", "qvel", "ctrl"):
            destination = getattr(data, name)
            wp.copy(
                destination, wp.array(walk[name][selected], dtype=destination.dtype)
            )

    def capture(data):
        # Compile/load kernels and complete lazy initialization before capture.
        data.qacc_warmstart.zero_()
        mjw.forward(warp_model, data)
        wp.synchronize_device()
        with wp.ScopedCapture() as scope:
            data.qacc_warmstart.zero_()
            mjw.forward(warp_model, data)
        return scope.graph

    def measure(graph):
        for _ in range(warmup):
            wp.capture_launch(graph)
        wp.synchronize_device()
        wall_times, device_times = [], []
        start_event = wp.Event(enable_timing=True)
        end_event = wp.Event(enable_timing=True)
        for _ in range(repeats):
            start = perf_counter_ns()
            wp.record_event(start_event)
            wp.capture_launch(graph)
            wp.record_event(end_event)
            wp.synchronize_device()
            wall_times.append((perf_counter_ns() - start) * 1e-9)
            device_times.append(
                wp.get_event_elapsed_time(start_event, end_event) * 1e-3
            )
        result = statistics(wall_times)
        result["device_seconds"] = statistics(device_times)["seconds"]
        return result

    with wp.ScopedDevice("cuda:0"):
        warp_model = mjw.put_model(model)
        single = mjw.put_data(
            model, mujoco.MjData(model), nworld=1, nconmax=128, njmax=512
        )
        set_states(single, indices[:1])
        single_graph = capture(single)
        individual = []
        for index in indices:
            set_states(single, [index])
            result = measure(single_graph)
            result["acceleration"] = single.qacc.numpy()[0].astype(float).tolist()
            individual.append(result)

        batch = mjw.put_data(
            model, mujoco.MjData(model), nworld=len(indices), nconmax=128, njmax=512
        )
        set_states(batch, indices)
        batch_graph = capture(batch)
        batched = measure(batch_graph)
        batched["worlds"] = len(indices)
        batched["seconds_per_world"] = batched["seconds"] / len(indices)
        batched["device_seconds_per_world"] = batched["device_seconds"] / len(indices)
        batched["acceleration"] = batch.qacc.numpy().astype(float).tolist()
        return {
            "device": wp.get_device().name,
            "scalar_type": single.qacc.dtype.__name__,
            "single": individual,
            "batch": batched,
            "scope": "CUDA-graph mjw.forward plus zeroing qacc_warmstart; synchronized wall and CUDA-event times",
            "warmup": warmup,
            "state_transfers_timed": False,
        }
