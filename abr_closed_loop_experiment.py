#!/usr/bin/env python3
"""
ABR Closed-Loop Experiment — Controller P vs Controller K
Metatron Dynamics / ABR Minimal AI project

Generator:  Claude Sonnet 4.6
Origin:     Human (Relational Relativity Corporation)
Verifier:   ChatGPT

Purpose:
  Two-controller comparison on the abr-robotics-demo plant.

  Controller P: Python reference — the KernelV8 class from abr_flex_demo.py.
    Operator path: A(directed diff in tau_s) → B(topology traversal) → Sigma → C
    A input:  tau_s(t) − tau_s(t-1)  [N·m directed difference in sensed torque]
    B output: delta_V = delta_tau * R_W/K_T  [V]
    Sigma:    sigma_v += delta_V

  Controller K: Frozen Kernel V8 Rust operators via PyO3 (irrs_kernel_py).
    Declared 2-locus topology: v_τ(node 0) → v_cmd(node 1), Provenance: Coupling
    NodeField: x[v_τ] = tau_s × R_W/K_T [V]; x[v_cmd] = v_cmd_prior [V]
    A input:  A_K = x[v_τ] − x[v_cmd]  [V spatial gap]
    B output: B_K = A_K  [terminal edge, adj⁺=∅, identity]
    Sigma:    sigma_v += B_K

  Key distinction:
    P accumulates the change in tau_s (converted to V).
    K accumulates the voltage gap between sensor equivalent and prior command.
    K produces A_K(t+1) = Residual_V(t) at every step (traceability identity).
    P produces A_P_V(t+1) = Residual_V(t) + frac(t)×V_RAISE (structural gap).

  R is OPEN in both controllers:
    rho_base and chi_0 have not been declared through M.
    B output is NOT reported as a completed R result.
    OC-TRACE-1, OC-TRACE-2, OC-TRACE-3 remain open.

Run:
  cd abr-robotics-demo
  python3 abr_closed_loop_experiment.py

Outputs (in abr-robotics-demo/experiment_outputs/):
  controller_p_trace.json    — step-by-step trace, Controller P
  controller_k_trace.json    — step-by-step trace, Controller K
  comparison_report.md       — human-readable comparison and conformance
"""

import math
import sys
import os
import json
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Import plant from abr_flex_demo.py (in the same directory)
# ---------------------------------------------------------------------------

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR))

from abr_flex_demo import (
    integrate_plant,
    KernelV8, KernelV8Topology,
    c_projection,
    MASS, ARM, G, I_EFF, DAMPING,
    V_BUS, R_W, K_T, TAU_ELEC_MAX, V_RAISE,
    THETA_TARGET, THETA_CROSS, DIST_MAG,
    FPS, DT, TOTAL,
)

# ---------------------------------------------------------------------------
# Import Frozen Kernel V8 binding (built by maturin in irrs_kernel_py/)
# ---------------------------------------------------------------------------

try:
    import irrs_kernel_py as kernel_rs
except ImportError as exc:
    print("ERROR: irrs_kernel_py not installed.")
    print("Build and install it first:")
    print("  cd abr-minimal-ai/irrs_kernel_py")
    print("  maturin build --release")
    print("  pip3 install target/wheels/irrs_kernel_py-*.whl --break-system-packages")
    raise SystemExit(1) from exc

# ---------------------------------------------------------------------------
# Controller K: 2-locus DeclaredRelations (created once, reused each step)
# ---------------------------------------------------------------------------
# Topology: v_τ(0) → v_cmd(1), Provenance: Coupling (K_T, R_W)
# This declares the structural relation between the sensor voltage equivalent
# and the prior command voltage. It does NOT encode operator logic.

REL_K = kernel_rs.DeclaredRelations.from_edges(
    n_nodes=2,
    edges=[(0, 1)],
    provenance=[kernel_rs.PROVENANCE_COUPLING],
)

assert REL_K.n_edges() == 1, "Expected exactly 1 declared edge"
assert REL_K.is_isolated(), "Expected terminal edge (adj+ empty)"


# ---------------------------------------------------------------------------
# Controller K step function
# ---------------------------------------------------------------------------

def controller_k_step(tau_s: float, theta: float, v_cmd_prior: float,
                       sigma_v: float) -> dict:
    """
    Execute one step of Controller K using the Frozen Kernel V8 Rust operators.

    NodeField declared values (both in Volts):
      node 0 (v_τ):   x = tau_s × R_W/K_T   [V] — sensor torque as voltage equivalent
      node 1 (v_cmd): x = v_cmd_prior        [V] — prior commanded voltage

    A_K = x[v_τ] − x[v_cmd]  [V]  (spatial gap on declared Coupling edge)
    B_K = A_K  (terminal edge: adj⁺ = ∅, identity)
    R:   OPEN — rho_base, chi_0 not declared through M

    Returns: dict with all admitted inputs, operator outputs, Sigma, C projection.
    """
    # M admission check (same boundary as Controller P)
    if abs(tau_s) > TAU_ELEC_MAX:
        return {
            'admitted': False,
            'x_v_tau': None, 'x_v_cmd': None,
            'a_k': None, 'b_k': None, 'r_status': 'OPEN',
            'sigma_v': sigma_v,
            'frac': None, 'v_cmd_raw': None, 'v_cmd_clamped': None,
            'tau_cmd': None,
        }

    # Declared NodeField: data[component=0][node 0, node 1]
    x_v_tau = tau_s * R_W / K_T   # [V] sensor torque as voltage equivalent
    x_v_cmd = v_cmd_prior          # [V] prior command

    node_field = kernel_rs.NodeField([[x_v_tau, x_v_cmd]])

    # A operator (Frozen Kernel V8 Rust)
    a_field = kernel_rs.operator_a(node_field, REL_K, [])
    a_k = a_field.spatial[0][0]   # [V] = x_v_tau − x_v_cmd

    # B operator (Frozen Kernel V8 Rust)
    # Terminal edge (adj⁺ = ∅): B(g)[e] = g[e] (identity)
    b_field = kernel_rs.operator_b(a_field, REL_K)
    b_k = b_field.spatial[0][0]   # [V] = a_k (verified identity)

    # R: OPEN — rho_base and chi_0 not declared through M (section 11.2)
    # operator_r is NOT called. B output is NOT reported as completed R.
    r_status = 'OPEN'

    # Sigma: declared accumulation of B output
    sigma_v_new = sigma_v + b_k

    # C projection (same formula as Controller P)
    frac = max(-1.0, min(1.0, (THETA_TARGET - theta) / THETA_TARGET))
    v_cmd_raw = sigma_v_new + frac * V_RAISE
    v_cmd_clamped = max(-V_BUS, min(V_BUS, v_cmd_raw))
    tau_cmd = K_T * (v_cmd_clamped / R_W)

    return {
        'admitted': True,
        # Declared NodeField inputs
        'x_v_tau': x_v_tau,
        'x_v_cmd': x_v_cmd,
        # Operator outputs
        'a_k': a_k,
        'b_k': b_k,
        'r_status': r_status,
        # Sigma state (after update)
        'sigma_v': sigma_v_new,
        # C projection
        'frac': frac,
        'v_cmd_raw': v_cmd_raw,
        'v_cmd_clamped': v_cmd_clamped,
        'tau_cmd': tau_cmd,
    }


# ---------------------------------------------------------------------------
# Run experiment
# ---------------------------------------------------------------------------

def run_experiment() -> tuple[list, list]:
    """
    Run both controllers for TOTAL steps on independent plants.

    Returns:
      (trace_p, trace_k) — one record per step per controller.
    """

    # ── Controller P state ─────────────────────────────────────────────────
    p_theta, p_omega = 0.0, 0.0
    p_tau_s = MASS * G * ARM * math.sin(0.0)   # sensor primed from t=0 plant
    p_kernel = KernelV8(tau_s_init=p_tau_s, sigma_init=0.0)
    p_dist_active = False
    p_v_cmd_prior = 0.0   # for residual calculation

    # ── Controller K state ─────────────────────────────────────────────────
    k_theta, k_omega = 0.0, 0.0
    k_tau_s = MASS * G * ARM * math.sin(0.0)   # same initial sensor
    k_sigma_v = 0.0
    k_v_cmd_prior = 0.0   # prior command voltage for NodeField x[v_cmd]
    k_dist_active = False

    trace_p = []
    trace_k = []

    for f in range(1, TOTAL + 1):
        # ── Disturbance (independent per-plant) ────────────────────────────
        if not p_dist_active and p_theta >= THETA_CROSS:
            p_dist_active = True
        if not k_dist_active and k_theta >= THETA_CROSS:
            k_dist_active = True

        dist_p = DIST_MAG if p_dist_active else 0.0
        dist_k = DIST_MAG if k_dist_active else 0.0

        # ── Controller P step ──────────────────────────────────────────────
        t0_p = time.perf_counter()

        tau_s_in_p = p_tau_s
        sigma_v_p, a_out_p, delta_i_p, delta_v_p, _, b_trace_p, admitted_p = (
            p_kernel.step(p_tau_s, p_theta, KernelV8Topology)
        )

        if admitted_p:
            frac_p = max(-1.0, min(1.0, (THETA_TARGET - p_theta) / THETA_TARGET))
            v_cmd_p_raw = sigma_v_p + frac_p * V_RAISE
            v_cmd_p = max(-V_BUS, min(V_BUS, v_cmd_p_raw))
            tau_cmd_p = K_T * (v_cmd_p / R_W)
        else:
            frac_p = 0.0
            v_cmd_p_raw = 0.0
            v_cmd_p = 0.0
            tau_cmd_p = 0.0

        t1_p = time.perf_counter()

        # Advance Controller P plant
        p_theta_new, p_omega_new, p_tau_s_new = integrate_plant(
            p_theta, p_omega, tau_cmd_p, dist_p
        )

        # Controller P record
        residual_v_p = p_tau_s_new * R_W / K_T - v_cmd_p
        derived_winding_current_p = v_cmd_p / R_W
        trace_p.append({
            'step':                    f,
            't_s':                     round(f * DT, 6),
            # Physical state
            'joint_angle_rad':         round(p_theta, 8),
            'joint_angle_deg':         round(math.degrees(p_theta), 6),
            'angular_velocity_rads':   round(p_omega, 8),
            'load_side_torque_nm':     round(tau_s_in_p, 8),   # tau_s fed into A this step
            'disturbance_nm':          dist_p,
            'dist_active':             p_dist_active,
            # Actuator quantities
            'commanded_voltage_v':     round(v_cmd_p_raw, 8),
            'represented_voltage_v':   round(v_cmd_p, 8),
            'derived_winding_current_a': round(derived_winding_current_p, 8),
            'derived_motor_torque_nm': round(tau_cmd_p, 8),
            # Admitted relational inputs (M boundary)
            'admitted':                admitted_p,
            'tau_s_in_p_nm':           round(tau_s_in_p, 8),   # |tau_s| <= TAU_ELEC_MAX
            'tau_s_prior_nm':          round(p_kernel._tau_s_prior - a_out_p, 8) if admitted_p else None,
            # Operator outputs (P uses Python approximation of A, B)
            'a_out_p_nm':              round(a_out_p, 8) if admitted_p else None,
            'delta_i_p_a':             round(delta_i_p, 8) if admitted_p else None,
            'delta_v_p_v':             round(delta_v_p, 8) if admitted_p else None,
            'r_status':                'OPEN',
            # Sigma state
            'sigma_v_p_v':             round(sigma_v_p, 8) if admitted_p else None,
            # C projection
            'frac_p':                  round(frac_p, 8) if admitted_p else None,
            # Subsequent observation and residual
            'tau_s_next_nm':           round(p_tau_s_new, 8),
            'residual_v_v':            round(residual_v_p, 8),
            'residual_nm':             round(p_tau_s_new - tau_cmd_p, 8),
            # Computational work
            'controller_wall_ns':      round((t1_p - t0_p) * 1e9),
        })

        p_v_cmd_prior = v_cmd_p
        p_theta, p_omega, p_tau_s = p_theta_new, p_omega_new, p_tau_s_new

        # ── Controller K step ──────────────────────────────────────────────
        t0_k = time.perf_counter()

        tau_s_in_k = k_tau_s
        kout = controller_k_step(
            tau_s=k_tau_s,
            theta=k_theta,
            v_cmd_prior=k_v_cmd_prior,
            sigma_v=k_sigma_v,
        )

        t1_k = time.perf_counter()

        if kout['admitted']:
            tau_cmd_k = kout['tau_cmd']
            v_cmd_k = kout['v_cmd_clamped']
            k_sigma_v = kout['sigma_v']
        else:
            tau_cmd_k = 0.0
            v_cmd_k = 0.0

        # Advance Controller K plant
        k_theta_new, k_omega_new, k_tau_s_new = integrate_plant(
            k_theta, k_omega, tau_cmd_k, dist_k
        )

        # Controller K record
        residual_v_k = k_tau_s_new * R_W / K_T - v_cmd_k
        derived_winding_current_k = v_cmd_k / R_W
        trace_k.append({
            'step':                    f,
            't_s':                     round(f * DT, 6),
            # Physical state
            'joint_angle_rad':         round(k_theta, 8),
            'joint_angle_deg':         round(math.degrees(k_theta), 6),
            'angular_velocity_rads':   round(k_omega, 8),
            'load_side_torque_nm':     round(tau_s_in_k, 8),
            'disturbance_nm':          dist_k,
            'dist_active':             k_dist_active,
            # Actuator quantities
            'commanded_voltage_v':     round(kout['v_cmd_raw'], 8) if kout['admitted'] else None,
            'represented_voltage_v':   round(v_cmd_k, 8),
            'derived_winding_current_a': round(derived_winding_current_k, 8),
            'derived_motor_torque_nm': round(tau_cmd_k, 8),
            # Admitted relational inputs (M boundary; both in Volts)
            'admitted':                kout['admitted'],
            'x_v_tau_v':               round(kout['x_v_tau'], 8) if kout['admitted'] else None,
            'x_v_cmd_prior_v':         round(kout['x_v_cmd'], 8) if kout['admitted'] else None,
            # Operator outputs (Rust Frozen Kernel V8)
            'a_k_v':                   round(kout['a_k'], 8) if kout['admitted'] else None,
            'b_k_v':                   round(kout['b_k'], 8) if kout['admitted'] else None,
            'r_status':                kout['r_status'],
            # Persistence state (sigma_v is the declared accumulator)
            'sigma_v_k_v':             round(kout['sigma_v'], 8) if kout['admitted'] else None,
            # C projection
            'frac_k':                  round(kout['frac'], 8) if kout['admitted'] else None,
            # Subsequent observation and residual
            'tau_s_next_nm':           round(k_tau_s_new, 8),
            'residual_v_v':            round(residual_v_k, 8),
            'residual_nm':             round(k_tau_s_new - tau_cmd_k, 8),
            # Conformance check: A_K(t) == Residual_V(t-1)?
            # Populated on the NEXT step; filled in post-loop below.
            'trace_k_identity_check':  None,
            # Computational work
            'controller_wall_ns':      round((t1_k - t0_k) * 1e9),
        })

        k_v_cmd_prior = v_cmd_k
        k_theta, k_omega, k_tau_s = k_theta_new, k_omega_new, k_tau_s_new

    # ── Post-loop: verify Trace K identity A_K(t) = Residual_V(t-1) ──────
    # Residual_V(t-1) is stored in trace_k[t-2]['residual_v_v'] (0-indexed).
    # A_K(t) is in trace_k[t-1]['a_k_v'].
    for i in range(1, len(trace_k)):
        r_prev = trace_k[i - 1]['residual_v_v']
        a_curr = trace_k[i]['a_k_v']
        if r_prev is not None and a_curr is not None:
            err = abs(a_curr - r_prev)
            trace_k[i]['trace_k_identity_check'] = {
                'a_k_t': a_curr,
                'residual_v_prev': r_prev,
                'error': round(err, 10),
                'identity_holds': err < 1e-9,
            }

    return trace_p, trace_k


# ---------------------------------------------------------------------------
# Save traces
# ---------------------------------------------------------------------------

def save_traces(trace_p: list, trace_k: list, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    p_path = out_dir / 'controller_p_trace.json'
    k_path = out_dir / 'controller_k_trace.json'
    with open(p_path, 'w') as fh:
        json.dump({'controller': 'P', 'steps': trace_p}, fh, indent=2)
    with open(k_path, 'w') as fh:
        json.dump({'controller': 'K', 'steps': trace_k}, fh, indent=2)
    return p_path, k_path


# ---------------------------------------------------------------------------
# Comparison report
# ---------------------------------------------------------------------------

def build_report(trace_p: list, trace_k: list) -> str:
    lines = []
    a = lines.append

    def hdr(title, char='─'):
        a(f'\n## {title}')

    a('# ABR Closed-Loop Experiment — Comparison Report')
    a('')
    a('Generator: Claude Sonnet 4.6  |  Origin: Human (Relational Relativity Corp)')
    a(f'Steps: {TOTAL}  |  dt = {DT:.6f} s  |  Total time: {TOTAL*DT:.4f} s')
    a('')
    a('---')

    # Setup
    hdr('1. Setup')
    a('')
    a('**Plant parameters (shared)**')
    a(f'- MASS={MASS} kg, ARM={ARM} m, G={G} m/s²')
    a(f'- I_EFF={I_EFF} kg·m², DAMPING={DAMPING} N·m·s/rad')
    a(f'- R_W={R_W} Ω, K_T={K_T} N·m/A, V_BUS={V_BUS} V')
    a(f'- THETA_TARGET={math.degrees(THETA_TARGET):.1f}°, THETA_CROSS={math.degrees(THETA_CROSS):.1f}°')
    a(f'- DIST_MAG={DIST_MAG} N·m (applied when θ ≥ THETA_CROSS, independent per plant)')
    a(f'- V_RAISE={V_RAISE} V, TAU_ELEC_MAX={V_BUS*K_T/R_W:.3f} N·m')
    a('')
    a('**Controller P** — Python KernelV8 class (abr_flex_demo.py)')
    a('- A: a_out = tau_s(t) − tau_s(t−1)  [N·m directed difference in sensed torque]')
    a('- B: delta_V = a_out × R_W/K_T  [V] (traverse declared K_T, R_W topology)')
    a('- R: OPEN (rho_base, chi_0 not declared through M)')
    a('- Sigma: sigma_v += delta_V')
    a('')
    a('**Controller K** — Frozen Kernel V8 Rust operators (irrs_kernel_py / abr-irrs)')
    a('- Declared topology: v_τ(0) → v_cmd(1), Provenance=Coupling, 1 edge')
    a('- NodeField: x[v_τ] = tau_s × R_W/K_T [V]; x[v_cmd] = v_cmd_prior [V]')
    a('- A: A_K = x[v_τ] − x[v_cmd]  [V] (Rust operator_a via PyO3)')
    a('- B: B_K = A_K (terminal edge identity; Rust operator_b via PyO3)')
    a('- R: OPEN (rho_base, chi_0 not declared through M; operator_r NOT called)')
    a('- Sigma: sigma_v += B_K')
    a('')
    a('**C projection (both controllers)**')
    a('- frac = clamp((THETA_TARGET − θ) / THETA_TARGET, −1, 1)')
    a('- v_cmd = sigma_v + frac × V_RAISE, clamped to [−V_BUS, +V_BUS]')
    a('- tau_cmd = K_T × v_cmd / R_W')

    # Final state
    final_p = trace_p[-1]
    final_k = trace_k[-1]

    hdr('2. Final State Comparison')
    a('')
    a(f'| Quantity | Controller P | Controller K |')
    a(f'|---|---|---|')
    a(f'| Joint angle (°) | {final_p["joint_angle_deg"]:.4f} | {final_k["joint_angle_deg"]:.4f} |')
    a(f'| Angular velocity (rad/s) | {final_p["angular_velocity_rads"]:.4f} | {final_k["angular_velocity_rads"]:.4f} |')
    a(f'| Target error (°) | {math.degrees(THETA_TARGET)-final_p["joint_angle_deg"]:.4f} | {math.degrees(THETA_TARGET)-final_k["joint_angle_deg"]:.4f} |')
    a(f'| sigma_v (V) | {final_p["sigma_v_p_v"]:.4f} | {final_k["sigma_v_k_v"]:.4f} |')
    a(f'| v_cmd (V) | {final_p["represented_voltage_v"]:.4f} | {final_k["represented_voltage_v"]:.4f} |')
    a(f'| tau_cmd (N·m) | {final_p["derived_motor_torque_nm"]:.4f} | {final_k["derived_motor_torque_nm"]:.4f} |')

    # Disturbance response
    hdr('3. Disturbance Response')
    a('')
    dist_p_rows = [r for r in trace_p if r['dist_active']]
    dist_k_rows = [r for r in trace_k if r['dist_active']]
    p_dist_step = dist_p_rows[0]['step'] if dist_p_rows else None
    k_dist_step = dist_k_rows[0]['step'] if dist_k_rows else None
    a(f'- Controller P disturbance onset: step {p_dist_step} (t={p_dist_step*DT:.3f} s)' if p_dist_step else '- Controller P: disturbance never triggered')
    a(f'- Controller K disturbance onset: step {k_dist_step} (t={k_dist_step*DT:.3f} s)' if k_dist_step else '- Controller K: disturbance never triggered')
    a('')

    if dist_p_rows:
        peak_p = max(dist_p_rows[:12], key=lambda r: abs(r['a_out_p_nm'] or 0))
        a(f'**Controller P peak A response** (step {peak_p["step"]}, θ={peak_p["joint_angle_deg"]:.2f}°):')
        a(f'- A_out = {peak_p["a_out_p_nm"]:.4f} N·m')
        a(f'- delta_V = {peak_p["delta_v_p_v"]:.4f} V')
        a(f'- sigma_v = {peak_p["sigma_v_p_v"]:.4f} V')
        a(f'- v_cmd = {peak_p["represented_voltage_v"]:.3f} V')
        a('')

    if dist_k_rows:
        peak_k = max(dist_k_rows[:12], key=lambda r: abs(r['a_k_v'] or 0))
        a(f'**Controller K peak A response** (step {peak_k["step"]}, θ={peak_k["joint_angle_deg"]:.2f}°):')
        a(f'- A_K = {peak_k["a_k_v"]:.4f} V (Rust operator_a)')
        a(f'- B_K = {peak_k["b_k_v"]:.4f} V (Rust operator_b, terminal edge identity)')
        a(f'- sigma_v = {peak_k["sigma_v_k_v"]:.4f} V')
        a(f'- v_cmd = {peak_k["represented_voltage_v"]:.3f} V')
        a('')

    # Trace K identity verification
    hdr('4. Trace K Identity: A_K(t) = Residual_V(t-1)')
    a('')
    a('From the traceability matrix: A_K(t+1) = tau_s(t+1)×R_W/K_T − v_cmd(t)')
    a('This is the voltage residual of the prior step — the gap between what the sensor')
    a('reports as needed and what was commanded.')
    a('')

    checks = [r['trace_k_identity_check'] for r in trace_k
              if r['trace_k_identity_check'] is not None]
    if checks:
        errors = [c['error'] for c in checks]
        max_err = max(errors)
        all_hold = all(c['identity_holds'] for c in checks)
        a(f'- Steps checked: {len(checks)}')
        a(f'- Maximum deviation: {max_err:.2e} V')
        a(f'- Identity holds at all steps (< 1e-9 V): **{"YES" if all_hold else "NO"}**')
        a('')
        a('Sample (first 6 steps with prior residual):')
        a('| Step | A_K(t) [V] | Residual_V(t-1) [V] | Error [V] |')
        a('|---|---|---|---|')
        for c in checks[:6]:
            a(f'| — | {c["a_k_t"]:.6f} | {c["residual_v_prev"]:.6f} | {c["error"]:.2e} |')
    else:
        a('- No identity checks available.')

    # Stability comparison
    hdr('5. Stability and Settling')
    a('')
    # Target error over last 24 steps (1 second)
    p_last = trace_p[-FPS:]
    k_last = trace_k[-FPS:]
    p_err_last = [abs(math.degrees(THETA_TARGET) - r['joint_angle_deg']) for r in p_last]
    k_err_last = [abs(math.degrees(THETA_TARGET) - r['joint_angle_deg']) for r in k_last]
    p_omega_last = [abs(r['angular_velocity_rads']) for r in p_last]
    k_omega_last = [abs(r['angular_velocity_rads']) for r in k_last]
    a(f'Final 1-second window (last {FPS} steps):')
    a(f'| Metric | Controller P | Controller K |')
    a(f'|---|---|---|')
    a(f'| Mean target error (°) | {sum(p_err_last)/len(p_err_last):.4f} | {sum(k_err_last)/len(k_err_last):.4f} |')
    a(f'| Max target error (°) | {max(p_err_last):.4f} | {max(k_err_last):.4f} |')
    a(f'| Mean |ω| (rad/s) | {sum(p_omega_last)/len(p_omega_last):.4f} | {sum(k_omega_last)/len(k_omega_last):.4f} |')
    a(f'| Max |ω| (rad/s) | {max(p_omega_last):.4f} | {max(k_omega_last):.4f} |')

    # Computational work
    hdr('6. Computational Work')
    a('')
    p_ns = [r['controller_wall_ns'] for r in trace_p]
    k_ns = [r['controller_wall_ns'] for r in trace_k]
    a(f'| Metric | Controller P | Controller K |')
    a(f'|---|---|---|')
    a(f'| Total wall time (µs) | {sum(p_ns)/1000:.1f} | {sum(k_ns)/1000:.1f} |')
    a(f'| Mean per step (ns) | {sum(p_ns)//len(p_ns)} | {sum(k_ns)//len(k_ns)} |')
    a(f'| Max per step (ns) | {max(p_ns)} | {max(k_ns)} |')
    a('')
    a('Note: Controller K includes Python→Rust FFI overhead per step.')
    a('The frozen Rust kernel compile time is not included.')

    # Conformance
    hdr('7. Conformance Assessment (PASS / FAIL / OPEN)')
    a('')
    a('| Condition | Controller P | Controller K | Status |')
    a('|---|---|---|---|')

    # Identity check
    all_hold = all(c['identity_holds'] for c in checks) if checks else False
    a(f'| Trace K identity: A_K(t+1) = Residual_V(t) | N/A | {"PASS" if all_hold else "FAIL"} | OC-TRACE-1 dependency |')

    # R OPEN
    a('| R not evaluated (rho_base, chi_0 undeclared) | OPEN | OPEN | OC-TRACE-2, OC-TRACE-3 |')

    # B not reported as R
    a('| B output reported separately from R | PASS | PASS | Explicit in trace |')

    # Operator authenticity
    a('| Operator authenticity: actual Frozen Kernel V8 Rust used | Python approx. | PASS (PyO3 binding) | Required by Origin |')

    # Separate plants
    a('| Controllers drive independent plants | PASS | PASS | Separate plant state |')

    # Independent disturbance
    a('| Independent disturbance triggers per plant | PASS | PASS | Per-theta trigger |')

    # Machine-readable
    a('| Machine-readable step traces (JSON) | PASS | PASS | Saved to experiment_outputs/ |')

    # OC-TRACE open conditions
    a('')
    a('**Open conditions (not resolvable here — require Origin declaration):**')
    a('- OC-TRACE-1: Structural gap A_P_V(t+1) = Residual_V(t) + frac(t)×V_RAISE not eliminated.')
    a('  Cause: Controller P accumulates Δtau_s, not the full voltage residual.')
    a('- OC-TRACE-2: ρ_base not declared through M. R not evaluated.')
    a('- OC-TRACE-3: χ₀ not declared through M. ρ computation blocked.')
    a('- OC-SIM-1: Controller K drives a different plant trajectory than Controller P.')
    a('  The two controllers are structurally distinct and produce distinct closed-loop behaviors.')
    a('  Convergence, stability, and target accuracy of Controller K are observed results')
    a('  of this experiment — not guaranteed by the declaration.')

    a('')
    a('---')
    a(f'*Report generated by abr_closed_loop_experiment.py*')

    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Console output
# ---------------------------------------------------------------------------

def print_console_summary(trace_p: list, trace_k: list):
    print()
    print('ABR CLOSED-LOOP EXPERIMENT')
    print('=' * 72)
    print(f'  Controllers: P (Python KernelV8) vs K (Rust Frozen Kernel V8)')
    print(f'  Steps: {TOTAL}  |  dt = {DT:.4f} s  |  Total: {TOTAL*DT:.2f} s')
    print(f'  Disturbance: {DIST_MAG:.0f} N·m at θ ≥ {math.degrees(THETA_CROSS):.0f}°  (independent per plant)')
    print()
    print(f'  {'Step':>5}  {'P-θ°':>7}  {'K-θ°':>7}  {'P-σ_v':>8}  {'K-σ_v':>8}  '
          f'{'P-v_cmd':>8}  {'K-v_cmd':>8}  {'dist_P':>6}  {'dist_K':>6}')
    print(f'  {"-"*5}  {"-"*7}  {"-"*7}  {"-"*8}  {"-"*8}  {"-"*8}  {"-"*8}  {"-"*6}  {"-"*6}')

    show_steps = set(range(1, 11)) | {r['step'] for r in trace_p if r['dist_active']}
    show_steps |= {TOTAL}
    for i, (rp, rk) in enumerate(zip(trace_p, trace_k)):
        if rp['step'] in show_steps:
            flag_p = ' DIST' if (rp['dist_active'] and (i == 0 or not trace_p[i-1]['dist_active'])) else ''
            print(f"  {rp['step']:>5}  "
                  f"{rp['joint_angle_deg']:>7.2f}  "
                  f"{rk['joint_angle_deg']:>7.2f}  "
                  f"{(rp['sigma_v_p_v'] or 0):>8.3f}  "
                  f"{(rk['sigma_v_k_v'] or 0):>8.3f}  "
                  f"{rp['represented_voltage_v']:>8.3f}  "
                  f"{rk['represented_voltage_v']:>8.3f}  "
                  f"{'ON' if rp['dist_active'] else 'off':>6}  "
                  f"{'ON' if rk['dist_active'] else 'off':>6}"
                  f"{flag_p}")

    print()
    fp, fk = trace_p[-1], trace_k[-1]
    print(f'  FINAL:')
    print(f'    P: θ={fp["joint_angle_deg"]:.3f}°  ω={fp["angular_velocity_rads"]:.4f} rad/s  '
          f'σ_v={fp["sigma_v_p_v"]:.3f} V  v_cmd={fp["represented_voltage_v"]:.3f} V')
    print(f'    K: θ={fk["joint_angle_deg"]:.3f}°  ω={fk["angular_velocity_rads"]:.4f} rad/s  '
          f'σ_v={fk["sigma_v_k_v"]:.3f} V  v_cmd={fk["represented_voltage_v"]:.3f} V')
    print(f'    Target: {math.degrees(THETA_TARGET):.3f}°')
    print(f'    Error P: {math.degrees(THETA_TARGET)-fp["joint_angle_deg"]:+.4f}°')
    print(f'    Error K: {math.degrees(THETA_TARGET)-fk["joint_angle_deg"]:+.4f}°')
    print()

    # Trace K identity
    checks = [r['trace_k_identity_check'] for r in trace_k
              if r['trace_k_identity_check'] is not None]
    if checks:
        max_err = max(c['error'] for c in checks)
        all_hold = all(c['identity_holds'] for c in checks)
        print(f'  TRACE K IDENTITY (A_K(t+1) = Residual_V(t)):')
        print(f'    Checked {len(checks)} steps  |  max error: {max_err:.2e} V  |  '
              f'{"ALL HOLD" if all_hold else "SOME FAIL"}')

    print()
    print('  R: OPEN in both controllers (rho_base, chi_0 not declared through M)')
    print('  B output is NOT reported as a completed R result.')
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    out_dir = SCRIPT_DIR / 'experiment_outputs'

    print('Running experiment...')
    t_start = time.perf_counter()
    trace_p, trace_k = run_experiment()
    t_end = time.perf_counter()
    print(f'  Completed in {(t_end-t_start)*1000:.1f} ms')

    print_console_summary(trace_p, trace_k)

    p_path, k_path = save_traces(trace_p, trace_k, out_dir)
    print(f'  Traces saved:')
    print(f'    {p_path}')
    print(f'    {k_path}')

    report = build_report(trace_p, trace_k)
    report_path = out_dir / 'comparison_report.md'
    with open(report_path, 'w') as fh:
        fh.write(report)
    print(f'  Report saved: {report_path}')
    print()
