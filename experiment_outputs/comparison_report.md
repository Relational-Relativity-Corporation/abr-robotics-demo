# ABR Closed-Loop Experiment — Comparison Report

Generator: Claude Sonnet 4.6  |  Origin: Human (Relational Relativity Corp)
Steps: 144  |  dt = 0.041667 s  |  Total time: 6.0000 s

*Refactored under the Principle of Observational Continuity.*
*K_T and R_W are now declared Coupling edges. τ_s enters the declared domain*
*in its physical units [N·m] at node 0. The voltage gap emerges through*
*declared topology traversal (A.spatial[2][2] on terminal edge e_2).*

---

## 1. Setup

**Plant parameters (shared)**
- MASS=3.0 kg, ARM=0.5 m, G=9.806 m/s²
- I_EFF=0.75 kg·m², DAMPING=0.5 N·m·s/rad
- R_W=0.7 Ω, K_T=1.0 N·m/A, V_BUS=48.0 V
- THETA_TARGET=20.0°, THETA_CROSS=10.0°
- DIST_MAG=25.0 N·m (applied when θ ≥ THETA_CROSS, independent per plant)
- V_RAISE=19.0 V, TAU_ELEC_MAX=68.571 N·m

**Controller P** — Python KernelV8 class (abr_flex_demo.py)
- A: a_out = tau_s(t) − tau_s(t−1)  [N·m directed difference in sensed torque]
- B: delta_V = a_out × R_W/K_T  [V] (traverse declared K_T, R_W topology)
- R: OPEN (rho_base, chi_0 not declared through M)
- Sigma: sigma_v += delta_V

**Controller K** — Frozen Kernel V8 Rust operators (irrs_kernel_py / abr-irrs)

  Declared topology (Principle of Observational Continuity):
  ```
  Node 0: load        — τ_s [N·m]       (primary observable)
  Node 1: motor shaft — i_req [A]        (K_T coupling: i = τ/K_T)
  Node 2: winding     — v_req [V]        (R_W coupling: v = i×R_W)
  Node 3: driver      — v_cmd_prior [V]  (prior actuator state, observable)

  e_0: (0→1) Coupling  K_T  [torque → current]
  e_1: (1→2) Coupling  R_W  [current → voltage]
  e_2: (2→3) Coupling       [required voltage vs commanded, terminal]
  ```

  NodeField (k=3 components):
  - comp 0 [N·m]:  [τ_s,  0,       0,       0        ]
  - comp 1 [A]:    [0,    τ_s/K_T, 0,       0        ]
  - comp 2 [V]:    [0,    0,       τ_s·R_W/K_T, v_cmd]

  A (Rust operator_a): A.spatial[2][2] = τ_s·R_W/K_T − v_cmd_prior  [V]
  B (Rust operator_b): B.spatial[2][2] = A.spatial[2][2]  [V] (e_2 terminal, identity)
  R: OPEN (rho_base, chi_0 not declared through M; operator_r NOT called)
  Sigma: sigma_v += B.spatial[2][2]

**C projection (both controllers)**
- frac = clamp((THETA_TARGET − θ) / THETA_TARGET, −1, 1)
- v_cmd = sigma_v + frac × V_RAISE, clamped to [−V_BUS, +V_BUS]
- tau_cmd = K_T × v_cmd / R_W

## 2. Final State Comparison

| Quantity | Controller P | Controller K |
|---|---|---|
| Joint angle (°) | 19.9084 | 1.1753 |
| Angular velocity (rad/s) | -0.0214 | -0.0000 |
| Target error (°) | 0.0916 | 18.8247 |
| sigma_v (V) | 21.0147 | -0.1723 |
| v_cmd (V) | 21.1017 | 17.7112 |
| tau_cmd (N·m) | 30.1453 | 25.3017 |

## 3. Disturbance Response

- Controller P disturbance onset: step 4 (t=0.167 s)
- Controller K disturbance onset: step 6 (t=0.250 s)

**Controller P peak A response** (step 5, θ=22.06°):
- A_out = 26.9312 N·m
- delta_V = 18.8519 V
- sigma_v = 20.6273 V
- v_cmd = 18.666 V

**Controller K peak A response** (step 7, θ=8.75°):
- A[e_2, voltage] = 18.8417 V (Rust operator_a, terminal edge)
- B[e_2, voltage] = 18.8417 V (Rust operator_b, terminal identity)
- sigma_v = 11.0948 V
- v_cmd = 21.784 V


## 4. Trace K Identity: A[e_2,volt](t) = Residual_V(t-1)

From the declared topology: A.spatial[2][2] at step t equals
τ_s(t)·R_W/K_T − v_cmd(t−1). The voltage residual of the prior step is
τ_s(t)·R_W/K_T − v_cmd(t−1). These are equal by construction.

The identity is preserved under the refactored declaration because the
voltage gap at the terminal edge e_2 is the same quantity, now derived
through declared topology traversal rather than pre-computation.

- Steps checked: 143
- Maximum deviation: 0.00e+00 V
- Identity holds at all steps (< 1e-9 V): **YES**

Sample (first 6 steps with prior residual):
| Step | A[e_2,volt](t) [V] | Residual_V(t-1) [V] | Error [V] |
|---|---|---|---|
| — | -19.000000 | -19.000000 | 0.00e+00 |
| — | 4.066435 | 4.066435 | 0.00e+00 |
| — | 3.081604 | 3.081604 | 0.00e+00 |
| — | 2.335147 | 2.335147 | 0.00e+00 |
| — | 1.769832 | 1.769832 | 0.00e+00 |
| — | 18.841739 | 18.841739 | 0.00e+00 |

## 5. Stability and Settling

Final 1-second window (last 24 steps):
| Metric | Controller P | Controller K |
|---|---|---|
| Mean target error (°) | 0.1254 | 18.8247 |
| Max target error (°) | 0.2560 | 18.8247 |
| Mean |ω| (rad/s) | 0.0245 | 0.0000 |
| Max |ω| (rad/s) | 0.0516 | 0.0000 |

## 6. Computational Work

| Metric | Controller P | Controller K |
|---|---|---|
| Total wall time (µs) | 395.0 | 5424.3 |
| Mean per step (ns) | 2743 | 37668 |
| Max per step (ns) | 27755 | 499146 |

Note: Controller K includes Python→Rust FFI overhead per step.
The frozen Rust kernel compile time is not included.

## 7. Conformance Assessment (PASS / FAIL / OPEN)

| Condition | Controller P | Controller K | Status |
|---|---|---|---|
| Trace K identity: A[e_2,volt](t+1) = Residual_V(t) | N/A | PASS | OC-TRACE-1 dependency |
| K_T, R_W declared as Coupling edges (not pre-computed) | N/A | PASS | Observational Continuity |
| τ_s [N·m] enters declared domain at node 0 in physical units | N/A | PASS | Operator fidelity |
| R not evaluated (rho_base, chi_0 undeclared) | OPEN | OPEN | OC-TRACE-2, OC-TRACE-3 |
| B output reported separately from R | PASS | PASS | Explicit in trace |
| Operator authenticity: actual Frozen Kernel V8 Rust used | Python approx. | PASS (PyO3 binding) | Required by Origin |
| Controllers drive independent plants | PASS | PASS | Separate plant state |
| Independent disturbance triggers per plant | PASS | PASS | Per-theta trigger |
| Machine-readable step traces (JSON) | PASS | PASS | Saved to experiment_outputs/ |
| Full A/B spatial fields (3×3) in trace for provenance | N/A | PASS | Observational closure |

**Open conditions (not resolvable here — require Origin declaration):**
- OC-TRACE-1: Structural gap A_P_V(t+1) = Residual_V(t) + frac(t)×V_RAISE not eliminated.
  Cause: Controller P accumulates Δtau_s, not the full voltage residual.
- OC-TRACE-2: ρ_base not declared through M. R not evaluated.
- OC-TRACE-3: χ₀ not declared through M. ρ computation blocked.
- OC-SIM-1: Controller K drives a different plant trajectory than Controller P.
  The two controllers are structurally distinct and produce distinct closed-loop behaviors.
  Convergence, stability, and target accuracy of Controller K are observed results
  of this experiment — not guaranteed by the declaration.

---
*Report generated by abr_closed_loop_experiment.py*
*Refactored under the Principle of Observational Continuity.*