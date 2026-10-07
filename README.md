# ABR Robotics Demo — Kernel V8 Flex Demonstration
**Metatron Dynamics, Inc.** | relationalrelativity.dev | V0.5.2 | 2026-10-06

---

## Experiment Question

Can an environmental change enter through a sensor, be processed by the actual
Kernel V8 relational operators (A, B), alter an actuator voltage command, change
the physical system, and consequently generate the next observation — without
inserting an undeclared control law or requiring MASS, ARM, or G in the operator path?

**PASS/FAIL: PASS** — V0.5.2

---

## What This Demonstrates

A single-axis load-bearing arm is controlled by the Kernel V8 operator chain.
A 25 N·m unannounced disturbance load is applied mid-task. The ABR path detects
and responds to the disturbance entirely through the sensor→operator→actuator chain.

The conventional PD + gravity-compensation controller is shown alongside for structural
comparison. Both receive the same task. Environmental quantities (MASS, ARM, G) appear
only in the plant simulation and the conventional controller — not in the ABR operator path.

**Runtime environment quantities in ABR operator path: 0**

---

## Operator Chain (Kernel V8 sections 10, 11)

```
O_t → M → DeclaredRelations → A → B → [R: rho OPEN] → Sigma → C → u_t
```

| Operator | Role | Status |
|---|---|---|
| **A** | Directed difference at persistence edge: `A_out = τ_s(t) − τ_s(t−1)` | Evaluated |
| **B** | Topology traversal with A output through declared coupling edges (K_T, R_W) | Evaluated |
| **R** | Antisymmetric operator — ongoing mismatch between fluctuating voltage and torque | **NOT EVALUATED** — `rho_base`, `chi_0` not declared through M (OPEN, section 11.2) |
| **Sigma** | Declared accumulation of B output across steps | Evaluated |
| **C** | Declared raise/resist projection (not a kernel operator) | Evaluated |

`not-evaluated ≠ evaluated with result 0.0` (declared-information rule, section 1)

---

## Declared Topology

```
Edge tau_to_I : Delta_tau --(/ K_T=1.0)--> Delta_I
Edge I_to_V   : Delta_I   --(× R_W=0.7)--> Delta_V
```

Coupling constants K_T (N·m/A) and R_W (Ω) declared through M.
These are motor constants — not derived from arm geometry or payload.

---

## Key Structural Distinction from V0.4

V0.4 evaluated absolute τ_s through coupling edges each step.
V0.5 applies A (directed difference), then B (topology traversal with delta),
accumulates with Sigma. Sigma carries disturbance memory across steps.

---

## Running

```bash
pip install -r requirements.txt
python abr_flex_demo.py
```

Produces:
- Console output: step-by-step operator trace, disturbance frame detail, PASS/FAIL summary
- `abr_flex_demo.png`: three-panel plot (joint angle, A_out/Sigma_v, v_cmd vs CONV tau_cmd)

---

## Open Conditions

| ID | Description |
|---|---|
| OC-FD-1 | `rho_base` — weight of antisymmetric coupling between torque and voltage domains |
| OC-FD-2 | `chi_0` — initial antisymmetric state for R operator |

R is the formalization of the ongoing antisymmetric mismatch between fluctuating voltage
and fluctuating torque. The topology structure for R is present in the declared edges.
M declaration of `rho_base` is the open condition.

---

## M Declaration

See [`docs/m_declaration.md`](docs/m_declaration.md) for full regional instantiation,
observable extent, declared topology, and admission rule.

---

## Honest Notes

- ABR settling angle depends on Sigma accumulation history — not guaranteed to be
  exactly at declared target. This is experimental evidence, not tuned output.
- CONV failure shown is tuning-specific (Kd braking at disturbance moment), not
  a categorical claim about conventional controllers.
- R not evaluated: `rho_base`, `chi_0` not declared through M (OPEN, section 11.2).

---

## Related Repositories

- [abr-kernel](https://github.com/Relational-Relativity-Corporation/abr-kernel) — Kernel V8 canonical operators (Rust)
- [abr-robotics-sim](https://github.com/Relational-Relativity-Corporation/abr-robotics-sim) — Single-joint crane, ABR vs conventional, Blender visualization
- [abr-fr3-joint-control](https://github.com/Relational-Relativity-Corporation/abr-fr3-joint-control) — FR3 robotic arm, Verifier-passed declaration

---

*Apache 2.0 License | Metatron Dynamics, Inc. | relationalrelativity.dev*
