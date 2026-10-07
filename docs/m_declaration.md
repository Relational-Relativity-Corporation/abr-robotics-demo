# M Declaration — ABR Flex Demo V0.5.2
Metatron Dynamics, Inc. — relationalrelativity.dev
Kernel V8 | 2026-10-06

---

## 1. Region Declaration

**Region:** Single-axis mechanical joint with electrical drive — load-bearing arm.

**Physical description:** A rigid arm of declared length, carrying a declared point load,
rotated about a fixed single-axis joint by an electrically driven motor. The arm operates
against gravity. The joint is the sole declared degree of freedom.

**Ground reference:** Earth gravity, 9.806 m/s². Base mount is rigid and fixed.

---

## 2. Observable Extent

The following quantities are admitted through M:

| Observable | Symbol | Units | Source |
|---|---|---|---|
| Load-side joint torque (sensor) | τ_s | N·m | Load-side torque sensor output |
| Joint angle | θ | rad | Angular position encoder |
| Motor voltage command | v_cmd | V | Actuator electrical input |

**Quantities NOT in observable extent (plant only):**
- MASS (kg) — point load mass
- ARM (m) — arm length
- G (m/s²) — gravitational acceleration
- I_EFF (kg·m²) — effective inertia
- DAMPING — joint damping coefficient

These quantities parameterize the plant simulation for ground-truth comparison only.
They do not enter the ABR operator path at any step.

---

## 3. Declared Electrical Topology

The following coupling constants are declared through M and appear in topology edges:

| Constant | Symbol | Value | Provenance |
|---|---|---|---|
| Bus voltage | V_BUS | 48.0 V | Declared drive specification |
| Winding resistance | R_W | 0.7 Ω | Declared motor constant |
| Torque constant | K_T | 1.0 N·m/A | Declared motor constant |
| Maximum electrical torque | TAU_ELEC_MAX | 68.57 N·m | V_BUS × K_T / R_W |
| Raise voltage authority | V_RAISE | 19.0 V | Declared raise parameter |

**Declared topology edges (coupling only — not operators):**

```
Edge tau_to_I : Delta_tau --(/ K_T)--> Delta_I
Edge I_to_V   : Delta_I   --(× R_W)--> Delta_V
```

These edges declare the relationship between torque change, current change, and voltage
change across the electrical-mechanical coupling. K_T and R_W carry this relationship.
They are not derived at runtime from MASS, ARM, or G.

---

## 4. Regional Admission Rule (M)

An observation τ_s is admitted if:

```
|τ_s| ≤ TAU_ELEC_MAX  (68.57 N·m)
```

Observations outside this bound are not processed. The admission boundary is derived
from declared electrical constants only.

---

## 5. Operator Chain Declaration

```
O_t  →  M  →  DeclaredRelations  →  A  →  B  →  [R: rho OPEN]  →  Sigma  →  C  →  u_t
```

**A operator** (directed difference at persistence edge):
```
A_out = τ_s(t) − τ_s(t−1)
```
A carries prior τ_s as state. Output is the directed torque difference — not the
absolute reading. Environmental quantities do not enter A.

**B operator** (topology traversal with A output):
```
delta_I = A_out / K_T    [edge tau_to_I]
delta_V = delta_I × R_W  [edge I_to_V]
```
B traverses the declared coupling edges with A's output. Only declared topology
constants (K_T, R_W) appear. Environmental quantities do not enter B.

**R operator** (antisymmetric relational operator):
- **STATUS: NOT EVALUATED** (OPEN, section 11.2)
- `rho_base` and `chi_0` not declared through M
- Not-evaluated ≠ evaluated with result 0.0 (declared-information rule, section 1)
- R is the formalization of the ongoing antisymmetric mismatch between fluctuating
  voltage and fluctuating torque — the `succ − pred` tension at each topology edge.
  The structure for R exists in the declared topology; the declaration of rho_base
  through M is the open condition.

**Sigma** (declared accumulation of B output):
```
sigma_v(t) = sigma_v(t−1) + delta_V
```
Sigma carries disturbance memory. After a load event, sigma_v is persistently
elevated. No reset between steps.

**C** (declared projection — not a kernel operator):
```
frac  = clamp((θ_target − θ) / θ_target, −1, 1)
v_cmd = sigma_v + frac × V_RAISE
```
- frac > 0 (below target): adds V_RAISE × frac — raise drive
- frac = 0 (at target): v_cmd = sigma_v — Sigma holds
- frac < 0 (above target): subtracts V_RAISE × |frac| — resist overshoot

---

## 6. Open Conditions

| ID | Description | Status |
|---|---|---|
| OC-FD-1 | `rho_base` — antisymmetric coupling weight between torque and voltage domains | OPEN — requires M declaration |
| OC-FD-2 | `chi_0` — initial antisymmetric state for R operator | OPEN — requires M declaration |

OC-FD-1 and OC-FD-2 together constitute the condition for R evaluation. The
topology structure for R is present. The declared weight of the antisymmetric
relationship between the torque domain and voltage domain has not been signed
through M.

---

## 7. Conventional Contrast

The conventional controller shown for structural comparison uses:

```
tau_cmd = Kp × (θ_target − θ) + Kd × (−ω) + MASS × G × ARM × sin(θ)
```

This formula requires MASS, ARM, and G at every step. These are environmental
quantities that do not appear in the ABR operator path. The honest note in the
demo source acknowledges that the conventional failure shown is tuning-specific
(Kd braking at disturbance moment), not a categorical claim.

---

*M declaration V1.0. Region: single-axis load-bearing arm with electrical drive.
Declared 2026-10-06.*
