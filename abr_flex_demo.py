#!/usr/bin/env python3
"""
ABR Flex Demo -- Kernel V8 Operator Execution Under Unannounced Environmental Load
Metatron Dynamics, Inc. -- relationalrelativity.dev
V0.5.2 -- 2026-10-06. Kernel V8.

Pass/fail experiment:
  Can an environmental change enter through a sensor, be processed by the
  actual Kernel V8 relational operators (A, B), alter an actuator command,
  change the physical system, and consequently generate the next observation --
  without inserting an undeclared control law?

  (R not evaluated: rho_base, chi_0 not declared through M -- OPEN, section 11.2)

Pipeline each step (Kernel V8 section 10, 11.1):
  O_t --> M --> DeclaredRelations_t --> A --> B --> [R: rho OPEN] --> Sigma --> C --> u_t

  O_t:             sensor observations at time t (tau_s from prior plant output, theta)
  M:               admission check -- sensor within declared electrical extent
  DeclaredRelations: coupling edges (K_T, R_W) -- declared topology, NOT operators
  A [operator]:    directed difference at persistence edge
                     A_out = tau_s(t) - tau_s(t-1)    [section 11.1: A(x)[e] = x[s] - x[t]]
  B [operator]:    traverse declared topology with A output
                     delta_I = A_out / K_T    [edge tau_to_I, declared K_T]
                     delta_V = delta_I * R_W  [edge I_to_V,   declared R_W]
                   [section 11.1: B(g)[e] = g[e] + sum_{succ(e)} g[f]]
  R [operator]:    not evaluated -- rho_base, chi_0 not declared through M
                   [section 11.2: OPEN -- Origin declaration required]
  Sigma:           declared accumulation of B output:
                     sigma_v(t) = sigma_v(t-1) + delta_V
  C:               declared projection (section 11.1 -- not a kernel operator):
                     frac = clamp((target - theta) / target, -1, 1)
                     v_cmd = sigma_v + frac * V_RAISE  (+raise / 0 hold / -resist)
  u_t:             final clamped actuator voltage command

Key distinction from V0.4:
  V0.4 evaluated absolute tau_s through coupling edges each step.
  V0.5 applies A (directed difference), then B (topology traversal with delta),
  accumulates with Sigma.  Sigma carries disturbance memory across steps.

CONVENTIONAL (shown alongside for structural comparison only):
  tau_cmd = Kp*(theta_target - theta) + Kd*(-omega) + M*g*L*sin(theta)
  No A/B/R operators.  Environment quantities (MASS, ARM, G) required every step.

HONEST NOTES:
  ABR settling angle depends on Sigma accumulation history -- not guaranteed to
  be exactly at declared target.  This is experimental evidence, not tuned output.
  CONV failure here is tuning-specific (Kd braking at disturbance moment).
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# -- Physical constants (plant only -- NOT in ABR command path) ----------------
MASS    = 3.0
ARM     = 0.5
G       = 9.806
I_EFF   = MASS * ARM * ARM
DAMPING = 0.5

# -- Declared motor constants (appear in ABR coupling edges) -------------------
V_BUS        = 48.0
R_W          = 0.7              # declared -- appears in edge I_to_V
K_T          = 1.0              # declared -- appears in edge tau_to_I
TAU_ELEC_MAX = V_BUS * K_T / R_W   # 68.57 N*m -- M extent boundary

# -- Declared raise authority (C projection term) ------------------------------
V_RAISE    = 19.0               # declared raise voltage
TAU_STATIC = MASS * G * ARM     # 14.709 N*m -- declared design load (full horizontal)

# -- Scenario -----------------------------------------------------------------
THETA_TARGET = math.radians(20.0)
THETA_CROSS  = math.radians(10.0)
DIST_MAG     = 25.0

# -- PD gains (CONV only) -----------------------------------------------------
KP = 80.0
KD = 16.0

# -- Simulation ---------------------------------------------------------------
FPS   = 24
DT    = 1.0 / FPS
TOTAL = 6 * FPS


# =============================================================================
# DECLARED COUPLING EDGES (topology -- not operators)
# =============================================================================

class CouplingEdge:
    """One declared relation between two observable nodes."""
    def __init__(self, name, from_node, to_node, fn, constant_name, constant_val):
        self.name          = name
        self.from_node     = from_node
        self.to_node       = to_node
        self.fn            = fn
        self.constant_name = constant_name
        self.constant_val  = constant_val

    def apply(self, input_val):
        return self.fn(input_val)


# Declared topology.  Defines the coupling relations.
# Kernel V8 operators act on this structure -- they are not this structure.
KernelV8Topology = [
    CouplingEdge('tau_to_I', 'Delta_tau', 'Delta_I',
                 fn             = lambda delta_tau: delta_tau / K_T,
                 constant_name  = 'K_T',
                 constant_val   = K_T),
    CouplingEdge('I_to_V', 'Delta_I', 'Delta_V',
                 fn             = lambda delta_I: delta_I * R_W,
                 constant_name  = 'R_W',
                 constant_val   = R_W),
]


# =============================================================================
# KERNEL V8 OPERATORS
# =============================================================================
# A: directed difference at declared observable nodes  (section 11.1)
# B: traverse declared topology with A output           (section 11.1)
# R: antisymmetric operator -- NOT EVALUATED            (rho OPEN, section 11.2)
# Sigma: declared accumulation of B output (separate from R)
# =============================================================================

class KernelV8:
    """
    Kernel V8 relational operators (sections 10, 11).
    Stateful: A carries prior tau_s; Sigma carries accumulated voltage.

    R is not evaluated: rho_base, chi_0 not declared through M (OPEN, section 11.2).
    """

    def __init__(self, tau_s_init=0.0, sigma_init=0.0):
        self._tau_s_prior = tau_s_init   # A operator state (prior observation)
        self._sigma       = sigma_init   # Sigma accumulator state

    # -- A operator: directed difference at persistence edge ------------------
    def A(self, tau_s):
        """
        A operator applied at persistence edge.
        A(x)[e] = x[s] - x[t]  (section 11.1)
        Here: A_out = tau_s(t) - tau_s(t-1)
        Input:  tau_s(t)  -- sensor reading this step (from prior plant output)
        Output: A_out -- directed difference in sensed torque
        State:  updates prior tau_s for next step.
        """
        a_out             = tau_s - self._tau_s_prior
        self._tau_s_prior = tau_s
        return a_out

    # -- B operator: traverse declared topology with A output -----------------
    def B(self, a_out, topology):
        """
        B operator: traverse declared topology with A output.
        B(g)[e] = g[e] + sum_{succ(e)} g[f]  (section 11.1)
        Applied here through declared coupling edges (K_T, R_W).
        Input:  a_out (from A operator)
        Output: (delta_I, delta_V) -- quantities at each topology node
        """
        edge_vals = {'Delta_tau': a_out}
        trace     = []
        current   = a_out
        for edge in topology:
            output = edge.apply(current)
            edge_vals[edge.to_node] = output
            trace.append((edge, current, output))
            current = output
        return edge_vals.get('Delta_I', 0.0), edge_vals.get('Delta_V', 0.0), trace

    # -- R operator: NOT EVALUATED (rho_base, chi_0 not declared through M) --
    # R(g)[e] = g[e] + rho[src(e)] * (sum_{succ(e)} g - sum_{pred(e)} g)
    # rho_base, chi_0 are OPEN conditions (section 11.2) -- M has not declared them.
    # Not-evaluated != evaluated with result 0.0  (section 1 declared-information rule).

    # -- Sigma: declared accumulation of B output (not R output) --------------
    def _accumulate(self, delta_V):
        """
        Declared Sigma accumulation of B output.
        sigma_v(t) = sigma_v(t-1) + delta_V
        Note: R not evaluated (rho OPEN); Sigma accumulates B output directly.
        """
        self._sigma += delta_V
        return self._sigma

    # -- Full step: A -> B -> [R: not evaluated] -> Sigma ---------------------
    def step(self, tau_s, theta, topology):
        """
        Execute Kernel V8: O_t -> M -> A -> B -> [R: rho OPEN] -> Sigma -> C -> u_t
        Returns (sigma_v, a_out, delta_I, delta_V, sigma_v, b_trace, admitted).
        """
        # M admission check
        if abs(tau_s) > TAU_ELEC_MAX:
            return 0.0, 0.0, 0.0, 0.0, 0.0, [], False

        a_out              = self.A(tau_s)                        # A operator
        delta_I, delta_V, b_trace = self.B(a_out, topology)      # B operator
        # R: not evaluated -- rho_base, chi_0 not declared through M (OPEN, s. 11.2)
        sigma_v            = self._accumulate(delta_V)            # Sigma accumulation

        return sigma_v, a_out, delta_I, delta_V, sigma_v, b_trace, True


# =============================================================================
# C: declared projection (section 11.1 -- not a kernel operator)
# =============================================================================

def c_projection(sigma_v, theta):
    """
    C: declared raise/resist projection (symmetric about declared target).
    frac = (target - theta) / target   in [-1, 1]
      frac > 0  (below target):  adds V_RAISE * frac  -- raise drive
      frac = 0  (at target):     v_cmd = Sigma only    -- hold
      frac < 0  (above target):  subtracts V_RAISE * |frac| -- resist overshoot
    Sigma provides load compensation; C provides declared positional authority.
    Neither is environment; neither is a kernel operator.
    """
    frac  = max(-1.0, min(1.0, (THETA_TARGET - theta) / THETA_TARGET))
    v_cmd = sigma_v + frac * V_RAISE
    return max(-V_BUS, min(V_BUS, v_cmd))


# =============================================================================
# CONVENTIONAL (structural comparison)
# =============================================================================

def conv_control(theta, omega):
    tau = KP * (THETA_TARGET - theta) + KD * (-omega) + MASS * G * ARM * math.sin(theta)
    return max(-TAU_ELEC_MAX, min(TAU_ELEC_MAX, tau))


# =============================================================================
# PLANT (shared)
# =============================================================================

def integrate_plant(theta, omega, tau_cmd, tau_disturbance=0.0):
    """
    Returns (new_theta, new_omega, tau_sensed).
    tau_sensed = load-side sensor output (gravity + disturbance, no motor term).
    This is O_t for the NEXT ABR step.
    """
    tau_net   = tau_cmd - MASS * G * ARM * math.sin(theta) - DAMPING * omega - tau_disturbance
    alpha     = tau_net / I_EFF
    new_omega = omega + alpha * DT
    new_theta = theta + new_omega * DT
    clamped   = max(0.0, min(math.pi / 2.0, new_theta))
    if abs(clamped - new_theta) > 1e-12:
        new_omega = 0.0
    tau_sensed = MASS * G * ARM * math.sin(theta) + tau_disturbance
    return clamped, new_omega, tau_sensed


# =============================================================================
# SIMULATION LOOP
# =============================================================================

def run():
    # CONV state
    c_theta, c_omega = 0.0, 0.0

    # ABR state
    a_theta, a_omega = 0.0, 0.0
    a_tau_s = MASS * G * ARM * math.sin(0.0)   # sensor primed from t=0 plant output

    # Kernel V8 instance (carries A and Sigma state)
    kernel = KernelV8(tau_s_init=a_tau_s, sigma_init=0.0)

    dist_active = False
    rows = []

    for f in range(1, TOTAL + 1):
        if not dist_active and (a_theta >= THETA_CROSS or c_theta >= THETA_CROSS):
            dist_active = True
        dist = DIST_MAG if dist_active else 0.0

        # -- CONVENTIONAL --------------------------------------------------
        tau_cmd_c = conv_control(c_theta, c_omega)
        c_theta, c_omega, c_tau_s = integrate_plant(c_theta, c_omega, tau_cmd_c, dist)

        # -- ABR Kernel V8 -------------------------------------------------
        # a_tau_s is O_t: sensor from prior plant step.
        # kernel.step() executes A -> B -> [R: rho OPEN] -> Sigma on that observation.
        tau_s_in = a_tau_s   # capture BEFORE integrate_plant overwrites a_tau_s
        sigma_v, a_out, delta_I, delta_V, _, b_trace, admitted = kernel.step(
            a_tau_s, a_theta, KernelV8Topology)

        # C projection: declared raise authority
        v_cmd_a   = c_projection(sigma_v, a_theta)
        tau_cmd_a = K_T * (v_cmd_a / R_W)

        # Advance plant; capture sensor (O_{t+1}) for next step
        a_theta, a_omega, a_tau_s = integrate_plant(
            a_theta, a_omega, tau_cmd_a, dist)

        rows.append({
            'f':          f,
            't':          f * DT,
            'dist':       dist,
            # CONV
            'c_deg':      math.degrees(c_theta),
            'c_omega':    c_omega,
            'c_tau_cmd':  tau_cmd_c,
            # ABR -- sensor input to A operator this step (O_t)
            'tau_s_in':   tau_s_in,   # absolute sensor reading fed into A
            # ABR -- Kernel V8 operator outputs
            'a_deg':      math.degrees(a_theta),
            'a_omega':    a_omega,
            'a_out':      a_out,       # A operator output (directed difference in tau_s)
            'delta_I':    delta_I,     # B operator: Delta_I
            'delta_V':    delta_V,     # B operator: Delta_V
            'sigma_v':    sigma_v,     # Sigma accumulation (R not evaluated)
            'a_v_cmd':    v_cmd_a,     # C projection output
            'a_tau_cmd':  tau_cmd_a,
            'admitted':   admitted,
            'b_trace':    b_trace,
        })

    return rows, kernel


# =============================================================================
# CONSOLE OUTPUT
# =============================================================================

def print_trace(rows, kernel):
    print()
    print("ABR FLEX DEMO V0.5.2 -- Kernel V8 Operator Execution")
    print("=" * 84)
    print(f"  Flex target: {math.degrees(THETA_TARGET):.0f} deg  |  "
          f"Disturbance: {DIST_MAG:.0f} N*m at ~{math.degrees(THETA_CROSS):.0f} deg")
    print()
    print("  Operator chain (Kernel V8 sections 10, 11.1):")
    print("  O_t -> M -> DeclaredRelations -> A -> B -> [R: rho OPEN] -> Sigma -> C -> u_t")
    print()
    print("  KernelV8Topology (declared structure, not operators):")
    for edge in KernelV8Topology:
        print(f"    Edge {edge.name}: {edge.from_node} --(/ {edge.constant_name}"
              f"={edge.constant_val})--> {edge.to_node}")
    print()
    print("  Kernel V8 operators:")
    print("    A: A_out = tau_s(t) - tau_s(t-1)  [directed difference, section 11.1]")
    print("    B: traverse declared topology with A output  [section 11.1]")
    print("       delta_I = A_out / K_T  (edge tau_to_I)")
    print("       delta_V = delta_I * R_W  (edge I_to_V)")
    print("    R: not evaluated -- rho_base, chi_0 not declared through M")
    print("       [OPEN, section 11.2]  not-evaluated != 0.0  [section 1]")
    print("    Sigma: sigma_v(t) = sigma_v(t-1) + delta_V  [declared accumulation of B]")
    print("    C: v_cmd = sigma_v + frac*V_RAISE  [declared projection, not an operator]")
    print()
    print(f"  NOTE: ABR settling is determined by Sigma accumulation history.")
    print(f"  NOTE: CONV failure is tuning-specific here.")
    print()

    hdr = (f"{'f':>4}  {'t':>5}  {'dist':>5}  "
           f"{'A_out':>9}  {'B(DI)':>7}  {'B(DV)':>7}  "
           f"{'Sigma_v':>9}  {'v_cmd':>7}  "
           f"{'ABR':>6}  {'CONV':>6}  note")
    print(hdr)
    print("-" * 84)

    prev_dist = 0.0
    for r in rows:
        show = r['f'] <= 10 or r['f'] % 6 == 0 or (not prev_dist and r['dist'])
        if show:
            note = ""
            if not prev_dist and r['dist']:
                note = " <- LOAD ON"
            elif r['f'] == 1:
                note = " <- start"
            print(f"{r['f']:>4}  {r['t']:>5.2f}  {r['dist']:>5.1f}  "
                  f"{r['a_out']:>9.4f}  {r['delta_I']:>7.4f}  {r['delta_V']:>7.4f}  "
                  f"{r['sigma_v']:>9.4f}  {r['a_v_cmd']:>7.3f}  "
                  f"{r['a_deg']:>6.2f}  {r['c_deg']:>6.2f}  {note}")
        prev_dist = r['dist']

    # -- Disturbance frame detail --
    # ABR trace: frame with peak |A_out| (first 6 dist frames) -- shows spike
    # CONV analysis: first dist frame -- shows Kd cancellation at load-onset
    dist_rows = [r for r in rows if r['dist'] > 0]
    if dist_rows:
        d     = max(dist_rows[:6], key=lambda r: abs(r['a_out']))  # peak A output frame
        d_1   = rows[d['f'] - 2]                                   # frame before d
        d_on  = dist_rows[0]                                        # first dist frame (CONV)
        d_on1 = rows[d_on['f'] - 2]                                # frame before load-on

        print()
        print("=" * 84)
        print("KERNEL V8 OPERATOR TRACE -- peak disturbance response")
        print("-" * 84)
        print(f"  t={d['t']:.2f}s  (disturbance activated at t={d_on['t']:.2f}s)")
        print()
        print(f"  O_t: tau_s from prior plant step = {d['tau_s_in']:.4f} N*m")
        print(f"       tau_s(t-1) = {d_1['tau_s_in']:.4f} N*m  (prior step sensor)")
        print()
        print(f"  A [operator]:  A_out = tau_s(t) - tau_s(t-1)")
        print(f"                 A_out = {d['a_out']:+.4f} N*m  "
              f"(directed difference -- disturbance now visible to sensor)")
        print()
        print(f"  B [operator]:  traverse KernelV8Topology with A_out = {d['a_out']:.4f}")
        for edge, inp, out in d['b_trace']:
            print(f"    Edge {edge.name}: {edge.from_node}={inp:.4f}"
                  f"  --({edge.constant_name}={edge.constant_val})--> "
                  f"{edge.to_node}={out:.4f}")
        print(f"                 delta_I = {d['delta_I']:.4f} A,  "
              f"delta_V = {d['delta_V']:.4f} V")
        print()
        print(f"  R [operator]:  not evaluated -- rho_base, chi_0 not declared through M")
        print(f"                 [OPEN, section 11.2]")
        print()
        print(f"  Sigma:         sigma_v += {d['delta_V']:.4f}")
        print(f"                 sigma_v = {d_1['sigma_v']:.4f} + {d['delta_V']:.4f}"
              f" = {d['sigma_v']:.4f} V")
        print()
        frac = max(-1.0, min(1.0, (THETA_TARGET - math.radians(d_1['a_deg'])) / THETA_TARGET))
        print(f"  C [projection]: v_cmd = {d['sigma_v']:.4f} + ({frac:+.3f})*{V_RAISE}"
              f" = {d['a_v_cmd']:.3f} V")
        print()
        print(f"  CONV at load-onset (t={d_on['t']:.2f}s)  -- formula, no operators:")
        kp  = KP * (THETA_TARGET - math.radians(d_on1['c_deg']))
        kd  = KD * (-d_on1['c_omega'])
        grv = MASS * G * ARM * math.sin(math.radians(d_on1['c_deg']))
        print(f"    tau_cmd = {kp:+.3f} (Kp*err)  {kd:+.3f} (Kd*(-w))  {grv:+.3f} (grav)")
        print(f"            = {kp+kd+grv:.3f} N*m  <-- Kd*(-omega) nearly cancels Kp*err")

    # -- Outcome --------------------------------------------------------------
    final = rows[-1]
    print()
    print("PASS/FAIL")
    print("-" * 84)
    print("  Question: did the environmental change flow through")
    print("  A (directed difference) -> B (topology traversal) -> Sigma -> C -> actuator")
    print("  -> plant -> sensor?")
    print("  (R not evaluated: rho_base, chi_0 not declared through M -- OPEN s. 11.2)")
    print()
    dist_rows2 = [r for r in rows if r['dist'] > 0]
    dist_r = max(dist_rows2[:6], key=lambda r: abs(r['a_out'])) if dist_rows2 else None
    if dist_r:
        print(f"  Peak A frame:  A_out  = {dist_r['a_out']:+.4f} N*m  "
              f"(directed difference observed by A)")
        print(f"                 B delta_V = {dist_r['delta_V']:+.4f} V  "
              f"(B topology output)")
        print(f"                 Sigma_v = {dist_r['sigma_v']:.4f} V  "
              f"(Sigma state after update)")
        print(f"                 u_t = {dist_r['a_v_cmd']:.3f} V  (actuator commanded)")
        print(f"                 arm at {dist_r['a_deg']:.2f} deg")
        print()
    print(f"  ABR final:  {final['a_deg']:.2f} deg  omega={final['a_omega']:.4f} rad/s")
    print(f"  CONV final: {final['c_deg']:.2f} deg  omega={final['c_omega']:.4f} rad/s")
    print()
    print(f"  Sigma_v at end: {final['sigma_v']:.4f} V  (accumulated across all steps)")
    print()
    print("STRUCTURE SUMMARY")
    print("-" * 84)
    print("  ABR: O_t -> M -> A (directed diff) -> B (topology) -> Sigma -> C -> u_t")
    print("  Topology (K_T, R_W) declares relations.  A, B are operators on topology.")
    print("  R not evaluated: rho_base, chi_0 not declared through M (OPEN, s. 11.2).")
    print("  Runtime environment quantities in operator path: 0")
    print()
    print("  CONV: O_t -> formula(MASS, ARM, G, KP, KD) -> tau_cmd")
    print(f"  Runtime environment quantities: MASS={MASS}, ARM={ARM}, G={G}")


# =============================================================================
# PLOT
# =============================================================================

def plot(rows):
    t       = [r['t']       for r in rows]
    a_deg   = [r['a_deg']   for r in rows]
    c_deg   = [r['c_deg']   for r in rows]
    a_vals  = [r['a_out']   for r in rows]   # A operator output (directed diff)
    sv_vals = [r['sigma_v'] for r in rows]   # Sigma accumulation
    v_cmd   = [r['a_v_cmd'] for r in rows]
    c_tc    = [r['c_tau_cmd'] for r in rows]

    dist_start = next((r['t'] for r in rows if r['dist'] > 0), None)

    BG    = '#0f1117'
    GOLD  = '#e8a020'
    BLUE  = '#4da6ff'
    RED   = '#e05050'
    GREEN = '#50c878'
    CYAN  = '#40d0c0'
    PURPLE= '#c080ff'
    WHITE = '#e8e8e8'

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    fig.patch.set_facecolor(BG)
    for ax in axes:
        ax.set_facecolor(BG)
        ax.tick_params(colors='#aaaaaa', labelsize=8)
        ax.spines[:].set_color('#333333')
        if dist_start:
            ax.axvspan(dist_start, t[-1], alpha=0.07, color=RED)
            ax.axvline(dist_start, color=RED, alpha=0.5, linewidth=1.0, linestyle='--')

    # Panel 1: position
    ax = axes[0]
    ax.axhline(math.degrees(THETA_TARGET), color=WHITE, alpha=0.25,
               linewidth=0.8, linestyle=':')
    ax.plot(t, a_deg, color=GOLD, linewidth=2.2,
            label='ABR  (A->B operators, Sigma, C projection)')
    ax.plot(t, c_deg, color=BLUE, linewidth=2.2, linestyle='--',
            label='CONV  (Kp*err + Kd*(-w) + M*g*L*sin(th))')
    ax.set_ylabel('Joint angle (deg)', color='#aaaaaa', fontsize=9)
    ax.set_ylim(-3, 33)
    ax.legend(loc='upper right', facecolor='#1a1a2e', labelcolor='white',
              fontsize=8, framealpha=0.8)
    ax.set_title(
        f'ABR Kernel V8 -- A/B operators  |  '
        f'{DIST_MAG:.0f} N*m unannounced at ~{math.degrees(THETA_CROSS):.0f} deg  [V0.5.2]',
        color='white', pad=8, fontsize=9)

    # Panel 2: A output and Sigma
    ax = axes[1]
    ax.axhline(0, color='#333333', linewidth=0.8)
    ax.plot(t, a_vals,  color=GREEN,  linewidth=2.0,
            label='A_out  = directed difference in tau_s (N*m)')
    ax.plot(t, sv_vals, color=PURPLE, linewidth=2.0, linestyle='--',
            label='Sigma_v  = accumulated B output (V)')
    ax.set_ylabel('A_out (N*m)  /  Sigma_v (V)', color='#aaaaaa', fontsize=9)
    ax.legend(loc='upper right', facecolor='#1a1a2e', labelcolor='white',
              fontsize=8, framealpha=0.8)
    ax.text(0.01, 0.87,
            'A: directed difference.  B: topology traversal.  R: not evaluated (rho OPEN).',
            transform=ax.transAxes, color='#aaaaaa', fontsize=7.5)

    # Panel 3: v_cmd and CONV torque
    ax = axes[2]
    ax.axhline( V_BUS,  color=RED, alpha=0.30, linewidth=0.8, linestyle=':')
    ax.axhline(-V_BUS,  color=RED, alpha=0.30, linewidth=0.8, linestyle=':')
    ax.axhline(0,       color='#333333', linewidth=0.8)
    ax.plot(t, v_cmd, color=GOLD, linewidth=2.0,
            label='ABR v_cmd  (Sigma_v + C projection)')
    ax.plot(t, c_tc,  color=BLUE, linewidth=2.0, linestyle='--', alpha=0.85,
            label='CONV tau_cmd  (formula)')
    ax.set_ylabel('Voltage / Torque', color='#aaaaaa', fontsize=9)
    ax.set_xlabel('Time (s)',         color='#aaaaaa', fontsize=9)
    ax.legend(loc='upper right', facecolor='#1a1a2e', labelcolor='white',
              fontsize=8, framealpha=0.8)

    fig.text(
        0.02, 0.012,
        'ABR: O_t->M->A[directed diff]->B[K_T,R_W]->Sigma->C->u_t   '
        '[R not evaluated: rho OPEN; 0 env quantities in operator path]\n'
        'CONV: tau=Kp*(target-theta)+Kd*(-omega)+MASS*G*ARM*sin(theta)   '
        f'[MASS={MASS}, ARM={ARM}, G={G}]',
        color='#777777', fontsize=7.5, family='monospace',
    )

    plt.tight_layout(rect=[0, 0.07, 1, 1])
    fname = 'abr_flex_demo.png'
    plt.savefig(fname, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close()
    print(f'\n  Plot saved: {fname}')


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == '__main__':
    rows, kernel = run()
    print_trace(rows, kernel)
    plot(rows)
