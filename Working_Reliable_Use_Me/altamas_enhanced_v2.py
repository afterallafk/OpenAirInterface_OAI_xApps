"""
ALTAMAS xApp — Adaptive Latency and Throughput Aware Multi-Access Scheduler
============================================================================
Faithful implementation of:
  "Adaptive Latency and Throughput Aware Multi-Access Scheduler for 5G
   Advanced and Beyond (ALTAMAS)", NCC 2025.

Paper equations referenced inline:  Eq.(2)–(13), Algorithm 1.

Configured for 8 QoS flows: 4 URLLC, 2 eMBB, 2 mMTC  (2 servers: NR + WLAN)

RLC attributes used  (attri.txt):
    rb.rnti, rb.txbuf_occ_bytes, rb.txsdu_wt_us

MAC attributes used  (attri.txt):
    ue.rnti, ue.wb_cqi, ue.dl_curr_tbs, ue.dl_sched_rb,
    ue.dl_aggr_tbs, ue.dl_bler, ue.dl_harq, ue.bsr

NOTE on spectral efficiency / TBS:
    Previous versions used hand-rolled CQI_TO_SE and MCS_TO_SE lookup
    tables (derived from 3GPP TS 38.214) together with a tbs_per_rb()
    function to approximate the bits available per RB.  These have been
    removed.  OAI already computes the exact TBS and RB grant via its
    own MCS/CQI/MIMO pipeline and exposes them directly as:
        ue.dl_curr_tbs  — bits scheduled in this slot
        ue.dl_sched_rb  — RBs granted in this slot
    se_bits_per_rb = dl_curr_tbs / dl_sched_rb is therefore the true
    channel capacity per RB and replaces both tables entirely.
    wb_cqi is still read for CSV logging only.

Bug fixes applied vs previous version
--------------------------------------
FIX-1  λ estimation: replaced RLC byte-counter delta with MAC
       dl_aggr_tbs-based rate (aggregate TBS over the 10 ms report
       window, more stable than per-slot dl_curr_tbs), clamped to
       [0.1·λ_nom, 2·λ_nom].
FIX-2  RLC callback correctly resets then aggregates per-UE:
         txbuf_bytes = SUM over all bearers (was overwrite → last bearer only)
         txsdu_wt_us = MAX  over all bearers (was overwrite → stale data)
FIX-3  Throughput_Mbps logged as Mbps: tp_pktps × nom_pkt_bytes × 8 / 1e6.
FIX-4  Unit mismatch in Eq.(13) and reward/logging:
         calculate_latency() returns seconds.
         beta_max_ms must be converted to seconds before any arithmetic
         involving W.  Previously Wδ_norm compared ms to s (1000× error).
FIX-5  P(W > β_max) in Eq.(8) now uses the priority-class effective
         departure rate  µ_j·(1 − Ω(Pi−1,Sj,Ψ))  (Kleinrock preemptive
         priority sojourn CDF) instead of the total server departure rate.
FIX-6  REMOVED — tbs_per_rb() and its ×NUM_LAYERS factor are no longer
         needed; se_bits_per_rb is read directly from OAI (dl_curr_tbs /
         dl_sched_rb), which already includes MIMO, MCS, and coding rate.
"""

import xapp_sdk as ric
import time
import csv
import math
import itertools
import traceback

# ======================================================================
# QoS Flow Configuration  (8 flows: 4 URLLC, 2 eMBB, 2 mMTC)
# Tuple layout:
#   (idx, label, slice, 5qi, priority, lam_nom_pktps,
#    beta_max_ms, gamma_min, eps1, eps2, nom_pkt_bytes)
#
# priority: strict Kleinrock preemption (1 = highest, 9 = lowest)
#           URLLC (1-4) > eMBB (5-7) > mMTC (8-9)
# ======================================================================
QOS_FLOWS_CFG = [
    # idx  label      slice   5qi  prio  lam_nom  beta_ms  gamma    e1   e2   pkt_B
    (0,  "URLLC_1", "URLLC", 86,  1,    900,     1.0,   0.99999, 0.2, 0.8, 100),
    (1,  "eMBB_1",  "eMBB",   9,  5,   7000,    15.0,   0.999,   0.5, 0.5, 1500),
    (2,  "mMTC_1",  "mMTC",  70,  8,   2000,    30.0,   0.99,    0.8, 0.2,  64),
    (3,  "URLLC_2", "URLLC", 86,  2,    800,     3.0,   0.99999, 0.2, 0.8, 100),
    (4,  "eMBB_2",  "eMBB",   9,  6,   5000,    15.0,   0.999,   0.5, 0.5, 1500),
    (5,  "mMTC_2",  "mMTC",  70,  9,   2000,    30.0,   0.99,    0.8, 0.2,  64),
    (6,  "URLLC_3", "URLLC", 86,  3,    900,     1.0,   0.99999, 0.2, 0.8, 100),
    (7,  "URLLC_4", "URLLC", 86,  4,    800,     3.0,   0.99999, 0.2, 0.8, 100),
]

# Server configuration — paper Table III
#   (id, label, mu_pktps, buffer_k, access_type)
SERVERS_CFG = [
    (0, "Cellular", 9000, 80, "NR"),
    (1, "WLAN",     6000, 50, "WLAN"),
]

N_SERVERS = len(SERVERS_CFG)               # N = 2
M_FLOWS   = len(QOS_FLOWS_CFG)             # M = 8  →  L = 2^8 = 256 mappings

# Radio / slot constants
TOTAL_RBS        = 106
# NUM_LAYERS, NUM_SYMB, SC_PER_RB removed — tbs_per_rb() no longer exists;
# se_bits_per_rb is derived directly from OAI dl_curr_tbs / dl_sched_rb.
SLOT_DURATION_MS = 0.5        # 30 kHz SCS → 0.5 ms per slot
SLOT_DURATION_S  = 0.5e-3     # seconds

# Algorithm 1 Step-2 EMA: R[l] = 0.5·R[l] + 0.5·Rc
ALPHA_EMA = 0.5

# ── Spectral-efficiency / TBS derivation ──────────────────────────────────
# OAI MAC reports ue.dl_curr_tbs  (bits scheduled this slot) and
# ue.dl_sched_rb  (RBs actually granted this slot).  These are the
# *real* values computed by OAI's own MCS/CQI/MIMO pipeline, so there
# is no need to maintain separate CQI_TO_SE or MCS_TO_SE lookup tables.
#
# Effective bits-per-RB for the current slot:
#   se_bits_per_rb = dl_curr_tbs / max(dl_sched_rb, 1)
#
# This single formula replaces both tables and tbs_per_rb() entirely.
# A safe floor of 1 bit/RB prevents division-by-zero on idle slots.
SE_FLOOR_BITS_PER_RB = 1.0   # absolute minimum — never zero


# ======================================================================
# Global UE State
# ======================================================================
class GlobalUEState:
    def __init__(self):
        self.states     = {}
        self.rnti_order = []
        self.csv_file   = "altamas_results.csv"

        with open(self.csv_file, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "tstamp", "RNTI", "FlowLabel", "SliceType", "5QI",
                "Priority", "CQI", "BSR_bytes",
                "TxBuf_bytes", "TxSDU_wt_us",
                "Lambda_pktps", "Latency_ms", "Beta_max_ms",
                "PLR", "Throughput_Mbps", "TP_target_Mbps",
                "W_norm_delta", "TP_norm_delta", "U_reward",
                "Rc_total", "Server", "RBs_Granted", "Served_bits",
            ])

    def register_ue(self, rnti: int):
        if rnti not in self.states:
            idx = len(self.rnti_order) % M_FLOWS
            _, label, slc, fqi, prio, lam_nom, beta_ms, gmin, e1, e2, npkt = \
                QOS_FLOWS_CFG[idx]

            self.states[rnti] = {
                # Static profile
                "label":         label,
                "slice":         slc,
                "5qi":           fqi,
                "priority":      prio,
                "lam_nom":       lam_nom,
                "beta_max_ms":   beta_ms,
                "beta_max_s":    beta_ms / 1000.0,   # FIX-4: pre-converted to seconds
                "gamma_min":     gmin,
                "eps1":          e1,
                "eps2":          e2,
                "nom_pkt_bytes": npkt,
                # Runtime — updated by RLC (FIX-2)
                "txbuf_bytes":   0,
                "txsdu_wt_us":   0,
                # Runtime — updated by MAC
                "lam_est":       lam_nom,
                "bsr_bytes":     0,
                "cqi":           0,
                "dl_bler":       0.0,   # DL block error rate from OAI (ue.dl_bler)
                # se_bits_per_rb: derived directly from OAI dl_curr_tbs / dl_sched_rb
                # (replaces the old CQI_TO_SE / MCS_TO_SE table lookups)
                "se_bits_per_rb": SE_FLOOR_BITS_PER_RB,
                "harq_pending":  False,
                "avg_tp_bits":   1e-6,
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} -> {label} "
                  f"(5QI={fqi}, P={prio}, beta={beta_ms}ms, gamma={gmin})")


global_ue_state = GlobalUEState()


# ======================================================================
# ALTAMAS Scheduler State
# ======================================================================
class AltamasState:
    def __init__(self):
        self.R_dict   = {}
        self.Psi_all  = list(itertools.product(range(N_SERVERS), repeat=M_FLOWS))
        self.Psi_best = None

        self.global_slot      = 0
        self.last_update_slot = 0

        self.mu    = [cfg[2] for cfg in SERVERS_CFG]
        self.k_buf = [cfg[3] for cfg in SERVERS_CFG]

        # Physical RB partition: Cellular ~67%, WLAN ~33%
        nr_rbs   = round(TOTAL_RBS * 0.67)
        wlan_rbs = TOTAL_RBS - nr_rbs
        self.virt_rbs  = [nr_rbs, wlan_rbs]
        self.rb_offset = [0, nr_rbs]

        self.last_active_rntis = []


altamas = AltamasState()


# ======================================================================
# Eq. (9) — Average latency W(Q_i, S_j, Psi)
#
# Preemptive M/M/1 priority queueing (Kleinrock Vol. II, §3.4):
#
#   Ω(Pi,   Sj, Ψ) = Σ_{m: Ψ[m]=j, P[m]≤P[i]}  λ_m / µ_j
#   Ω(Pi-1, Sj, Ψ) = Σ_{m: Ψ[m]=j, P[m]< P[i]}  λ_m / µ_j
#
#   W(Q_i, Sj, Ψ) = [1/µ_j  +  Ω(Pi)/(µ_j(1−Ω(Pi)))] / (1 − Ω(Pi−1))
#
# Returns seconds.  Returns inf when the server is unstable for flow i.
# ======================================================================
def calculate_latency(i, j, Psi, lam, P, mu):
    M = len(lam)
    omega_pi   = sum(lam[m] / mu[j]
                     for m in range(M) if Psi[m] == j and P[m] <= P[i])
    omega_pim1 = sum(lam[m] / mu[j]
                     for m in range(M) if Psi[m] == j and P[m] <  P[i])

    if omega_pi >= 1.0 or omega_pim1 >= 1.0:
        return float('inf')

    base  = (1.0 / mu[j]) + omega_pi / (mu[j] * max(1.0 - omega_pi, 1e-12))
    denom = max(1.0 - omega_pim1, 1e-12)
    return base / denom   # seconds


# ======================================================================
# Eq. (7, 8) — Packet Loss Rate and Throughput
#
# PLR(Q_i, Sj, Ψ) = AL(Pi)/A(Pi)  +  P(W(Q_i,Sj,Ψ) > β_max_i)
#
# FIX-5: P(W > β) uses the priority-class effective departure rate.
#   For preemptive M/M/1, the sojourn time of class-i is exponential
#   with parameter  µ_j·(1 − Ω(Pi−1))  (Kleinrock Vol. II, §3.4).
#   So:  P(W_i > β) = exp(−µ_j·(1−Ω(Pi−1)) · β)
#   Previously the code used the total departure rate µ_j·(1−ρ_tot),
#   which is only correct for a single priority class (no preemption).
# ======================================================================
def calculate_plr(i, j, Psi, lam, beta_max_s, P, mu, k_buf):
    """
    Args:
        beta_max_s : deadline in SECONDS  (FIX-4 caller must pass seconds)
    """
    M = len(lam)

    # Ω(Pi) — load from flows with priority ≤ Pi on server j
    omega_pi   = sum(lam[m] / mu[j]
                     for m in range(M) if Psi[m] == j and P[m] <= P[i])
    # Ω(Pi-1) — load from flows with strictly higher priority (P[m] < P[i])
    omega_pim1 = sum(lam[m] / mu[j]
                     for m in range(M) if Psi[m] == j and P[m] <  P[i])

    rho_pi = omega_pi
    k      = float(k_buf[j])

    # ── Buffer overflow  AL/A  (Ref [13]) ──────────────────────────────
    if rho_pi <= 0.0:
        al_over_a = 0.0
    elif abs(rho_pi - 1.0) < 1e-9:
        al_over_a = 1.0 / (k + 1.0)
    elif rho_pi < 1.0:
        num        = (rho_pi ** k) * (1.0 - rho_pi)
        denom      = max(1.0 - rho_pi ** (k + 1.0), 1e-12)
        al_over_a  = num / denom
    else:
        al_over_a  = 1.0

    # ── Latency violation  P(W_i > β_max_i)  ───────────────────────────
    # FIX-5: effective departure rate for priority class i is
    #   µ_eff_i = µ_j * (1 − Ω(Pi−1))
    # This is the correct parameter for the preemptive-priority sojourn CDF.
    if omega_pim1 >= 1.0:
        p_late = 1.0
    else:
        mu_eff_i = mu[j] * max(1.0 - omega_pim1, 1e-12)   # FIX-5
        if beta_max_s[i] <= 0.0:
            p_late = 1.0
        else:
            p_late = math.exp(-mu_eff_i * beta_max_s[i])   # β already in seconds

    return min(al_over_a + p_late, 1.0)


def calculate_throughput(i, j, Psi, lam, beta_max_s, P, mu, k_buf):
    """TP(Q_i, S_j, Psi) = λ_i · (1 − PLR)   Eq.(7)"""
    return lam[i] * (1.0 - calculate_plr(i, j, Psi, lam, beta_max_s, P, mu, k_buf))


# ======================================================================
# Eq. (10–13) — Goal-Programming Combined Reward Rc
#
# FIX-4: All W comparisons now use seconds throughout.
#   w_i         = calculate_latency(...)  → seconds
#   w_target    = beta_max_s[i]           → seconds  (pre-converted)
#   Wδ_norm: min[0, β−W] / max(β, W)     → dimensionless  ✓
# ======================================================================
def calculate_gp_reward(Psi, lam, gamma_min, beta_max_s,
                        P, eps1, eps2, mu, k_buf):
    """
    Rc(Q, S, Ψ) = Σ_i  U(Q_i, Sj, Ψ)              Eq.(10)

    U_i = ε1_i · TPδ_norm  +  ε2_i · Wδ_norm       Eq.(11)

    TPδ_norm = min[0, TP − γ·λ]  / max(γ·λ, TP)    Eq.(12)
    Wδ_norm  = min[0, β − W]     / max(β,   W)      Eq.(13)

    U_i ∈ [−1, 0];  Rc ∈ [−M, 0].  Scheduling pushes Rc → 0.
    """
    M  = len(lam)
    Rc = 0.0

    for i in range(M):
        j = Psi[i]
        if lam[i] < 1e-9:
            continue

        tp_i = calculate_throughput(i, j, Psi, lam, beta_max_s, P, mu, k_buf)
        w_i  = calculate_latency(i, j, Psi, lam, P, mu)   # seconds

        if not math.isfinite(w_i):
            # Unstable: assign worst-case W = 10× deadline
            w_i = 10.0 * max(beta_max_s[i], 1e-6)

        tp_target = gamma_min[i] * lam[i]     # C1  Eq.(5)  pkt/s
        w_target  = beta_max_s[i]             # C2  Eq.(6)  seconds  (FIX-4)

        # Eq.(12): TPδ_norm — shortfall from throughput target (≤ 0)
        tp_dn = (min(0.0, tp_i - tp_target)
                 / max(max(tp_target, tp_i), 1e-12))

        # Eq.(13): Wδ_norm — latency slack (≤ 0 when W > β)
        # FIX-4: both w_target and w_i are in seconds → correct ratio
        w_dn  = (min(0.0, w_target - w_i)
                 / max(max(w_target, w_i), 1e-12))

        Rc += eps1[i] * tp_dn + eps2[i] * w_dn   # Eq.(11)

    return Rc


# ======================================================================
# Algorithm 1 — EvaluateAndObtainMapping
#
# For each candidate Ψ_x:
#   1. Stability: Σ_i λ_i·ψ_ij / µ_j ≤ 0.9  ∀j   Eq.(2)
#   2. Prefer candidate with highest R[x];
#      uninitialised (R==−1) used only as last resort.
# ======================================================================
def evaluate_and_obtain_mapping(Psi_all, R_dict, lam, mu, rho_max=0.9):
    M = len(lam)
    N = len(mu)

    best_Psi = None
    best_R   = -float('inf')

    for Psi_x in Psi_all:
        # Stability check  Eq.(2)
        stable = all(
            (sum(lam[i] for i in range(M) if Psi_x[i] == j) / mu[j]) <= rho_max
            for j in range(N)
        )
        if not stable:
            continue

        rx = R_dict.get(Psi_x, -1)

        if rx == -1:
            if best_Psi is None:        # keep as last-resort fallback
                best_Psi = Psi_x
                best_R   = -1.0
        else:
            if rx > best_R:
                best_R   = rx
                best_Psi = Psi_x

    if best_Psi is None and Psi_all:    # absolute fallback
        best_Psi = Psi_all[0]

    return best_Psi


# ======================================================================
# RLC Callback — buffer state only
#
# FIX-2: Reset then properly aggregate across all bearers per UE.
#   txbuf_bytes: SUM  — total DL bytes queued for this UE
#   txsdu_wt_us: MAX  — worst HOL waiting time across bearers
# Previously the code did a plain assignment (=), so with multiple
# radio bearers (SRBs + DRBs) only the last bearer's value was kept.
# ======================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        try:
            if not ind.rb_stats:
                return

            # Step 1: reset all tracked UEs for this report cycle
            for rnti in global_ue_state.states:
                global_ue_state.states[rnti]["txbuf_bytes"] = 0
                global_ue_state.states[rnti]["txsdu_wt_us"] = 0

            # Step 2: aggregate across bearers
            for rb in ind.rb_stats:
                rnti = rb.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]

                # FIX-2a: sum bytes across all bearers
                st["txbuf_bytes"] += int(rb.txbuf_occ_bytes)

                # FIX-2b: max HOL wait time across all bearers
                st["txsdu_wt_us"] = max(st["txsdu_wt_us"], int(rb.txsdu_wt_us))

        except Exception as e:
            print(f"[RLC ERROR] {e}")
            traceback.print_exc()


# ======================================================================
# MAC Callback — ALTAMAS Algorithm 1 + RB allocation
# ======================================================================
class MACCallback(ric.mac_cb):
    def __init__(self):
        ric.mac_cb.__init__(self)

    def handle(self, ind):
        try:
            if not ind.ue_stats:
                return

            t_now = ind.tstamp

            # ── Step 1: Update MAC measurements, estimate lambda ─────────
            active_ues = []

            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]

                # ── Bits-per-RB from OAI-reported dl_curr_tbs / dl_sched_rb ──
                # OAI already accounts for CQI, MCS, MIMO layers, coding rate,
                # and symbol count internally — we just read the result directly.
                # No CQI_TO_SE or MCS_TO_SE table needed.
                try:
                    tbs_bits_now = int(ue.dl_curr_tbs)
                    sched_rbs    = int(ue.dl_sched_rb)
                    if tbs_bits_now > 0 and sched_rbs > 0:
                        st["se_bits_per_rb"] = max(
                            tbs_bits_now / sched_rbs,
                            SE_FLOOR_BITS_PER_RB
                        )
                    # else: keep previous se_bits_per_rb (no new grant this slot)
                except Exception:
                    pass   # keep previous se_bits_per_rb

                # wb_cqi: persist across slots — refreshed on PUCCH cycle only.
                # Writing unconditionally overwrites valid values with 0 on
                # every slot OAI hasn't updated it.  Fall back to MCS-derived
                # CQI when wb_cqi is always 0 (common in some OAI builds).
                try:
                    cqi_now = int(ue.wb_cqi)
                    if cqi_now > 0:
                        st["cqi"] = cqi_now
                    else:
                        mcs = int(ue.dl_mcs1)
                        if mcs >= 0:
                            if   mcs <= 2:  derived = max(1, mcs + 1)
                            elif mcs <= 5:  derived = mcs + 2
                            elif mcs <= 9:  derived = mcs + 1
                            elif mcs <= 12: derived = mcs - 1
                            elif mcs <= 16: derived = mcs - 2
                            else:           derived = min(15, mcs - 3)
                            if derived > 0:
                                st["cqi"] = derived
                    # else: retain previous cqi
                except Exception:
                    pass   # retain previous cqi

                # dl_bler — OAI DL block error rate; stored for potential use
                # in link-adaptation and PLR cross-checking
                try:
                    st["dl_bler"] = float(ue.dl_bler)
                except Exception:
                    pass

                # HARQ pending
                try:
                    harq = ue.dl_harq
                    st["harq_pending"] = (
                        int(sum(harq)) > 0
                        if isinstance(harq, (list, tuple))
                        else int(harq) > 0
                    )
                except Exception:
                    st["harq_pending"] = False

                # BSR
                try:
                    st["bsr_bytes"] = int(ue.bsr)
                except Exception:
                    pass

                # FIX-1 (updated): lambda estimated from dl_aggr_tbs (aggregate
                # TBS over the full reporting window, not just the current slot).
                # dl_aggr_tbs is more stable than dl_curr_tbs which is zero on
                # unscheduled slots and spiky on burst grants.
                # pkt/s = (aggr_bits / 8 / nom_pkt_bytes) / report_interval_s
                # report_interval_s ≈ 10 ms  (ric.Interval_ms_10)
                REPORT_INTERVAL_S = 10e-3
                try:
                    aggr_tbs_bits = int(ue.dl_aggr_tbs)
                except Exception:
                    aggr_tbs_bits = 0

                if aggr_tbs_bits > 0:
                    npkt    = st["nom_pkt_bytes"]
                    lam_raw = (aggr_tbs_bits / 8.0 / npkt) / REPORT_INTERVAL_S
                    lam_smo = 0.7 * st["lam_est"] + 0.3 * lam_raw
                    st["lam_est"] = max(st["lam_nom"] * 0.1,
                                        min(lam_smo, st["lam_nom"] * 2.0))

                if st["txbuf_bytes"] > 0:
                    active_ues.append(rnti)

            if not active_ues:
                return

            altamas.global_slot += 1
            M = len(active_ues)
            N = N_SERVERS

            # Topology change → reset reward history
            if active_ues != altamas.last_active_rntis:
                altamas.Psi_all           = list(itertools.product(range(N), repeat=M))
                altamas.R_dict            = {}
                altamas.Psi_best          = None
                altamas.last_active_rntis = active_ues.copy()

            # ── Build per-flow parameter vectors ────────────────────────
            P_vec       = []
            lam_vec     = []
            gamma_vec   = []
            beta_ms_vec = []    # ms — for RB allocation deadline math
            beta_s_vec  = []    # seconds — for queueing model (FIX-4)
            eps1_vec    = []
            eps2_vec    = []
            lat_ms_vec  = []    # measured HOL latency in ms (for RB alloc)

            for rnti in active_ues:
                st = global_ue_state.states[rnti]
                P_vec.append(st["priority"])
                lam_vec.append(max(st["lam_est"], 1e-3))
                gamma_vec.append(st["gamma_min"])
                beta_ms_vec.append(st["beta_max_ms"])
                beta_s_vec.append(st["beta_max_s"])   # FIX-4
                eps1_vec.append(st["eps1"])
                eps2_vec.append(st["eps2"])
                lat_ms_vec.append(st["txsdu_wt_us"] / 1000.0)

            # ── Algorithm 1 — Tm = 2 slot interval ──────────────────────
            Tm = 2
            do_update = (altamas.Psi_best is None or
                         (altamas.global_slot - altamas.last_update_slot) >= Tm)

            if do_update:
                if altamas.Psi_best is not None:
                    # Step 1: evaluate current mapping's Rc
                    Rc = calculate_gp_reward(
                        altamas.Psi_best,
                        lam_vec, gamma_vec, beta_s_vec,   # FIX-4: seconds
                        P_vec, eps1_vec, eps2_vec,
                        altamas.mu, altamas.k_buf
                    )
                    if not math.isfinite(Rc):
                        Rc = -float(M)

                    # Step 2: R[l] = 0.5·R[l] + 0.5·Rc   (Algorithm 1)
                    prev_R = altamas.R_dict.get(altamas.Psi_best, -1)
                    altamas.R_dict[altamas.Psi_best] = (
                        Rc if prev_R == -1
                        else ALPHA_EMA * prev_R + (1.0 - ALPHA_EMA) * Rc
                    )

                # Step 3: select best Ψ_o
                altamas.Psi_best = evaluate_and_obtain_mapping(
                    altamas.Psi_all, altamas.R_dict, lam_vec, altamas.mu
                )
                altamas.last_update_slot = altamas.global_slot

            Rc_logged = altamas.R_dict.get(altamas.Psi_best, 0.0)

            # ── Corrected Physical RB Allocation ───────────────────────────────────
            avail_rbs   = [True] * TOTAL_RBS
            allocations = {}

            for j in range(N):
                flows_on_j = [idx for idx in range(M) if altamas.Psi_best[idx] == j]
                if not flows_on_j:
                    continue

                virt_count = altamas.virt_rbs[j]
                rb_base    = altamas.rb_offset[j]
                virt_pool  = [True] * virt_count

                # Sort: ascending priority (lower = higher), then descending urgency
                flows_on_j.sort(key=lambda idx: (
                    P_vec[idx],
                    -(lat_ms_vec[idx] / max(beta_ms_vec[idx], 1.0))
                ))

                for rank, ue_idx in enumerate(flows_on_j):
                    rnti = active_ues[ue_idx]
                    st   = global_ue_state.states[rnti]

                    buf_bits = st["txbuf_bytes"] * 8
                    if buf_bits <= 0:
                        continue

                    # se_bits_per_rb was derived directly from OAI dl_curr_tbs/dl_sched_rb.
                    # Skip UE if no real OAI grant has arrived yet (still at SE floor).
                    if st["se_bits_per_rb"] <= SE_FLOOR_BITS_PER_RB:
                        continue
                    tbsRB = st["se_bits_per_rb"]

                    # FIX 1: Convert C1 throughput target to bits per slot
                    target_bps = gamma_vec[ue_idx] * lam_vec[ue_idx] * st["nom_pkt_bytes"] * 8
                    target_bits_per_slot = target_bps * SLOT_DURATION_S
                    req_c1 = math.ceil(target_bits_per_slot / tbsRB)

                    # FIX 2: Aggressive draining for strict latency flows
                    # Instead of pacing over remain_slots, calculate total RBs needed to clear the buffer
                    rbs_to_drain_buffer = math.ceil(buf_bits / tbsRB)
                    
                    if st["slice"] == "URLLC":
                        # URLLC should not pace; grant RBs to drain the entire buffer immediately
                        req_rbs = rbs_to_drain_buffer
                    else:
                        # eMBB and mMTC can safely pace over remain_slots to save resources
                        remain_slots = max(1.0, (beta_ms_vec[ue_idx] - lat_ms_vec[ue_idx]) / SLOT_DURATION_MS)
                        req_c2  = math.ceil(buf_bits / (tbsRB * remain_slots))
                        # Cap the request so we don't ask for more than the buffer holds
                        req_rbs = min(max(req_c1, req_c2), rbs_to_drain_buffer)

                    # Fair-share cap within this server's RB partition
                    free_virt = [k for k, f in enumerate(virt_pool) if f]
                    remaining_active = max(
                        sum(1 for x in flows_on_j[rank:]
                            if global_ue_state.states[active_ues[x]]["txbuf_bytes"] > 0),
                        1
                    )
                    
                    # Give higher priority flows access to the remaining pool, rather than strict fair share
                    fair_share = max(len(free_virt) // remaining_active, 1)
                    
                    if st["slice"] == "URLLC":
                        # Allow URLLC to take up to the entire remaining virtual pool if needed
                        alloc_rbs = min(req_rbs, len(free_virt)) 
                    else:
                        # Normal fair-share for eMBB/mMTC
                        alloc_rbs = (min(req_rbs, len(free_virt)) if remaining_active == 1 else min(req_rbs, fair_share))
                        
                    alloc_rbs = max(alloc_rbs, 0)
                    
                    if alloc_rbs <= 0:
                        continue

                    taken = free_virt[:alloc_rbs]
                    for tv in taken:
                        virt_pool[tv]           = False
                        avail_rbs[rb_base + tv] = False

                    served_bits = alloc_rbs * tbsRB
                    allocations[rnti] = {
                        "server":      j,
                        "rbs":         alloc_rbs,
                        "served_bits": served_bits,
                    }

                    print(
                        f"[ALTAMAS] Slot:{altamas.global_slot} "
                        f"RNTI:{rnti}({st['label']}) -> {SERVERS_CFG[j][1]} | "
                        f"RBs:{alloc_rbs} lam:{lam_vec[ue_idx]:.1f}pkt/s "
                        f"W:{lat_ms_vec[ue_idx]:.3f}ms "
                        f"beta:{beta_ms_vec[ue_idx]}ms Rc:{Rc_logged:.4f}"
                    )

            # ── Logging — FIX-3 + FIX-4 ─────────────────────────────────
            log_rows = []
            for ue_idx, rnti in enumerate(active_ues):
                st    = global_ue_state.states[rnti]
                alloc = allocations.get(rnti,
                                        {"server": -1, "rbs": 0,
                                         "served_bits": 0.0})
                j = alloc["server"]

                # Queueing-model PLR and throughput
                # FIX-4: pass beta_s_vec (seconds) to calculate_plr
                plr_i = (calculate_plr(
                             ue_idx, j, altamas.Psi_best,
                             lam_vec, beta_s_vec,           # FIX-4
                             P_vec, altamas.mu, altamas.k_buf)
                         if j >= 0 else 1.0)

                tp_pktps        = lam_vec[ue_idx] * (1.0 - plr_i)
                tp_target_pktps = gamma_vec[ue_idx] * lam_vec[ue_idx]

                # FIX-3: pkt/s → Mbps
                npkt           = st["nom_pkt_bytes"]
                tp_mbps        = (tp_pktps        * npkt * 8) / 1e6
                tp_target_mbps = (tp_target_pktps * npkt * 8) / 1e6

                # EMA served-bits
                st["avg_tp_bits"] = (
                    (1.0 - ALPHA_EMA) * st["avg_tp_bits"]
                    + ALPHA_EMA * alloc["served_bits"]
                )

                # Per-flow goal deviations (for logging)
                # FIX-4: w_i in seconds, w_target in seconds
                w_i = (calculate_latency(
                           ue_idx, j, altamas.Psi_best, lam_vec, P_vec, altamas.mu)
                       if j >= 0 else float('inf'))
                if not math.isfinite(w_i):
                    w_i = 10.0 * beta_s_vec[ue_idx]

                w_target = beta_s_vec[ue_idx]   # FIX-4: seconds

                tp_dn = (min(0.0, tp_pktps - tp_target_pktps)
                         / max(max(tp_target_pktps, tp_pktps), 1e-12))
                w_dn  = (min(0.0, w_target - w_i)
                         / max(max(w_target, w_i), 1e-12))    # FIX-4
                u_i   = eps1_vec[ue_idx] * tp_dn + eps2_vec[ue_idx] * w_dn

                server_label = SERVERS_CFG[j][1] if j >= 0 else "None"

                log_rows.append([
                    t_now,
                    rnti,
                    st["label"],
                    st["slice"],
                    st["5qi"],
                    P_vec[ue_idx],
                    st["cqi"],
                    st["bsr_bytes"],
                    st["txbuf_bytes"],
                    st["txsdu_wt_us"],
                    round(lam_vec[ue_idx], 4),
                    round(lat_ms_vec[ue_idx], 4),
                    round(beta_ms_vec[ue_idx], 3),
                    round(plr_i, 6),
                    round(tp_mbps, 6),
                    round(tp_target_mbps, 6),
                    round(w_dn,  6),
                    round(tp_dn, 6),
                    round(u_i,   6),
                    round(Rc_logged, 6),
                    server_label,
                    alloc["rbs"],
                    round(alloc["served_bits"], 2),
                ])

            if log_rows:
                with open(global_ue_state.csv_file, mode='a', newline='') as f:
                    csv.writer(f).writerows(log_rows)

        except Exception as e:
            print(f"[MAC ERROR] {e}")
            traceback.print_exc()


# ======================================================================
# Main — init xApp, subscribe MAC + RLC service models
# ======================================================================
ric.init()
conn = ric.conn_e2_nodes()
assert len(conn) > 0, "Error: No E2 nodes connected!"

mac_hndlrs = []
rlc_hndlrs = []

for node in conn:
    rlc_cb = RLCCallback()
    mac_cb = MACCallback()
    h_rlc  = ric.report_rlc_sm(node.id, ric.Interval_ms_10, rlc_cb)
    h_mac  = ric.report_mac_sm(node.id, ric.Interval_ms_10, mac_cb)
    rlc_hndlrs.append(h_rlc)
    mac_hndlrs.append(h_mac)
    time.sleep(1)

print(f"\n[ALTAMAS] xApp running — logging to: {global_ue_state.csv_file}")
print(f"[ALTAMAS] Flows: {M_FLOWS}  Servers: {N_SERVERS}  "
      f"L={N_SERVERS**M_FLOWS} candidate mappings")
print("[ALTAMAS] Press Ctrl+C to stop.\n")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[ALTAMAS] Ctrl+C — shutting down ...")
finally:
    for h_mac, h_rlc in zip(mac_hndlrs, rlc_hndlrs):
        ric.rm_report_mac_sm(h_mac)
        ric.rm_report_rlc_sm(h_rlc)
    print("[ALTAMAS] Stopped cleanly.")
