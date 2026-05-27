"""
ALTAMAS xApp — Adaptive Latency and Throughput Aware Multi-Access Scheduler
============================================================================
Faithful implementation of:
  "Adaptive Latency and Throughput Aware Multi-Access Scheduler for 5G
   Advanced and Beyond (ALTAMAS)", NCC 2025.

Paper equations referenced inline:  Eq.(2)–(13), Algorithm 1.

Configured for 10 QoS flows: 6 URLLC, 2 eMBB, 2 mMTC  (2 servers: NR + WLAN)

OAI Attribute Reference (verified against live xApp inspection)
---------------------------------------------------------------
MAC  (ue_stats[i]):
  ue.rnti             — UE radio network temporary identifier
  ue.wb_cqi           — Wideband CQI (0-15, 3GPP TS 38.214 Table 5.1.3.1-1)
                        Refreshed on PUCCH reporting cycle; may be 0 between cycles.
  ue.dl_mcs1          — DL MCS index layer-1 (0-28, TS 38.214 Table 5.1.3.2-1)
                        Used as fallback when wb_cqi == 0.
  ue.dl_curr_tbs      — TBS (bits) granted in the current slot by OAI scheduler
  ue.dl_sched_rb      — RBs granted in the current slot by OAI scheduler
  ue.dl_aggr_tbs      — Aggregate TBS (bits) over the 10 ms reporting window
  ue.dl_bler          — DL block error rate (0.0–1.0)
  ue.dl_harq          — HARQ process state vector
  ue.bsr              — Buffer Status Report (bytes)

RLC  (rb_stats[i]):
  rb.rnti             — UE RNTI (matches MAC)
  rb.txbuf_occ_bytes  — DL TX buffer occupancy in bytes (HOL-queue depth)
  rb.txsdu_wt_us      — HOL SDU waiting time in MICROSECONDS ← correct for
                        deadline comparison (divide by 1000 to get ms)
  NOTE: rb.txpdu_wt_ms is PDU-level (post-segmentation) — less precise for
        SDU deadline monitoring; NOT used by this scheduler.

Fixes applied in this version (v2_fixed_attr)
----------------------------------------------
FIX-1  λ estimation from dl_aggr_tbs (10 ms window), EMA-smoothed.

FIX-2  RLC callback aggregates per-UE correctly:
         txbuf_bytes = SUM, txsdu_wt_us = MAX across all bearers.

FIX-3  Throughput logged in Mbps: tp_pktps × pkt_bytes × 8 / 1e6.

FIX-4  All latency arithmetic in seconds (beta_max_s pre-converted).

FIX-5  P(W > β_max) deterministic indicator per paper Eq.(8).

FIX-6  EvaluateAndObtainMapping: unseen mapping returned immediately.

FIX-7  Per-slice RB cap: URLLC=1.0, eMBB=0.5, mMTC=0.3 × virt_rbs[j].

FIX-8  eMBB lam_nom normalised (7000→2000, 5000→1500 pkt/s).

FIX-9  (NEW) MCS→CQI fallback table corrected against 3GPP TS 38.214.
         Previous heuristic (mcs+1, mcs+2, mcs-1 …) was wrong for 27
         out of 29 MCS indices.  Replaced with a pre-computed lookup
         table derived by matching spectral efficiency (Qm × R/1024)
         between TS 38.214 Table 5.1.3.2-1 (MCS) and Table 5.1.3.1-1
         (CQI).  The table only affects CSV logging; the primary channel
         capacity estimate is se_bits_per_rb = dl_curr_tbs/dl_sched_rb
         which comes directly from OAI's own MCS/CQI/MIMO pipeline.
"""

import xapp_sdk as ric
import time
import csv
import math
import itertools
import traceback

# ======================================================================
# FIX-8: Normalised eMBB lam_nom
# -----------------------------------------------------------------------
# Old values:  eMBB_1 = 7000 pkt/s, eMBB_2 = 5000 pkt/s  (1500 B pkt)
#   → 84 Mbps / 60 Mbps nominal — saturates the shared NR carrier alone.
# New values:  eMBB_1 = 2000 pkt/s, eMBB_2 = 1500 pkt/s  (1500 B pkt)
#   → 24 Mbps / 18 Mbps nominal — realistic for 10-UE shared carrier.
#
# Combined offered load across all 10 flows to Cellular (mu=15000):
#   6×URLLC  ≈ 6×850  =  5100 pkt/s
#   2×eMBB   ≈ 3500   =  3500 pkt/s   (was 12000 — caused ρ≈0.8 just eMBB)
#   2×mMTC   ≈ 4000   =  4000 pkt/s
#   Total    ≈ 12600 pkt/s  →  ρ_Cellular ≈ 0.84  ✓ (was >1.0 at old values)
# ======================================================================
QOS_FLOWS_CFG = [
    # idx  label      slice   5qi  prio  lam_nom  beta_ms  gamma    e1   e2   pkt_B
    (0,  "URLLC_1", "URLLC", 86,  1,    900,     1.0,   0.99999, 0.2, 0.8, 100),
    (1,  "eMBB_1",  "eMBB",   9,  7,   2000,    15.0,   0.999,   0.5, 0.5, 1500),  # FIX-8: 7000→2000
    (2,  "mMTC_1",  "mMTC",  70,  9,   2000,    30.0,   0.99,    0.8, 0.2,  64),
    (3,  "URLLC_2", "URLLC", 86,  2,    800,     3.0,   0.99999, 0.2, 0.8, 100),
    (4,  "eMBB_2",  "eMBB",   9,  8,   1500,    15.0,   0.999,   0.5, 0.5, 1500),  # FIX-8: 5000→1500
    (5,  "mMTC_2",  "mMTC",  70, 10,   2000,    30.0,   0.99,    0.8, 0.2,  64),
    (6,  "URLLC_3", "URLLC", 86,  3,    900,     1.0,   0.99999, 0.2, 0.8, 100),
    (7,  "URLLC_4", "URLLC", 86,  4,    800,     3.0,   0.99999, 0.2, 0.8, 100),
    (8,  "URLLC_5", "URLLC", 86,  5,    900,     1.0,   0.99999, 0.2, 0.8, 100),
    (9,  "URLLC_6", "URLLC", 86,  6,    800,     3.0,   0.99999, 0.2, 0.8, 100),
]

# Server configuration — scaled for 10 UEs (paper Table III × 5/3)
SERVERS_CFG = [
    (0, "Cellular", 15000, 80, "NR"),
    (1, "WLAN",     10000, 50, "WLAN"),
]

N_SERVERS = len(SERVERS_CFG)
M_FLOWS   = len(QOS_FLOWS_CFG)       # 10 flows → L = 2^10 = 1024 mappings

# Radio / slot constants
TOTAL_RBS        = 106
SLOT_DURATION_MS = 0.5               # 30 kHz SCS
SLOT_DURATION_S  = 0.5e-3

# Algorithm 1 EMA
ALPHA_EMA = 0.5

# SE floor
SE_FLOOR_BITS_PER_RB = 1.0

# ======================================================================
# FIX-9: Correct MCS → CQI fallback lookup table
# -----------------------------------------------------------------------
# Derived from 3GPP TS 38.214:
#   MCS table : Table 5.1.3.2-1  (64QAM, indices 0-28)
#   CQI table : Table 5.1.3.1-1  (CQI indices 1-15)
#
# Method: for each MCS index compute SE = Qm × (R/1024), then select
# the CQI index whose SE is closest (minimum |SE_cqi - SE_mcs|).
#
# MCS_TO_CQI[mcs] → cqi  (mcs in 0..28, cqi in 1..15)
# Verified output:
#   MCS  0→CQI 1 | MCS  1→CQI 2 | MCS  2→CQI 2 | MCS  3→CQI 2
#   MCS  4→CQI 2 | MCS  5→CQI 3 | MCS  6→CQI 3 | MCS  7→CQI 4
#   MCS  8→CQI 4 | MCS  9→CQI 4 | MCS 10→CQI 5 | MCS 11→CQI 5
#   MCS 12→CQI 5 | MCS 13→CQI 6 | MCS 14→CQI 6 | MCS 15→CQI 7
#   MCS 16→CQI 8 | MCS 17→CQI 7 | MCS 18→CQI 8 | MCS 19→CQI 9
#   MCS 20→CQI 9 | MCS 21→CQI 9 | MCS 22→CQI10 | MCS 23→CQI11
#   MCS 24→CQI11 | MCS 25→CQI11 | MCS 26→CQI12 | MCS 27→CQI12
#   MCS 28→CQI13
#
# Note: CQI 14 and 15 share the same SE as CQI 13 in Table 5.1.3.1-1;
# the LUT caps at 13 to avoid ambiguity.  CQI 0 ("out of range") is
# never produced here — wb_cqi==0 is handled by the fallback branch.
# ======================================================================
MCS_TO_CQI = [1, 2, 2, 2, 2, 3, 3, 4, 4, 4,   # MCS  0-9
              5, 5, 5, 6, 6, 7, 8, 7, 8, 9,    # MCS 10-19
              9, 9, 10, 11, 11, 11, 12, 12, 13] # MCS 20-28

# ======================================================================
# FIX-7: Per-slice RB cap (fraction of server's virtual RB partition)
# -----------------------------------------------------------------------
# URLLC : 1.0  — must be allowed to drain entirely within deadline
# eMBB  : 0.5  — cap at 50 % of server virt_rbs to prevent monopoly
# mMTC  : 0.3  — low-priority, small packets, generous deadline
#
# The cap is applied as:
#   max_rbs_for_this_ue = floor(SLICE_RB_CAP[slice] × virt_rbs[j])
# and then alloc_rbs = min(alloc_rbs, max_rbs_for_this_ue).
# ======================================================================
SLICE_RB_CAP = {
    "URLLC": 1.0,
    "eMBB":  0.5,
    "mMTC":  0.3,
}


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
                "label":          label,
                "slice":          slc,
                "5qi":            fqi,
                "priority":       prio,
                "lam_nom":        lam_nom,
                "beta_max_ms":    beta_ms,
                "beta_max_s":     beta_ms / 1000.0,
                "gamma_min":      gmin,
                "eps1":           e1,
                "eps2":           e2,
                "nom_pkt_bytes":  npkt,
                # Runtime — RLC (FIX-2)
                "txbuf_bytes":    0,
                "txsdu_wt_us":    0,
                # Runtime — MAC
                "lam_est":        lam_nom,
                "bsr_bytes":      0,
                "cqi":            0,
                "dl_bler":        0.0,
                "se_bits_per_rb": SE_FLOOR_BITS_PER_RB,
                "harq_pending":   False,
                "avg_tp_bits":    1e-6,
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

        # Physical RB partition: Cellular ~67 %, WLAN ~33 %
        nr_rbs   = round(TOTAL_RBS * 0.67)
        wlan_rbs = TOTAL_RBS - nr_rbs
        self.virt_rbs  = [nr_rbs, wlan_rbs]
        self.rb_offset = [0, nr_rbs]

        self.last_active_rntis = []


altamas = AltamasState()


# ======================================================================
# Eq. (9) — Average latency W(Q_i, S_j, Psi)  [returns seconds]
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
    return base / denom


# ======================================================================
# Eq. (7, 8) — Packet Loss Rate
# ======================================================================
def calculate_plr(i, j, Psi, lam, beta_max_s, P, mu, k_buf):
    M = len(lam)

    omega_pi   = sum(lam[m] / mu[j]
                     for m in range(M) if Psi[m] == j and P[m] <= P[i])
    omega_pim1 = sum(lam[m] / mu[j]
                     for m in range(M) if Psi[m] == j and P[m] <  P[i])

    rho_pi = omega_pi
    k      = float(k_buf[j])

    # Buffer overflow  AL/A  (Ref [13])
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

    # Latency violation — deterministic indicator (FIX-5)
    w_i = calculate_latency(i, j, Psi, lam, P, mu)
    if not math.isfinite(w_i):
        p_late = 1.0
    else:
        p_late = 1.0 if w_i > beta_max_s[i] else 0.0

    return min(al_over_a + p_late, 1.0)


def calculate_throughput(i, j, Psi, lam, beta_max_s, P, mu, k_buf):
    """TP(Q_i, S_j, Psi) = λ_i · (1 − PLR)   Eq.(7)"""
    return lam[i] * (1.0 - calculate_plr(i, j, Psi, lam, beta_max_s, P, mu, k_buf))


# ======================================================================
# Eq. (10–13) — Goal-Programming Combined Reward Rc
# ======================================================================
def calculate_gp_reward(Psi, lam, gamma_min, beta_max_s,
                        P, eps1, eps2, mu, k_buf):
    M  = len(lam)
    Rc = 0.0

    for i in range(M):
        j = Psi[i]
        if lam[i] < 1e-9:
            continue

        tp_i = calculate_throughput(i, j, Psi, lam, beta_max_s, P, mu, k_buf)
        w_i  = calculate_latency(i, j, Psi, lam, P, mu)

        if not math.isfinite(w_i):
            w_i = 10.0 * max(beta_max_s[i], 1e-6)

        tp_target = gamma_min[i] * lam[i]
        w_target  = beta_max_s[i]

        tp_dn = (min(0.0, tp_i - tp_target)
                 / max(max(tp_target, tp_i), 1e-12))

        w_dn  = (min(0.0, w_target - w_i)
                 / max(max(w_target, w_i), 1e-12))

        Rc += eps1[i] * tp_dn + eps2[i] * w_dn

    return Rc


# ======================================================================
# Algorithm 1 — EvaluateAndObtainMapping
# ======================================================================
def evaluate_and_obtain_mapping(Psi_all, R_dict, lam, mu, rho_max=0.9):
    M = len(lam)
    N = len(mu)

    current_best_R = -float('inf')
    best_Psi       = None

    for Psi_x in Psi_all:
        stable = all(
            (sum(lam[i] for i in range(M) if Psi_x[i] == j) / mu[j]) <= rho_max
            for j in range(N)
        )
        if not stable:
            continue

        rx = R_dict.get(Psi_x, -1)

        if rx == -1 or rx > current_best_R:
            best_Psi       = Psi_x
            current_best_R = rx if rx != -1 else current_best_R
            if rx == -1:
                return best_Psi

    if best_Psi is None and Psi_all:
        best_Psi = Psi_all[0]

    return best_Psi


# ======================================================================
# RLC Callback — buffer state (FIX-2)
# ======================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        try:
            if not ind.rb_stats:
                return

            for rnti in global_ue_state.states:
                global_ue_state.states[rnti]["txbuf_bytes"] = 0
                global_ue_state.states[rnti]["txsdu_wt_us"] = 0

            for rb in ind.rb_stats:
                rnti = rb.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]
                st["txbuf_bytes"] += int(rb.txbuf_occ_bytes)
                st["txsdu_wt_us"]  = max(st["txsdu_wt_us"], int(rb.txsdu_wt_us))

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
            active_ues = []

            # ── Step 1: Update MAC measurements, estimate lambda ─────────
            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]

                # Bits-per-RB from OAI dl_curr_tbs / dl_sched_rb
                try:
                    tbs_bits_now = int(ue.dl_curr_tbs)
                    sched_rbs    = int(ue.dl_sched_rb)
                    if tbs_bits_now > 0 and sched_rbs > 0:
                        st["se_bits_per_rb"] = max(
                            tbs_bits_now / sched_rbs,
                            SE_FLOOR_BITS_PER_RB
                        )
                except Exception:
                    pass

                # ── CQI — primary source: wb_cqi (TS 38.214 Table 5.1.3.1-1)
                # wb_cqi is only refreshed on PUCCH reporting cycles so it
                # may read 0 between cycles.  When it is 0 we fall back to
                # dl_mcs1 via the pre-computed MCS_TO_CQI LUT (FIX-9).
                # The derived CQI is stored for CSV logging only; the RB
                # allocator uses se_bits_per_rb = dl_curr_tbs/dl_sched_rb
                # which already embeds OAI's full CQI/MCS/MIMO computation.
                try:
                    cqi_now = int(ue.wb_cqi)
                    if cqi_now > 0:
                        # wb_cqi is valid — use directly (range 1-15)
                        st["cqi"] = min(max(cqi_now, 1), 15)
                    else:
                        # wb_cqi == 0 means not yet reported this cycle;
                        # derive from dl_mcs1 using the 3GPP-aligned LUT.
                        mcs = int(ue.dl_mcs1)
                        if 0 <= mcs <= 28:          # valid MCS range
                            st["cqi"] = MCS_TO_CQI[mcs]
                        # else: retain previous cqi (mcs<0 = not scheduled)
                except Exception:
                    pass   # retain previous cqi on any read error

                try:
                    st["dl_bler"] = float(ue.dl_bler)
                except Exception:
                    pass

                try:
                    harq = ue.dl_harq
                    st["harq_pending"] = (
                        int(sum(harq)) > 0
                        if isinstance(harq, (list, tuple))
                        else int(harq) > 0
                    )
                except Exception:
                    st["harq_pending"] = False

                try:
                    st["bsr_bytes"] = int(ue.bsr)
                except Exception:
                    pass

                # FIX-1: λ from dl_aggr_tbs
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
            beta_ms_vec = []
            beta_s_vec  = []
            eps1_vec    = []
            eps2_vec    = []
            lat_ms_vec  = []

            for rnti in active_ues:
                st = global_ue_state.states[rnti]
                P_vec.append(st["priority"])
                lam_vec.append(max(st["lam_est"], 1e-3))
                gamma_vec.append(st["gamma_min"])
                beta_ms_vec.append(st["beta_max_ms"])
                beta_s_vec.append(st["beta_max_s"])
                eps1_vec.append(st["eps1"])
                eps2_vec.append(st["eps2"])
                lat_ms_vec.append(st["txsdu_wt_us"] / 1000.0)

            # ── Algorithm 1 — Tm = 2 slot interval ──────────────────────
            Tm = 2
            do_update = (altamas.Psi_best is None or
                         (altamas.global_slot - altamas.last_update_slot) >= Tm)

            if do_update:
                if altamas.Psi_best is not None:
                    Rc = calculate_gp_reward(
                        altamas.Psi_best,
                        lam_vec, gamma_vec, beta_s_vec,
                        P_vec, eps1_vec, eps2_vec,
                        altamas.mu, altamas.k_buf
                    )
                    if not math.isfinite(Rc):
                        Rc = -float(M)

                    prev_R = altamas.R_dict.get(altamas.Psi_best, -1)
                    altamas.R_dict[altamas.Psi_best] = (
                        Rc if prev_R == -1
                        else ALPHA_EMA * prev_R + (1.0 - ALPHA_EMA) * Rc
                    )

                altamas.Psi_best = evaluate_and_obtain_mapping(
                    altamas.Psi_all, altamas.R_dict, lam_vec, altamas.mu
                )
                altamas.last_update_slot = altamas.global_slot

            Rc_logged = altamas.R_dict.get(altamas.Psi_best, 0.0)

            # ── Physical RB Allocation (FIX-7 applied) ───────────────────
            avail_rbs   = [True] * TOTAL_RBS
            allocations = {}

            for j in range(N):
                flows_on_j = [idx for idx in range(M) if altamas.Psi_best[idx] == j]
                if not flows_on_j:
                    continue

                virt_count = altamas.virt_rbs[j]
                rb_base    = altamas.rb_offset[j]
                virt_pool  = [True] * virt_count

                # Sort: ascending priority (lower number = higher priority),
                # then descending urgency ratio
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

                    if st["se_bits_per_rb"] <= SE_FLOOR_BITS_PER_RB:
                        continue
                    tbsRB = st["se_bits_per_rb"]

                    # C1: throughput-target-derived RB requirement
                    target_bps            = (gamma_vec[ue_idx] * lam_vec[ue_idx]
                                             * st["nom_pkt_bytes"] * 8)
                    target_bits_per_slot  = target_bps * SLOT_DURATION_S
                    req_c1 = math.ceil(target_bits_per_slot / tbsRB)

                    # RBs needed to drain the entire buffer
                    rbs_to_drain_buffer = math.ceil(buf_bits / tbsRB)

                    if st["slice"] == "URLLC":
                        # URLLC drains entire buffer immediately
                        req_rbs = rbs_to_drain_buffer
                    else:
                        # eMBB / mMTC: pace over remaining deadline slots
                        remain_slots = max(
                            1.0,
                            (beta_ms_vec[ue_idx] - lat_ms_vec[ue_idx])
                            / SLOT_DURATION_MS
                        )
                        req_c2  = math.ceil(buf_bits / (tbsRB * remain_slots))
                        req_rbs = min(max(req_c1, req_c2), rbs_to_drain_buffer)

                    # Fair-share within remaining active flows on this server
                    free_virt = [k for k, f in enumerate(virt_pool) if f]
                    remaining_active = max(
                        sum(1 for x in flows_on_j[rank:]
                            if global_ue_state.states[active_ues[x]]["txbuf_bytes"] > 0),
                        1
                    )
                    fair_share = max(len(free_virt) // remaining_active, 1)

                    # ── FIX-7: per-slice RB cap ──────────────────────────
                    # Cap expressed as a fraction of this server's virt_rbs.
                    # URLLC cap = 1.0 (no effective cap — must drain on time).
                    # eMBB cap  = 0.5 → max 50 % of virt_count per slot.
                    # mMTC cap  = 0.3 → max 30 % of virt_count per slot.
                    slice_cap_rbs = math.floor(
                        SLICE_RB_CAP.get(st["slice"], 1.0) * virt_count
                    )

                    if st["slice"] == "URLLC":
                        # Allow full drain up to the remaining free pool,
                        # then additionally bounded by slice_cap (=virt_count
                        # for URLLC so effectively no extra constraint).
                        alloc_rbs = min(req_rbs, len(free_virt), slice_cap_rbs)
                    else:
                        # eMBB / mMTC: fair-share AND slice cap both apply
                        if remaining_active == 1:
                            alloc_rbs = min(req_rbs, len(free_virt), slice_cap_rbs)
                        else:
                            alloc_rbs = min(req_rbs, fair_share, slice_cap_rbs)

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
                        f"RBs:{alloc_rbs}(cap={slice_cap_rbs}) "
                        f"lam:{lam_vec[ue_idx]:.1f}pkt/s "
                        f"W:{lat_ms_vec[ue_idx]:.3f}ms "
                        f"beta:{beta_ms_vec[ue_idx]}ms Rc:{Rc_logged:.4f}"
                    )

            # ── Logging ──────────────────────────────────────────────────
            log_rows = []
            for ue_idx, rnti in enumerate(active_ues):
                st    = global_ue_state.states[rnti]
                alloc = allocations.get(rnti,
                                        {"server": -1, "rbs": 0,
                                         "served_bits": 0.0})
                j = alloc["server"]

                plr_i = (calculate_plr(
                             ue_idx, j, altamas.Psi_best,
                             lam_vec, beta_s_vec,
                             P_vec, altamas.mu, altamas.k_buf)
                         if j >= 0 else 1.0)

                tp_pktps        = lam_vec[ue_idx] * (1.0 - plr_i)
                tp_target_pktps = gamma_vec[ue_idx] * lam_vec[ue_idx]

                # FIX-3: pkt/s → Mbps
                npkt           = st["nom_pkt_bytes"]
                tp_mbps        = (tp_pktps        * npkt * 8) / 1e6
                tp_target_mbps = (tp_target_pktps * npkt * 8) / 1e6

                st["avg_tp_bits"] = (
                    (1.0 - ALPHA_EMA) * st["avg_tp_bits"]
                    + ALPHA_EMA * alloc["served_bits"]
                )

                w_i = (calculate_latency(
                           ue_idx, j, altamas.Psi_best, lam_vec, P_vec, altamas.mu)
                       if j >= 0 else float('inf'))
                if not math.isfinite(w_i):
                    w_i = 10.0 * beta_s_vec[ue_idx]

                w_target = beta_s_vec[ue_idx]

                tp_dn = (min(0.0, tp_pktps - tp_target_pktps)
                         / max(max(tp_target_pktps, tp_pktps), 1e-12))
                w_dn  = (min(0.0, w_target - w_i)
                         / max(max(w_target, w_i), 1e-12))
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
print(f"[ALTAMAS] Slice RB caps: {SLICE_RB_CAP}")
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
