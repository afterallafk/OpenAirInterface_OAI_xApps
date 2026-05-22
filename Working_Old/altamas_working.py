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
    ue.rnti, ue.wb_cqi, ue.dl_mcs1, ue.dl_curr_tbs, ue.dl_harq,
    ue.dl_sched_rb, ue.bsr, ue.pucch_snr, ue.pusch_snr

Bug fixes applied vs previous version
--------------------------------------
FIX-1  λ estimation: replaced RLC byte-counter delta (which produced λ
       100–1000x too large due to ~50 µs tstamp deltas) with MAC
       dl_curr_tbs-based rate, clamped to [0.1·λ_nom, 2·λ_nom].
FIX-2  RLC callback no longer estimates λ at all — only updates buffer
       state (txbuf_bytes, txsdu_wt_us).  Removed _prev_txsdu_bytes /
       _prev_tstamp from per-UE state.
FIX-3  Throughput_Mbps column now correctly logged in Mbps:
       tp_achieved_pktps × nom_pkt_bytes × 8 / 1e6.
       Previously it was logged as raw pkt/s (unit mismatch).
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
    # Slot 0: URLLC_1
    (0, "URLLC_1", "URLLC", 86, 1,  900,   1.0, 0.99999, 0.2, 0.8, 100),
    # Slot 1: eMBB_1
    (1, "eMBB_1",  "eMBB",   9, 5, 7000,  15.0, 0.999,   0.5, 0.5, 1500),
    # Slot 2: mMTC_1
    (2, "mMTC_1",  "mMTC",  70, 8, 2000,  30.0, 0.99,    0.8, 0.2,  64),
    # Slot 3: URLLC_2
    (3, "URLLC_2", "URLLC", 86, 2,  800,   3.0, 0.99999, 0.2, 0.8, 100),
    # Slot 4: eMBB_2
    (4, "eMBB_2",  "eMBB",   9, 6, 5000,  15.0, 0.999,   0.5, 0.5, 1500),
    # Slot 5: mMTC_2
    (5, "mMTC_2",  "mMTC",  70, 9, 2000,  30.0, 0.99,    0.8, 0.2,  64),
    # Slot 6: URLLC_3
    (6, "URLLC_3", "URLLC", 86, 3,  900,   1.0, 0.99999, 0.2, 0.8, 100),
    # Slot 7: URLLC_4
    (7, "URLLC_4", "URLLC", 86, 4,  800,   3.0, 0.99999, 0.2, 0.8, 100)
]

# Server configuration — paper Table III
#   (id, label, mu_pktps, buffer_k, access_type)
SERVERS_CFG = [
    (0, "Cellular", 9000, 80, "NR"),
    (1, "WLAN",     6000, 50, "WLAN"),
]

N_SERVERS = len(SERVERS_CFG)           # N = 2
M_FLOWS   = len(QOS_FLOWS_CFG)         # M = 10   → L = 2^10 = 1024 mappings

# Radio / slot constants
TOTAL_RBS        = 106
SLOT_DURATION_MS = 0.5      # NR numerology-1 slot duration (ms)
ALPHA_EMA        = 0.5      # Algorithm 1 Step-2 EMA coefficient (0.5·R + 0.5·Rc)

# CQI → spectral efficiency  (3GPP TS 38.214, Table 5.2.2.1-3)
CQI_TO_SE = [0.0, 0.1523, 0.2344, 0.3770, 0.6016, 0.8770,
             1.1758, 1.4766, 1.9141, 2.4063, 2.7305, 3.3223,
             3.9023, 4.5234, 5.1152, 5.5547]

# MCS → spectral efficiency  (fallback when CQI unavailable)
MCS_TO_SE = [
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953, 1.9141,
    2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547, 5.5547,
    5.5547, 6.2266, 6.9141, 7.4063, 7.4063, 7.4063, 7.4063, 7.4063,
    7.4063, 7.4063, 7.4063, 7.4063, 7.4063,
]


# ======================================================================
# Global UE State
# ======================================================================
class GlobalUEState:
    """
    Per-RNTI runtime state.  Each new RNTI is assigned a QoS flow profile
    from QOS_FLOWS_CFG in round-robin order.
    """

    def __init__(self):
        self.states     = {}    # rnti (int) → state dict
        self.rnti_order = []    # insertion order for round-robin assignment
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
                "priority":      prio,       # P_i — lower = higher priority
                "lam_nom":       lam_nom,    # nominal λ_i (pkt/s)
                "beta_max_ms":   beta_ms,    # β_max_i (ms)
                "gamma_min":     gmin,       # γ_min_i
                "eps1":          e1,         # ε1 throughput goal weight
                "eps2":          e2,         # ε2 latency goal weight
                "nom_pkt_bytes": npkt,       # for Mbps conversion  [FIX-3]
                # Runtime — updated by RLC callback
                "txbuf_bytes":   0,          # DL transmit buffer occupancy (bytes)
                "txsdu_wt_us":   0,          # head-of-line SDU wait time (µs)
                # Runtime — updated by MAC callback  [FIX-1]
                "lam_est":       lam_nom,    # estimated λ_i (pkt/s), seeded with nominal
                "bsr_bytes":     0,
                "cqi":           0,
                "se":            0.15,       # spectral efficiency (bits/RE)
                "harq_pending":  False,
                # EMA served-bits tracker
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
    """
    Goal-programming reward history R[l] and current best decision
    matrix Psi_o.  Exact match to Algorithm 1.
    """

    def __init__(self):
        self.R_dict   = {}      # Psi tuple -> float reward, default -1
        self.Psi_all  = list(itertools.product(range(N_SERVERS), repeat=M_FLOWS))
        self.Psi_best = None

        self.global_slot      = 0
        self.last_update_slot = 0

        # Server params from Table III
        self.mu    = [cfg[2] for cfg in SERVERS_CFG]   # serving rates (pkt/s)
        self.k_buf = [cfg[3] for cfg in SERVERS_CFG]   # buffer capacities

        # Physical RB partition: Cellular ~67 %, WLAN ~33 %
        nr_rbs   = round(TOTAL_RBS * 0.67)
        wlan_rbs = TOTAL_RBS - nr_rbs
        self.virt_rbs  = [nr_rbs, wlan_rbs]
        self.rb_offset = [0, nr_rbs]

        self.last_active_rntis = []


altamas = AltamasState()


# ======================================================================
# Eq. (9) — Average latency W(Q_i, S_j, Psi)
# ======================================================================
def calculate_latency(i, j, Psi, lam, P, mu):
    """
    Preemptive priority M/M/1 queueing latency (Kleinrock Vol. II, s3.4).

    W = [1/mu_j  +  Omega(P_i) / (mu_j*(1-Omega(P_i)))]
        / (1 - Omega(P_i-1))                                   Eq.(9)

    Omega(P_i,  S_j, Psi) = sum_{m: Psi[m]==j, P[m]<=P[i]}  lam_m / mu_j
    Omega(P_i-1,S_j, Psi) = sum_{m: Psi[m]==j, P[m]< P[i]}  lam_m / mu_j

    Lower P value = higher preemption priority.
    Returns inf when the server is unstable for this flow.
    """
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
# ======================================================================
def calculate_plr(i, j, Psi, lam, beta_max_ms, P, mu, k_buf):
    """
    PLR(Q_i, S_j, Psi) = AL(P_i)/A(P_i)  +  P(W > beta_max_i)   Eq.(8)

    Buffer-overflow term: finite-buffer Erlang-B formula (Ref [13]).
    Latency-violation term: exponential sojourn-time CDF of M/M/1.
    """
    M = len(lam)
    omega_pi = sum(lam[m] / mu[j]
                   for m in range(M) if Psi[m] == j and P[m] <= P[i])
    rho_pi = omega_pi   # already normalised by mu[j]
    k      = float(k_buf[j])

    # Buffer overflow  AL/A  (Ref [13])
    if rho_pi <= 0.0:
        al_over_a = 0.0
    elif abs(rho_pi - 1.0) < 1e-9:
        al_over_a = 1.0 / (k + 1.0)
    elif rho_pi < 1.0:
        num   = (rho_pi ** k) * (1.0 - rho_pi)
        denom = max(1.0 - rho_pi ** (k + 1.0), 1e-12)
        al_over_a = num / denom
    else:
        al_over_a = 1.0

    # Latency violation  P(W > beta_max)  — exponential CDF
    rho_tot = min(sum(lam[m] / mu[j] for m in range(M) if Psi[m] == j),
                  1.0 - 1e-9)
    mu_eff  = mu[j] * (1.0 - rho_tot)

    w_i = calculate_latency(i, j, Psi, lam, P, mu)
    if not math.isfinite(w_i):
        p_late = 1.0
    elif beta_max_ms[i] <= 0.0:
        p_late = 1.0
    else:
        p_late = math.exp(-mu_eff * (beta_max_ms[i] / 1000.0))  # ms -> s

    return min(al_over_a + p_late, 1.0)


def calculate_throughput(i, j, Psi, lam, beta_max_ms, P, mu, k_buf):
    """TP(Q_i, S_j, Psi) = lam_i * (1 - PLR)   Eq.(7)"""
    return lam[i] * (1.0 - calculate_plr(i, j, Psi, lam, beta_max_ms,
                                          P, mu, k_buf))


# ======================================================================
# Eq. (10–13) — Goal-Programming Combined Reward R_c
# ======================================================================
def calculate_gp_reward(Psi, lam, gamma_min, beta_max_ms,
                        P, eps1, eps2, mu, k_buf):
    """
    R_c(Q, S, Psi) = sum_i  U(Q_i, S_j, Psi)              Eq.(10)

    U_i = eps1_i * TPdelta_norm  +  eps2_i * Wdelta_norm   Eq.(11)

    TPdelta_norm = min[0, TP - gamma*lam] / max(gamma*lam, TP) Eq.(12)
    Wdelta_norm  = min[0, beta - W]       / max(beta, W)       Eq.(13)

    U_i in [-1, 0];  R_c in [-M, 0].  Scheduling pushes R_c -> 0.
    """
    M  = len(lam)
    Rc = 0.0
    for i in range(M):
        j = Psi[i]
        if lam[i] < 1e-9:
            continue

        tp_i = calculate_throughput(i, j, Psi, lam, beta_max_ms, P, mu, k_buf)
        w_i  = calculate_latency(i, j, Psi, lam, P, mu)
        if not math.isfinite(w_i):
            w_i = 10.0 * max(beta_max_ms[i], 1.0)

        tp_target = gamma_min[i] * lam[i]   # C1  Eq.(5)
        w_target  = beta_max_ms[i]          # C2  Eq.(6)

        tp_dn = (min(0.0, tp_i - tp_target)
                 / max(max(tp_target, tp_i), 1e-12))   # Eq.(12)
        w_dn  = (min(0.0, w_target - w_i)
                 / max(max(w_target, w_i), 1e-12))     # Eq.(13)

        Rc += eps1[i] * tp_dn + eps2[i] * w_dn         # Eq.(11)

    return Rc


# ======================================================================
# Algorithm 1 — EvaluateAndObtainMapping
# ======================================================================
def evaluate_and_obtain_mapping(Psi_all, R_dict, lam, mu, rho_max=0.9):
    """
    Algorithm 1 — EvaluateAndObtainMapping(Q, S, Psi).

    For each candidate Psi_x:
      1. Stability check: sum_i lam_i * psi_ij / mu_j <= rho_max  Eq.(2)
      2. Among stable candidates, return the one with highest R[x].
         Uninitialised entries (R == -1) are used only as a fallback
         when no evaluated mapping exists yet.
    """
    M = len(lam)
    N = len(mu)

    best_Psi = None
    best_R   = -float('inf')

    for Psi_x in Psi_all:
        # Stability  Eq.(2)
        stable = all(
            (sum(lam[i] for i in range(M) if Psi_x[i] == j) / mu[j]) <= rho_max
            for j in range(N)
        )
        if not stable:
            continue

        rx = R_dict.get(Psi_x, -1)

        if rx == -1:
            if best_Psi is None:        # keep as fallback only
                best_Psi = Psi_x
                best_R   = -1.0
        else:
            if rx > best_R:             # evaluated always beats uninitialised
                best_R   = rx
                best_Psi = Psi_x

    if best_Psi is None and Psi_all:    # absolute fallback
        best_Psi = Psi_all[0]

    return best_Psi


# ======================================================================
# RLC Callback — buffer state only
# FIX-2: lambda estimation removed; previously caused PLR=1 always
# ======================================================================
class RLCCallback(ric.rlc_cb):
    """
    Updates DL buffer occupancy and head-of-line wait time per UE.

    FIX-2 explanation:
      The previous implementation estimated lambda from RLC txsdu_bytes
      counter deltas.  Because the RLC handle() fires with tstamp deltas
      of ~50 us (not the 10 ms report interval), dt_s was ~0.00005 s,
      making lambda = delta_bytes / dt_s / pkt_size roughly 100-1000x
      larger than the server mu values.  This drove rho >> 1 for every
      flow, making the queueing model declare PLR=1 and therefore
      Throughput=0 on every single report.

      Lambda is now estimated cleanly from MAC dl_curr_tbs (FIX-1).
    """

    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        try:
            for rb in ind.rb_stats:
                rnti = rb.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]
                # DL transmit buffer occupancy — feeds queueing model
                st["txbuf_bytes"] = int(rb.txbuf_occ_bytes)
                # Head-of-line SDU wait (us) — experienced latency proxy
                st["txsdu_wt_us"] = int(rb.txsdu_wt_us)
        except Exception as e:
            print(f"[RLC ERROR] {e}")
            traceback.print_exc()


# ======================================================================
# MAC Callback — ALTAMAS Algorithm 1 + RB allocation
# ======================================================================
class MACCallback(ric.mac_cb):
    """
    Executes every MAC report interval (~10 ms).

    1. Update CQI/MCS/HARQ/BSR per UE.
    2. FIX-1: Estimate lambda from dl_curr_tbs (bits/TTI -> pkt/s),
       EMA-smoothed and clamped to [0.1*lam_nom, 2*lam_nom].
    3. Algorithm 1 every Tm=2 slots: compute R_c, update R[l], select Psi_o.
    4. Allocate RBs per flow within its server partition (C1 + C2 aware).
    5. FIX-3: Log Throughput_Mbps = tp_pktps * nom_pkt_bytes * 8 / 1e6.
    """

    def __init__(self):
        ric.mac_cb.__init__(self)

    def handle(self, ind):
        try:
            if not ind.ue_stats:
                return

            t_now = ind.tstamp

            # ----------------------------------------------------------
            # Step 1 + FIX-1: Update MAC measurements, estimate lambda
            # ----------------------------------------------------------
            active_ues = []

            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]

                # CQI / MCS -> spectral efficiency
                try:
                    cqi_val = int(ue.wb_cqi)
                    mcs_val = int(ue.dl_mcs1)
                    if cqi_val > 0:
                        st["se"]  = CQI_TO_SE[min(cqi_val, 15)]
                        st["cqi"] = cqi_val
                    elif mcs_val > 0:
                        st["se"]  = MCS_TO_SE[min(mcs_val, 28)]
                        st["cqi"] = mcs_val
                    else:
                        st["se"]  = 0.15
                        st["cqi"] = 0
                except Exception:
                    st["se"]  = 0.15
                    st["cqi"] = 0

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

                # FIX-1: lambda from dl_curr_tbs
                # dl_curr_tbs = bits scheduled this 0.5 ms TTI
                # pkt/s = (bits/8 bytes / nom_pkt_bytes) / slot_duration_s
                try:
                    tbs_bits = int(ue.dl_curr_tbs)
                except Exception:
                    tbs_bits = 0

                if tbs_bits > 0:
                    npkt    = st["nom_pkt_bytes"]
                    lam_raw = (tbs_bits / 8.0 / npkt) / (SLOT_DURATION_MS / 1000.0)
                    # EMA smooth (alpha=0.3 for lambda — moderate responsiveness)
                    lam_smoothed = 0.7 * st["lam_est"] + 0.3 * lam_raw
                    # Hard clamp: never let lambda blow past 2x nominal
                    # This prevents a single noisy report from driving rho >> 1
                    st["lam_est"] = max(st["lam_nom"] * 0.1,
                                        min(lam_smoothed, st["lam_nom"] * 2.0))

                # Active if DL buffer has data pending
                if st["txbuf_bytes"] > 0:
                    active_ues.append(rnti)

            if not active_ues:
                return

            altamas.global_slot += 1
            M = len(active_ues)
            N = N_SERVERS  # 2

            # ----------------------------------------------------------
            # Topology change -> reset reward history
            # ----------------------------------------------------------
            if active_ues != altamas.last_active_rntis:
                altamas.Psi_all           = list(itertools.product(range(N),
                                                                    repeat=M))
                altamas.R_dict            = {}
                altamas.Psi_best          = None
                altamas.last_active_rntis = active_ues.copy()

            # ----------------------------------------------------------
            # Build per-flow parameter vectors
            # ----------------------------------------------------------
            P_vec      = []
            lam_vec    = []
            gamma_vec  = []
            beta_vec   = []
            eps1_vec   = []
            eps2_vec   = []
            lat_ms_vec = []

            for rnti in active_ues:
                st = global_ue_state.states[rnti]
                P_vec.append(st["priority"])
                lam_vec.append(max(st["lam_est"], 1e-3))
                gamma_vec.append(st["gamma_min"])
                beta_vec.append(st["beta_max_ms"])
                eps1_vec.append(st["eps1"])
                eps2_vec.append(st["eps2"])
                lat_ms_vec.append(st["txsdu_wt_us"] / 1000.0)  # us -> ms

            # ----------------------------------------------------------
            # Algorithm 1 — Tm = 2 slot interval
            # ----------------------------------------------------------
            Tm = 2
            do_update = (altamas.Psi_best is None or
                         (altamas.global_slot - altamas.last_update_slot) >= Tm)

            if do_update:
                # Step 1: evaluate current mapping's R_c
                if altamas.Psi_best is not None:
                    Rc = calculate_gp_reward(
                        altamas.Psi_best,
                        lam_vec, gamma_vec, beta_vec,
                        P_vec, eps1_vec, eps2_vec,
                        altamas.mu, altamas.k_buf
                    )
                    if not math.isfinite(Rc):
                        Rc = -float(M)

                    # Step 2: R[l] = 0.5*R[l] + 0.5*R_c  (Algorithm 1)
                    prev_R = altamas.R_dict.get(altamas.Psi_best, -1)
                    altamas.R_dict[altamas.Psi_best] = (
                        Rc if prev_R == -1
                        else ALPHA_EMA * prev_R + (1.0 - ALPHA_EMA) * Rc
                    )

                # Step 3: select new optimal Psi_o
                altamas.Psi_best = evaluate_and_obtain_mapping(
                    altamas.Psi_all, altamas.R_dict, lam_vec, altamas.mu
                )
                altamas.last_update_slot = altamas.global_slot

            Rc_logged = altamas.R_dict.get(altamas.Psi_best, 0.0)

            # ----------------------------------------------------------
            # Physical RB Allocation
            # Each server gets a fixed RB partition.
            # Within a server: flows sorted by priority then urgency.
            # ----------------------------------------------------------
            avail_rbs   = [True] * TOTAL_RBS
            allocations = {}

            for j in range(N):
                flows_on_j = [idx for idx in range(M)
                              if altamas.Psi_best[idx] == j]
                if not flows_on_j:
                    continue

                virt_count = altamas.virt_rbs[j]
                rb_base    = altamas.rb_offset[j]
                virt_pool  = [True] * virt_count

                # Sort: ascending priority (lower = higher), then descending urgency
                flows_on_j.sort(key=lambda idx: (
                    P_vec[idx],
                    -(lat_ms_vec[idx] / max(beta_vec[idx], 1.0))
                ))

                for rank, ue_idx in enumerate(flows_on_j):
                    rnti = active_ues[ue_idx]
                    st   = global_ue_state.states[rnti]

                    buf_bits = st["txbuf_bytes"] * 8
                    if buf_bits <= 0:
                        continue

                    # Bits per RB per slot: SE * 12 subcarriers * 14 OFDM symbols
                    tbs_per_rb = max(st["se"] * 12.0 * 14.0, 1.0)

                    # C1 — min RBs to meet throughput target  Eq.(5)
                    req_c1 = math.ceil(
                        (gamma_vec[ue_idx] * lam_vec[ue_idx]) / tbs_per_rb
                    )

                    # C2 — RBs to drain buffer before deadline  Eq.(6)
                    remain_slots = max(
                        1.0,
                        (beta_vec[ue_idx] - lat_ms_vec[ue_idx]) / SLOT_DURATION_MS
                    )
                    req_c2  = math.ceil(buf_bits / (tbs_per_rb * remain_slots))
                    req_rbs = min(max(req_c1, req_c2),
                                  math.ceil(buf_bits / tbs_per_rb))

                    # Deadline already exceeded -> drain at max rate
                    if lat_ms_vec[ue_idx] >= beta_vec[ue_idx]:
                        req_rbs = math.ceil(buf_bits / tbs_per_rb)

                    # Fair-share cap within this server partition
                    free_virt = [k for k, f in enumerate(virt_pool) if f]
                    remaining_active = max(
                        sum(1 for x in flows_on_j[rank:]
                            if global_ue_state.states[active_ues[x]]["txbuf_bytes"] > 0),
                        1
                    )
                    fair_share = max(len(free_virt) // remaining_active, 1)
                    alloc_rbs  = (min(req_rbs, len(free_virt))
                                  if remaining_active == 1
                                  else min(req_rbs, fair_share))
                    alloc_rbs  = max(alloc_rbs, 0)
                    if alloc_rbs <= 0:
                        continue

                    taken = free_virt[:alloc_rbs]
                    for tv in taken:
                        virt_pool[tv]           = False
                        avail_rbs[rb_base + tv] = False

                    served_bits = alloc_rbs * tbs_per_rb
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
                        f"beta:{beta_vec[ue_idx]}ms Rc:{Rc_logged:.4f}"
                    )

            # ----------------------------------------------------------
            # Logging — FIX-3: Throughput_Mbps in Mbps
            # ----------------------------------------------------------
            log_rows = []
            for ue_idx, rnti in enumerate(active_ues):
                st    = global_ue_state.states[rnti]
                alloc = allocations.get(rnti,
                                        {"server": -1, "rbs": 0,
                                         "served_bits": 0.0})
                j = alloc["server"]

                # Queueing-model PLR and throughput
                plr_i = (calculate_plr(
                             ue_idx, j, altamas.Psi_best,
                             lam_vec, beta_vec, P_vec,
                             altamas.mu, altamas.k_buf)
                         if j >= 0 else 1.0)

                tp_pktps        = lam_vec[ue_idx] * (1.0 - plr_i)
                tp_target_pktps = gamma_vec[ue_idx] * lam_vec[ue_idx]

                # FIX-3: pkt/s -> Mbps
                npkt           = st["nom_pkt_bytes"]
                tp_mbps        = (tp_pktps        * npkt * 8) / 1e6
                tp_target_mbps = (tp_target_pktps * npkt * 8) / 1e6

                # EMA served-bits
                st["avg_tp_bits"] = (
                    (1.0 - ALPHA_EMA) * st["avg_tp_bits"]
                    + ALPHA_EMA * alloc["served_bits"]
                )

                # Per-flow goal deviations
                w_i = (calculate_latency(
                           ue_idx, j, altamas.Psi_best, lam_vec, P_vec, altamas.mu)
                       if j >= 0 else float('inf'))
                if not math.isfinite(w_i):
                    w_i = 10.0 * beta_vec[ue_idx]

                tp_dn = (min(0.0, tp_pktps - tp_target_pktps)
                         / max(max(tp_target_pktps, tp_pktps), 1e-12))
                w_dn  = (min(0.0, beta_vec[ue_idx] - w_i)
                         / max(max(beta_vec[ue_idx], w_i), 1e-12))
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
                    round(beta_vec[ue_idx], 3),
                    round(plr_i, 6),
                    round(tp_mbps, 6),          # FIX-3
                    round(tp_target_mbps, 6),   # FIX-3
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
