import xapp_sdk as ric
import time
import csv
import math
import os
import traceback

# =====================================================================
# SDAP Layer & 5G QoS Configuration (8 UEs)
# 4 URLLC, 2 eMBB, 2 mMTC
# =====================================================================
QOS_MAPPING = {
    0: {"type": "URLLC", "label": "URLLC_1", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    1: {"type": "eMBB",  "label": "eMBB_1",  "5qi": 9,  "priority": 90, "pdb_ms": 300.0},
    2: {"type": "mMTC",  "label": "mMTC_1",  "5qi": 70, "priority": 55, "pdb_ms": 200.0},
    3: {"type": "URLLC", "label": "URLLC_2", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    4: {"type": "eMBB",  "label": "eMBB_2",  "5qi": 9,  "priority": 90, "pdb_ms": 300.0},
    5: {"type": "mMTC",  "label": "mMTC_2",  "5qi": 70, "priority": 55, "pdb_ms": 200.0},
    6: {"type": "URLLC", "label": "URLLC_3", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    7: {"type": "URLLC", "label": "URLLC_4", "5qi": 86, "priority": 18, "pdb_ms": 5.0}
}

# CQI to Spectral Efficiency Mapping — 3GPP TS 38.214 Table 5.2.2.1-2 (CQI Table 1, max 64QAM)
# Indices 0-15; index 0 = out-of-range.
CQI_TO_SE = [0.0, 0.1523, 0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766,
             1.9141, 2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547]

# -----------------------------------------------------------------------
# SLOT_DURATION_MS: 30 kHz SCS → 0.5 ms per slot
# -----------------------------------------------------------------------
SLOT_DURATION_MS = 0.5          # ms  (= 0.5e-3 s)
SLOT_DURATION_S  = 0.5e-3       # seconds

TOTAL_RBS  = 106    # Adjust: 106 RBs for 40 MHz, 275 RBs for 100 MHz NR
NUM_LAYERS = 2      # 2×2 MIMO
NUM_SYMB   = 14     # OFDM symbols per slot (normal CP, 30 kHz SCS)
SC_PER_RB  = 12     # Subcarriers per RB (fixed in 5G NR)

ALPHA_EMA  = 0.08   # EMA smoothing coefficient

# -----------------------------------------------------------------------
# tbs_per_rb(se): approximation for one RB, NUM_LAYERS layers, NUM_SYMB symbols
#   = SE × SC_PER_RB × NUM_SYMB × NUM_LAYERS   (bits)
# -----------------------------------------------------------------------
def tbs_per_rb(se: float) -> float:
    """Approximate TBS for 1 RB, 2 MIMO layers, 14 OFDM symbols."""
    return se * SC_PER_RB * NUM_SYMB * NUM_LAYERS


class GlobalUEState:
    def __init__(self):
        self.states     = {}
        self.rnti_order = []
        self.csv_file   = "pf_results.csv"

        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "tstamp", "RNTI", "TrafficType", "5QI", "Priority",
                "CQI", "Buffer_Bytes", "Latency_ms", "Deadline_ms",
                "PF_Metric", "RBs_Granted", "Served_Bits", "Throughput_Mbps"
            ])

    def register_ue(self, rnti):
        if rnti not in self.states:
            idx = len(self.rnti_order) % 8
            qos = QOS_MAPPING[idx]
            self.states[rnti] = {
                "type":           qos["type"],
                "label":          qos["label"],
                "5qi":            qos["5qi"],
                "priority":       qos["priority"],
                "pdb_ms":         qos["pdb_ms"],
                "deadline_slots": math.ceil(qos["pdb_ms"] / SLOT_DURATION_MS),
                "buffer_bytes":   0,
                "latency_us":     0,
                "avg_throughput": 1e-6,   # initialised > 0 to avoid div-by-zero
                "cqi":            0,
                "se":             0.15,
                "harq_pending":   False
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} as {qos['label']} "
                  f"(5QI: {qos['5qi']}, deadline: {self.states[rnti]['deadline_slots']} slots)")


global_ue_state = GlobalUEState()


# =====================================================================
# RLC Callback — uses rb.txsdu_wt_us (confirmed in attri.txt)
# =====================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        try:
            if len(ind.rb_stats) > 0:
                # Reset all tracked UE buffers to 0 (stale data guard)
                for rnti in global_ue_state.states.keys():
                    global_ue_state.states[rnti]["buffer_bytes"] = 0
                    global_ue_state.states[rnti]["latency_us"]   = 0

                # Aggregate per RNTI across all Radio Bearers
                for rb in ind.rb_stats:
                    rnti = rb.rnti
                    global_ue_state.register_ue(rnti)

                    # Sum bytes from all bearers (SRBs + DRBs)
                    global_ue_state.states[rnti]["buffer_bytes"] += rb.txbuf_occ_bytes

                    # Track maximum HOL (head-of-line) waiting time across bearers
                    current_max = global_ue_state.states[rnti]["latency_us"]
                    global_ue_state.states[rnti]["latency_us"] = max(current_max, rb.txsdu_wt_us)

        except Exception as e:
            print(f"\n[PYTHON RLC ERROR] {e}")
            traceback.print_exc()


# =====================================================================
# MCS-index → Spectral Efficiency (3GPP TS 38.214 Table 5.1.3.1-2)
# Used as fallback when CQI is 0 but MCS is valid.
# =====================================================================
MCS_TO_SE = [
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953, 1.9141,  # 0-7
    2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547, 5.5547,  # 8-15
    5.5547, 6.2266, 6.9141, 7.4063, 7.4063, 7.4063, 7.4063, 7.4063,  # 16-23
    7.4063, 7.4063, 7.4063, 7.4063, 7.4063                            # 24-28
]


# =====================================================================
# MAC Callback & PF Scheduler Algorithm
#
# Classic Proportional Fair metric:
#   PF_metric(i) = instantaneous_rate(i) / avg_throughput(i)
#
# where:
#   instantaneous_rate(i) = tbs_per_rb(SE_i)  [bits/RB, current channel]
#   avg_throughput(i)     = EMA-filtered past throughput (bits/slot)
#
# All active UEs are ranked by PF_metric in descending order and
# served greedily (highest-metric first) with a demand-capped fair-share
# allocation, identical to the RB loop used in LAPF_Enhanced.py.
#
# HARQ boost (+30 %) is retained so retransmissions are prioritised,
# consistent with the reference implementation.
# =====================================================================
class MACCallback(ric.mac_cb):
    def __init__(self):
        ric.mac_cb.__init__(self)

    def handle(self, ind):
        try:
            if len(ind.ue_stats) == 0:
                return

            t_now      = ind.tstamp
            active_ues = []

            # ── 1. Update MAC attributes ──────────────────────────────────
            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)

                # --- Channel quality: CQI → SE, fallback to MCS → SE -----
                try:
                    cqi_val = int(ue.wb_cqi)
                    mcs_val = int(ue.dl_mcs1)

                    if cqi_val > 0:
                        se = CQI_TO_SE[min(cqi_val, 15)]
                        global_ue_state.states[rnti]["cqi"] = cqi_val
                    elif mcs_val > 0:
                        se = MCS_TO_SE[min(mcs_val, 28)]
                        global_ue_state.states[rnti]["cqi"] = mcs_val
                    else:
                        se = 0.15
                        global_ue_state.states[rnti]["cqi"] = 0

                    global_ue_state.states[rnti]["se"] = se
                except Exception:
                    global_ue_state.states[rnti]["se"]  = 0.15
                    global_ue_state.states[rnti]["cqi"] = 0

                # --- HARQ pending flag ------------------------------------
                try:
                    if isinstance(ue.dl_harq, (list, tuple)):
                        global_ue_state.states[rnti]["harq_pending"] = sum(ue.dl_harq) > 0
                    else:
                        global_ue_state.states[rnti]["harq_pending"] = int(ue.dl_harq) > 0
                except Exception:
                    global_ue_state.states[rnti]["harq_pending"] = False

                if global_ue_state.states[rnti]["buffer_bytes"] > 0:
                    active_ues.append(rnti)

            if not active_ues:
                return

            # ── 2. PF Metric Computation ──────────────────────────────────
            # PF_metric(i) = instantaneous_rate_per_RB(i) / avg_throughput(i)
            # HARQ-pending UEs receive a 1.3× boost to prioritise
            # retransmissions (consistent with LAPF reference).
            pf_metrics = {}

            for rnti in active_ues:
                st = global_ue_state.states[rnti]

                # Instantaneous achievable rate for one RB (bits)
                inst_rate = tbs_per_rb(st["se"])

                # Core PF metric
                metric = inst_rate / max(st["avg_throughput"], 1e-6)

                # HARQ boost
                if st["harq_pending"]:
                    metric *= 1.3

                pf_metrics[rnti] = metric

            # ── 3. PF Scheduling: greedy highest-metric-first ─────────────
            # UEs are sorted globally (no priority floors — pure PF).
            # RB allocation mirrors the LAPF fair-share loop exactly:
            #   demand  = ceil(bits_needed / tbs_per_rb(se))
            #   n_left  = remaining UEs with demand > 0
            #   granted = min(demand, ceil(RBavail / n_left), RBavail)
            sorted_ues = sorted(active_ues,
                                key=lambda x: pf_metrics[x],
                                reverse=True)

            # Compute per-UE RB demand
            demands = {}
            for u in sorted_ues:
                st          = global_ue_state.states[u]
                bits_needed = st["buffer_bytes"] * 8
                rbs_demanded = math.ceil(bits_needed / max(tbs_per_rb(st["se"]), 1e-6))
                demands[u]  = min(rbs_demanded, TOTAL_RBS)

            available_rbs   = TOTAL_RBS
            pf_allocations  = {}

            for idx_u, u in enumerate(sorted_ues):
                if available_rbs <= 0:
                    break
                if demands[u] <= 0:
                    continue

                # Remaining UEs (from current position onward) with demand > 0
                n_left     = sum(1 for x in sorted_ues[idx_u:] if demands[x] > 0)
                n_left     = max(n_left, 1)
                fair_share = math.ceil(available_rbs / n_left)

                granted_rbs = min(demands[u], fair_share, available_rbs)
                served_bits = granted_rbs * tbs_per_rb(global_ue_state.states[u]["se"])

                pf_allocations[u] = {
                    "rbs":         granted_rbs,
                    "metric":      pf_metrics[u],
                    "served_bits": served_bits
                }

                available_rbs -= granted_rbs
                demands[u]     = 0   # mark as served

                if granted_rbs > 0:
                    st = global_ue_state.states[u]
                    print(f"[PF] tstamp: {t_now} | UE: {u} ({st['label']}) | "
                          f"CQI/MCS: {st['cqi']} | RBs: {granted_rbs} | "
                          f"PF_Metric: {pf_metrics[u]:.2f}")

            # ── 4. Throughput evaluation & EMA update ─────────────────────
            # EMA applied to ALL active UEs (including those with 0 RBs) so
            # avg_throughput decays for unserved UEs, raising their PF term
            # next slot.
            log_rows = []

            for rnti in active_ues:
                st    = global_ue_state.states[rnti]
                alloc = pf_allocations.get(rnti,
                                           {"rbs": 0, "metric": 0.0, "served_bits": 0.0})

                lat_ms  = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]

                # URLLC latency penalty: if packet already exceeded its PDB,
                # count it as lost (effective throughput = 0).
                if st["type"] == "URLLC" and lat_ms > st["pdb_ms"]:
                    effective_tbs = 0.0
                else:
                    effective_tbs = raw_tbs

                # Throughput in Mbps over one 0.5 ms slot
                throughput_mbps = (effective_tbs / 1e6) / SLOT_DURATION_S

                # EMA update
                st["avg_throughput"] = ((1 - ALPHA_EMA) * st["avg_throughput"]
                                        + ALPHA_EMA * effective_tbs)

                log_rows.append([
                    t_now, rnti, st["type"], st["5qi"], st["priority"],
                    st["cqi"], st["buffer_bytes"], round(lat_ms, 2), st["pdb_ms"],
                    round(alloc["metric"], 4), alloc["rbs"],
                    round(raw_tbs, 0), round(throughput_mbps, 4)
                ])

            if log_rows:
                with open(global_ue_state.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(log_rows)

        except Exception as e:
            print(f"\n[PYTHON MAC ERROR] {e}")
            traceback.print_exc()


# =====================================================================
# Main Initialization Loop
# =====================================================================
ric.init()
conn = ric.conn_e2_nodes()
assert len(conn) > 0, "Error: No E2 nodes connected!"

mac_hndlr = []
rlc_hndlr = []

for i in range(len(conn)):
    mac_cb = MACCallback()
    rlc_cb = RLCCallback()

    hndlr_rlc = ric.report_rlc_sm(conn[i].id, ric.Interval_ms_10, rlc_cb)
    hndlr_mac = ric.report_mac_sm(conn[i].id, ric.Interval_ms_10, mac_cb)

    rlc_hndlr.append(hndlr_rlc)
    mac_hndlr.append(hndlr_mac)
    time.sleep(1)

print(f"PF Scheduler xApp running. Logging to {global_ue_state.csv_file}...")
print("Press Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Ctrl+C detected. Shutting down gracefully...")
finally:
    print("[INFO] Cleaning up MAC and RLC reports...")
    for i in range(len(mac_hndlr)):
        ric.rm_report_mac_sm(mac_hndlr[i])
        ric.rm_report_rlc_sm(rlc_hndlr[i])
    print("[INFO] xApp successfully stopped.")
