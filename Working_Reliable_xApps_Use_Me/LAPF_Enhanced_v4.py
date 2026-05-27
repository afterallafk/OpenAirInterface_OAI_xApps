import xapp_sdk as ric
import time
import csv
import math
import os
import traceback

# =====================================================================
# SDAP Layer & 5G QoS Configuration (10 UEs)
# 6 URLLC, 2 eMBB, 2 mMTC
# =====================================================================
QOS_MAPPING = {
    0: {"type": "URLLC", "label": "URLLC_1", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    1: {"type": "eMBB",  "label": "eMBB_1",  "5qi": 9,  "priority": 90, "pdb_ms": 300.0},
    2: {"type": "mMTC",  "label": "mMTC_1",  "5qi": 70, "priority": 55, "pdb_ms": 200.0},
    3: {"type": "URLLC", "label": "URLLC_2", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    4: {"type": "eMBB",  "label": "eMBB_2",  "5qi": 9,  "priority": 90, "pdb_ms": 300.0},
    5: {"type": "mMTC",  "label": "mMTC_2",  "5qi": 70, "priority": 55, "pdb_ms": 200.0},
    6: {"type": "URLLC", "label": "URLLC_3", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    7: {"type": "URLLC", "label": "URLLC_4", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    8: {"type": "URLLC", "label": "URLLC_5", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
    9: {"type": "URLLC", "label": "URLLC_6", "5qi": 86, "priority": 18, "pdb_ms": 5.0}
}

# =====================================================================
# Constants
# =====================================================================
SLOT_DURATION_MS = 0.5
SLOT_DURATION_S  = 0.5e-3
TOTAL_RBS        = 106
ALPHA_EMA        = 0.08
EPSILON          = 1e-6

# =====================================================================
# FIX: Correct MCS → CQI lookup table
# -----------------------------------------------------------------------
# Derived from 3GPP TS 38.214:
#   MCS table : Table 5.1.3.2-1  (64QAM, indices 0-28)
#   CQI table : Table 5.1.3.1-1  (CQI indices 1-15)
#
# Method: for each MCS index compute SE = Qm × (R/1024), then select
# the CQI index with the closest SE value (min |SE_cqi - SE_mcs|).
#
# Previous heuristic (mcs+1, mcs+2, mcs-1 …) was incorrect for 27 out
# of 29 MCS indices.  Example errors:
#   MCS  5 → old heuristic gave CQI 7,  correct is CQI 3
#   MCS 18 → old heuristic gave CQI 15, correct is CQI 8
#   MCS 28 → old heuristic gave CQI 15, correct is CQI 13
#
# MCS_TO_CQI[mcs_index] → cqi_index  (mcs: 0..28, cqi: 1..13)
# CQI 14/15 share the same SE as CQI 13 in Table 5.1.3.1-1; the LUT
# caps at 13 to avoid ambiguity.
#
# NOTE: This LUT is used ONLY for CSV logging of the CQI column when
# wb_cqi == 0 (not yet reported on this PUCCH cycle).  The RB allocator
# path uses se_bits_per_rb = dl_curr_tbs / dl_sched_rb, which comes
# directly from OAI's internal MCS/CQI/MIMO pipeline and is independent
# of this table.
# =====================================================================
MCS_TO_CQI = [
    1,   # MCS  0  SE=0.2344  → CQI  1 (SE=0.1523, closest)
    2,   # MCS  1  SE=0.3066  → CQI  2 (SE=0.3770)
    2,   # MCS  2  SE=0.3770  → CQI  2 (SE=0.3770, exact)
    2,   # MCS  3  SE=0.4902  → CQI  2 (SE=0.3770, closest)
    2,   # MCS  4  SE=0.6016  → CQI  2 (SE=0.3770, closest)
    3,   # MCS  5  SE=0.7402  → CQI  3 (SE=0.8770, closest)
    3,   # MCS  6  SE=0.8770  → CQI  3 (SE=0.8770, exact)
    4,   # MCS  7  SE=1.0273  → CQI  4 (SE=1.1758, closest)
    4,   # MCS  8  SE=1.1758  → CQI  4 (SE=1.1758, exact)
    4,   # MCS  9  SE=1.3262  → CQI  4 (SE=1.1758, closest)
    5,   # MCS 10  SE=1.3281  → CQI  5 (SE=1.4766, closest)
    5,   # MCS 11  SE=1.4766  → CQI  5 (SE=1.4766, exact)
    5,   # MCS 12  SE=1.6953  → CQI  5 (SE=1.4766, closest)
    6,   # MCS 13  SE=1.9141  → CQI  6 (SE=1.9141, exact)
    6,   # MCS 14  SE=2.1602  → CQI  6 (SE=1.9141, closest)
    7,   # MCS 15  SE=2.4063  → CQI  7 (SE=2.4063, exact)
    8,   # MCS 16  SE=2.5703  → CQI  8 (SE=2.7305, closest)
    7,   # MCS 17  SE=2.5664  → CQI  7 (SE=2.4063, closest)
    8,   # MCS 18  SE=2.7305  → CQI  8 (SE=2.7305, exact)
    9,   # MCS 19  SE=3.0293  → CQI  9 (SE=3.3223, closest)
    9,   # MCS 20  SE=3.3223  → CQI  9 (SE=3.3223, exact)
    9,   # MCS 21  SE=3.6094  → CQI  9 (SE=3.3223, closest)
    10,  # MCS 22  SE=3.9023  → CQI 10 (SE=3.9023, exact)
    11,  # MCS 23  SE=4.2129  → CQI 11 (SE=4.5234, closest)
    11,  # MCS 24  SE=4.5234  → CQI 11 (SE=4.5234, exact)
    11,  # MCS 25  SE=4.8164  → CQI 11 (SE=4.5234, closest)
    12,  # MCS 26  SE=5.1152  → CQI 12 (SE=5.1152, exact)
    12,  # MCS 27  SE=5.3320  → CQI 12 (SE=5.1152, closest)
    13,  # MCS 28  SE=5.5547  → CQI 13 (SE=5.5547, exact)
]


class GlobalUEState:
    def __init__(self):
        self.states = {}
        self.rnti_order = []
        self.csv_file = "lapf_results.csv"

        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "tstamp", "RNTI", "TrafficType", "5QI", "Priority",
                "CQI", "Buffer_Bytes", "Latency_ms", "Deadline_ms",
                "Metric", "RBs_Granted", "Served_Bits", "Throughput_Mbps"
            ])

    def register_ue(self, rnti):
        if rnti not in self.states:
            idx = len(self.rnti_order) % len(QOS_MAPPING)
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
                "avg_throughput": EPSILON,
                "cqi":            0,
                "se_bits_per_rb": 0.0,
                "harq_pending":   False,
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} as {qos['label']} "
                  f"(5QI: {qos['5qi']}, deadline: {self.states[rnti]['deadline_slots']} slots)")


global_ue_state = GlobalUEState()


# =====================================================================
# RLC Callback — buffer occupancy and head-of-line latency
# =====================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        try:
            if len(ind.rb_stats) > 0:
                for rnti in global_ue_state.states.keys():
                    global_ue_state.states[rnti]["buffer_bytes"] = 0
                    global_ue_state.states[rnti]["latency_us"]   = 0

                for rb in ind.rb_stats:
                    rnti = rb.rnti
                    global_ue_state.register_ue(rnti)

                    global_ue_state.states[rnti]["buffer_bytes"] += rb.txbuf_occ_bytes

                    current_max = global_ue_state.states[rnti]["latency_us"]
                    global_ue_state.states[rnti]["latency_us"] = max(current_max, rb.txsdu_wt_us)

        except Exception as e:
            print(f"\n[RLC ERROR] {e}")
            traceback.print_exc()


# =====================================================================
# MAC Callback — LAPF Scheduler
# =====================================================================
class MACCallback(ric.mac_cb):
    def __init__(self):
        ric.mac_cb.__init__(self)

    def handle(self, ind):
        try:
            if len(ind.ue_stats) == 0:
                return

            t_now = ind.tstamp
            active_ues = []

            # ── 1. Read OAI MAC attributes ────────────────────────────────
            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]

                # ── se_bits_per_rb: persist across slots ───────────────────
                try:
                    tbs_now   = int(ue.dl_curr_tbs)
                    sched_rbs = int(ue.dl_sched_rb)
                    if tbs_now > 0 and sched_rbs > 0:
                        st["se_bits_per_rb"] = tbs_now / sched_rbs
                    # else: retain previous se_bits_per_rb
                except Exception:
                    pass

                # ── CQI: wb_cqi primary, MCS_TO_CQI LUT fallback ──────────
                #
                # wb_cqi (TS 38.214 Table 5.1.3.1-1, indices 0-15):
                #   Reported on the PUCCH measurement cycle, not every MAC
                #   slot.  Value is 0 between reporting cycles.
                #   When non-zero, use directly (valid range 1-15).
                #
                # dl_mcs1 fallback (TS 38.214 Table 5.1.3.2-1, indices 0-28):
                #   When wb_cqi == 0, derive CQI from dl_mcs1 via the
                #   MCS_TO_CQI lookup table (3GPP SE-matched, corrected).
                #   Valid MCS range is 0-28; values outside this range
                #   indicate no active DL grant and are skipped.
                #
                # In both cases the result is stored only when a valid
                # non-zero value is obtained, preserving the last known
                # CQI across slots where neither source is available.
                try:
                    cqi_now = int(ue.wb_cqi)
                    if cqi_now > 0:
                        # wb_cqi is valid this cycle — use directly
                        st["cqi"] = min(max(cqi_now, 1), 15)
                    else:
                        # wb_cqi not yet reported — fall back to dl_mcs1
                        mcs = int(ue.dl_mcs1)
                        if 0 <= mcs <= 28:          # valid MCS range only
                            st["cqi"] = MCS_TO_CQI[mcs]
                        # else: mcs < 0 or > 28 means no active grant; retain
                        #       previous cqi unchanged
                except Exception:
                    pass   # retain previous cqi on any read error

                try:
                    if isinstance(ue.dl_harq, (list, tuple)):
                        st["harq_pending"] = sum(ue.dl_harq) > 0
                    else:
                        st["harq_pending"] = int(ue.dl_harq) > 0
                except Exception:
                    st["harq_pending"] = False

                if st["buffer_bytes"] > 0:
                    active_ues.append(rnti)

            if not active_ues:
                return

            # ── 2. Group by SDAP Priority Floors ─────────────────────────
            priority_floors = {}
            for rnti in active_ues:
                prio = global_ue_state.states[rnti]["priority"]
                if prio not in priority_floors:
                    priority_floors[prio] = []
                priority_floors[prio].append(rnti)

            sorted_priorities = sorted(priority_floors.keys())
            available_rbs     = TOTAL_RBS
            lapf_allocations  = {}

            # ── 3. Floor-by-Floor Scheduling ─────────────────────────────
            for prio in sorted_priorities:
                if available_rbs <= 0:
                    break

                floor_ues = priority_floors[prio]
                qmax = max(global_ue_state.states[u]["buffer_bytes"] for u in floor_ues) + EPSILON
                metrics = {}

                for u in floor_ues:
                    st = global_ue_state.states[u]

                    if st["se_bits_per_rb"] <= 0.0:
                        metrics[u] = -1.0
                        continue

                    inst_rate_per_rb = st["se_bits_per_rb"]
                    pf_term  = inst_rate_per_rb / max(st["avg_throughput"], EPSILON)
                    qnorm    = st["buffer_bytes"] / qmax
                    lat_slots      = (st["latency_us"] / 1000.0) / SLOT_DURATION_MS
                    deadline_slots = st["deadline_slots"]
                    urgency        = 1.0 / max(deadline_slots - lat_slots, 1e-3)
                    metric         = pf_term * qnorm * urgency

                    if st["harq_pending"]:
                        metric *= 1.3

                    metrics[u] = metric

                sorted_floor_ues = [
                    u for u in sorted(floor_ues, key=lambda x: metrics[x], reverse=True)
                    if metrics[u] >= 0.0
                ]

                demands = {}
                for u in sorted_floor_ues:
                    st               = global_ue_state.states[u]
                    inst_rate_per_rb = st["se_bits_per_rb"]
                    bits_needed      = st["buffer_bytes"] * 8
                    rbs_demanded     = math.ceil(bits_needed / inst_rate_per_rb)
                    demands[u]       = min(rbs_demanded, available_rbs)

                rb_avail_floor = available_rbs

                for idx_u, u in enumerate(sorted_floor_ues):
                    if rb_avail_floor <= 0:
                        break
                    if demands[u] <= 0:
                        continue

                    n_left     = sum(1 for x in sorted_floor_ues[idx_u:] if demands[x] > 0)
                    n_left     = max(n_left, 1)
                    fair_share = math.ceil(rb_avail_floor / n_left)

                    granted_rbs = min(demands[u], fair_share, rb_avail_floor)

                    st               = global_ue_state.states[u]
                    inst_rate_per_rb = st["se_bits_per_rb"]

                    if inst_rate_per_rb <= 0.0:
                        demands[u] = 0
                        continue

                    served_bits = granted_rbs * inst_rate_per_rb

                    lapf_allocations[u] = {
                        "rbs":         granted_rbs,
                        "metric":      metrics[u],
                        "served_bits": served_bits,
                    }

                    rb_avail_floor -= granted_rbs
                    demands[u]      = 0

                    if granted_rbs > 0:
                        print(f"[LAPF] tstamp: {t_now} | UE: {u} ({st['label']}) | "
                              f"CQI: {st['cqi']} | RBs: {granted_rbs} | "
                              f"Metric: {metrics[u]:.2f}")

                available_rbs = rb_avail_floor

            # ── 4. Throughput evaluation & EMA update ─────────────────────
            log_rows = []

            for rnti in active_ues:
                st    = global_ue_state.states[rnti]
                alloc = lapf_allocations.get(rnti, {"rbs": 0, "metric": 0.0, "served_bits": 0.0})

                lat_ms  = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]

                if st["type"] == "URLLC" and lat_ms > st["pdb_ms"]:
                    effective_tbs = 0.0
                else:
                    effective_tbs = raw_tbs

                throughput_mbps = (effective_tbs / 1e6) / SLOT_DURATION_S

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
            print(f"\n[MAC ERROR] {e}")
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

print(f"LAPF Scheduler xApp running. Logging to {global_ue_state.csv_file}...")
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
    print("[INFO] xApp stopped.")
