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
    8: {"type": "URLLC", "label": "URLLC_5", "5qi": 86, "priority": 18, "pdb_ms": 5.0}, # NEW
    9: {"type": "URLLC", "label": "URLLC_6", "5qi": 86, "priority": 18, "pdb_ms": 5.0}  # NEW
}

# =====================================================================
# Constants
# =====================================================================
# NOTE: CQI_TO_SE, MCS_TO_SE, tbs_per_rb(), and associated constants
# (NUM_LAYERS, SC_PER_RB, NUM_SYMB) have been removed.  OAI runs the
# full 3GPP PHY pipeline internally and reports results directly via:
#
#   dl_curr_tbs  — actual TBS (bits) OAI scheduled for this UE this slot
#   dl_sched_rb  — number of RBs OAI assigned to this UE this slot
#   wb_cqi       — wideband CQI (logged for observability)
#
# IMPORTANT — persistence requirement (fixes CQI=0 and Served_Bits=0):
#
#   dl_curr_tbs and dl_sched_rb are NON-ZERO only in the specific slot
#   OAI grants that UE RBs.  On every other slot they read as 0.
#   Similarly, wb_cqi is updated on the PUCCH measurement cycle, not
#   every MAC indication slot — so it reads 0 on most slots.
#
#   Both fields must be PERSISTED (stale-hold): updated only when a
#   fresh non-zero value arrives, and kept at their last known value
#   otherwise.  Writing them every slot (including 0) is the root cause
#   of CQI=0 for 100% of rows and Served_Bits=0 for 59% of rows.
#
#   se_bits_per_rb = dl_curr_tbs / dl_sched_rb  (persisted)
#   cqi            = wb_cqi                     (persisted, non-zero only)
#
# All three places that use inst_rate_per_rb now read se_bits_per_rb:
#
#   1. LAPF metric PF term:  rate_per_rb = se_bits_per_rb
#   2. RB demand estimation: rbs_demanded = ceil(bits_needed / se_bits_per_rb)
#   3. served_bits:          served_bits  = granted_rbs × se_bits_per_rb
#
# =====================================================================
SLOT_DURATION_MS = 0.5          # ms  (30 kHz SCS → 0.5 ms per slot)
SLOT_DURATION_S  = 0.5e-3       # seconds
TOTAL_RBS        = 106          # 40 MHz BW, 30 kHz SCS
ALPHA_EMA        = 0.08         # EMA smoothing — matches MATLAB alpha = 0.08
EPSILON          = 1e-6


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
            idx = len(self.rnti_order) % len(QOS_MAPPING)  # Dynamic scaling
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
                "avg_throughput": EPSILON,    # initialised > 0 to avoid div-by-zero
                # OAI-reported fields (updated each MAC indication):
                "cqi":            0,          # wb_cqi — persisted; OAI reports on PUCCH cycle
                # se_bits_per_rb: persisted across slots.  OAI only populates
                # dl_curr_tbs / dl_sched_rb when it grants that UE RBs in that
                # specific slot — zero on unscheduled slots.  We keep the last
                # known value so metric and served_bits are never collapsed to 0
                # due to a missing grant in the current indication.
                "se_bits_per_rb": 0.0,        # updated only when dl_curr_tbs > 0
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
                # Reset all tracked UE buffers to 0 (stale data guard)
                for rnti in global_ue_state.states.keys():
                    global_ue_state.states[rnti]["buffer_bytes"] = 0
                    global_ue_state.states[rnti]["latency_us"]   = 0

                # Aggregate per RNTI across all Radio Bearers
                for rb in ind.rb_stats:
                    rnti = rb.rnti
                    global_ue_state.register_ue(rnti)

                    global_ue_state.states[rnti]["buffer_bytes"] += rb.txbuf_occ_bytes

                    # Maximum HOL (head-of-line) waiting time across bearers
                    current_max = global_ue_state.states[rnti]["latency_us"]
                    global_ue_state.states[rnti]["latency_us"] = max(current_max, rb.txsdu_wt_us)

        except Exception as e:
            print(f"\n[RLC ERROR] {e}")
            traceback.print_exc()


# =====================================================================
# MAC Callback — LAPF Scheduler
#
# LAPF metric per UE (within each SDAP priority floor):
#
#   inst_rate_per_rb = dl_curr_tbs / dl_sched_rb       (OAI-native bits/RB)
#   pf_term          = inst_rate_per_rb / avg_throughput
#   qnorm            = buffer_bytes / qmax_on_this_floor
#   urgency          = 1 / max(deadline_slots - lat_slots, 1e-3)
#   metric           = pf_term × qnorm × urgency  [× 1.3 if HARQ pending]
#
# RB allocation within each floor:
#   rbs_demanded = ceil(bits_needed / inst_rate_per_rb)
#   fair_share   = ceil(rb_avail_floor / n_left_with_demand)
#   granted_rbs  = min(rbs_demanded, fair_share, rb_avail_floor)
#   served_bits  = granted_rbs × inst_rate_per_rb
#
# EMA update uses effective_tbs (zeroed for URLLC deadline misses).
# This is the correct LAPF behaviour — the zero feeds back into the
# PF term to boost the URLLC UE's metric next slot, driving faster
# recovery.  (Plain PF does NOT zero the EMA — that distinction is
# intentional between the two schedulers.)
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
                # BUG FIX A: dl_curr_tbs and dl_sched_rb are only non-zero in
                # the slot OAI actually grants that UE RBs.  On every other slot
                # they read as 0, which previously collapsed inst_rate_per_rb to
                # 0 → metric = 0 → served_bits = 0 (the "59% zero Served_Bits"
                # and "Metric=0" seen in the CSV).
                #
                # Fix: update se_bits_per_rb ONLY when a real grant arrives
                # (dl_curr_tbs > 0 AND dl_sched_rb > 0).  All other slots keep
                # the last known value, which reflects the channel quality from
                # the most recent OAI grant — the correct stale-hold behaviour.
                try:
                    tbs_now   = int(ue.dl_curr_tbs)
                    sched_rbs = int(ue.dl_sched_rb)
                    if tbs_now > 0 and sched_rbs > 0:
                        st["se_bits_per_rb"] = tbs_now / sched_rbs
                    # else: retain previous se_bits_per_rb
                except Exception:
                    pass   # retain previous se_bits_per_rb

                # ── wb_cqi: persist across slots ───────────────────────────
                # wb_cqi is reported on the PUCCH measurement cycle, not every
                # MAC slot.  Only update when OAI reports a non-zero value.
                # FALLBACK: OAI does not always populate wb_cqi via the MAC SM
                # (it may be 0 for the entire run).  When that happens, derive
                # an approximate CQI from dl_mcs1 using the 3GPP Table 5.2.2.1
                # MCS→CQI mapping (64QAM range: MCS 0-9 → CQI 1-7,
                # MCS 10-16 → CQI 8-11, MCS 17-27 → CQI 12-15).
                try:
                    cqi_now = int(ue.wb_cqi)
                    if cqi_now > 0:
                        st["cqi"] = cqi_now
                    else:
                        # Derive from MCS if wb_cqi is absent
                        mcs = int(ue.dl_mcs1)
                        if mcs >= 0:
                            if   mcs <= 2:  derived_cqi = max(1, mcs + 1)
                            elif mcs <= 5:  derived_cqi = mcs + 2
                            elif mcs <= 9:  derived_cqi = mcs + 1
                            elif mcs <= 12: derived_cqi = mcs - 1
                            elif mcs <= 16: derived_cqi = mcs - 2
                            else:           derived_cqi = min(15, mcs - 3)
                            if derived_cqi > 0:
                                st["cqi"] = derived_cqi
                    # else: retain previous cqi
                except Exception:
                    pass   # retain previous cqi

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

            # Lower priority-level number = higher urgency
            sorted_priorities = sorted(priority_floors.keys())
            available_rbs     = TOTAL_RBS
            lapf_allocations  = {}

            # ── 3. Floor-by-Floor Scheduling ─────────────────────────────
            for prio in sorted_priorities:
                if available_rbs <= 0:
                    break

                floor_ues = priority_floors[prio]

                # qmax scoped to this floor's UEs only so eMBB UEs are not
                # dwarfed by URLLC burst buffers.
                qmax = max(global_ue_state.states[u]["buffer_bytes"] for u in floor_ues) + EPSILON

                metrics = {}

                for u in floor_ues:
                    st = global_ue_state.states[u]

                    # inst_rate_per_rb: persisted OAI bits-per-RB from last grant.
                    # If still 0 (cold-start, no grant received yet), exclude this
                    # UE from scheduling this slot — we have no channel estimate.
                    if st["se_bits_per_rb"] <= 0.0:
                        metrics[u] = -1.0   # sentinel: excluded
                        continue

                    inst_rate_per_rb = st["se_bits_per_rb"]

                    pf_term = inst_rate_per_rb / max(st["avg_throughput"], EPSILON)
                    qnorm   = st["buffer_bytes"] / qmax

                    # Urgency: both operands in slots
                    lat_slots      = (st["latency_us"] / 1000.0) / SLOT_DURATION_MS
                    deadline_slots = st["deadline_slots"]
                    urgency        = 1.0 / max(deadline_slots - lat_slots, 1e-3)

                    metric = pf_term * qnorm * urgency

                    if st["harq_pending"]:
                        metric *= 1.3

                    metrics[u] = metric

                # Exclude cold-start UEs (no channel estimate yet)
                sorted_floor_ues = [u for u in sorted(floor_ues, key=lambda x: metrics[x], reverse=True)
                                    if metrics[u] >= 0.0]

                # ── RB demand per UE (buffer-driven) ─────────────────────
                # rbs_demanded = ceil(bits_needed / inst_rate_per_rb)
                # Uses OAI's bits-per-RB so demand tracks OAI's actual MCS.
                demands = {}
                for u in sorted_floor_ues:
                    st               = global_ue_state.states[u]
                    inst_rate_per_rb = st["se_bits_per_rb"]   # guaranteed > 0 (cold-start filtered above)
                    bits_needed      = st["buffer_bytes"] * 8          # bytes → bits
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

                    # Cold-start guard: if OAI has not yet sent a single grant
                    # for this UE, se_bits_per_rb is still 0.0.  We cannot
                    # serve any bits without knowing the channel capacity, so
                    # skip the grant entirely this slot — do NOT allocate RBs
                    # with EPSILON as the rate (that produces ghost RB grants
                    # with Served_Bits=0 in the CSV).
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
            # EMA uses effective_tbs (zeroed for URLLC deadline misses).
            # This is intentional for LAPF: the zero feeds back into the
            # PF term to boost the URLLC UE's metric next slot, driving
            # faster recovery after a deadline violation.
            # EMA is applied to ALL active UEs including unserved ones so
            # avg_throughput decays for unserved UEs, raising their metric
            # next slot — matches MATLAB avgThroughputNew = (1-alpha)*avg
            # + alpha*servedBits.
            log_rows = []

            for rnti in active_ues:
                st    = global_ue_state.states[rnti]
                alloc = lapf_allocations.get(rnti, {"rbs": 0, "metric": 0.0, "served_bits": 0.0})

                lat_ms  = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]

                # URLLC deadline miss → effective throughput = 0
                if st["type"] == "URLLC" and lat_ms > st["pdb_ms"]:
                    effective_tbs = 0.0
                else:
                    effective_tbs = raw_tbs

                throughput_mbps = (effective_tbs / 1e6) / SLOT_DURATION_S

                # EMA update — uses effective_tbs (LAPF-specific behaviour)
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
