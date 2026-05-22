import xapp_sdk as ric
import time
import csv
import math
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
    7: {"type": "URLLC", "label": "URLLC_4", "5qi": 86, "priority": 18, "pdb_ms": 5.0},
}

# =====================================================================
# Constants
# =====================================================================
# NOTE: CQI/MCS tables, SE lookups, and TBS approximation formulas have
# been removed entirely.  OAI already runs the full 3GPP PHY pipeline
# internally (CQI measurement → MCS selection → TBS computation) and
# exposes the results directly via the MAC indication attributes:
#
#   dl_curr_tbs  — actual TBS (bits) OAI scheduled for this UE this slot
#   dl_sched_rb  — number of RBs OAI assigned to this UE this slot
#   wb_cqi       — wideband CQI reported by OAI (logged only, not used
#                  for SE computation — avoids the CQI>15 / MCS confusion)
#
# The PF metric is computed as:
#   inst_rate_per_rb = dl_curr_tbs / max(dl_sched_rb, 1)   (bits / RB)
#   pf_metric        = inst_rate_per_rb / max(avg_throughput, EPSILON)
#
# This is equivalent to MATLAB's nrTBS(…,1,…) / avgThroughput but uses
# OAI's own TBS value rather than a re-implemented approximation.
# =====================================================================
SLOT_DURATION_MS = 0.5
SLOT_DURATION_S  = 0.5e-3
TOTAL_RBS        = 106       # 40 MHz, 30 kHz SCS
ALPHA_EMA        = 0.08      # EMA decay — matches MATLAB pfScheduler alpha=0.08
EPSILON          = 1e-6


# =====================================================================
# Global UE State
# =====================================================================
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
                "type":            qos["type"],
                "label":           qos["label"],
                "5qi":             qos["5qi"],
                "priority":        qos["priority"],
                "pdb_ms":          qos["pdb_ms"],
                "deadline_slots":  math.ceil(qos["pdb_ms"] / SLOT_DURATION_MS),
                "buffer_bytes":    0,
                "latency_us":      0,
                "avg_throughput":  EPSILON,   # initialise to EPSILON not 1e-6 literal
                # OAI-reported fields (updated each MAC indication):
                "cqi":             0,         # wb_cqi — persisted (non-zero only)
                # se_bits_per_rb: persisted across slots.  OAI only populates
                # dl_curr_tbs / dl_sched_rb in the slot it grants that UE RBs;
                # on every other slot both read as 0.  We stale-hold the last
                # known value so the PF metric and served_bits are never
                # collapsed to 0 on unscheduled slots.
                "se_bits_per_rb":  0.0,       # updated only when dl_curr_tbs > 0
                "harq_pending":    False,
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} as {qos['label']} "
                  f"(5QI: {qos['5qi']}, deadline: {self.states[rnti]['deadline_slots']} slots)")


global_ue_state = GlobalUEState()


# =====================================================================
# RLC Callback — buffer status and per-packet latency
# =====================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        try:
            if len(ind.rb_stats) == 0:
                return

            for rnti in global_ue_state.states:
                global_ue_state.states[rnti]["buffer_bytes"] = 0
                global_ue_state.states[rnti]["latency_us"]   = 0

            for rb in ind.rb_stats:
                rnti = rb.rnti
                global_ue_state.register_ue(rnti)
                global_ue_state.states[rnti]["buffer_bytes"] += rb.txbuf_occ_bytes
                cur = global_ue_state.states[rnti]["latency_us"]
                global_ue_state.states[rnti]["latency_us"] = max(cur, rb.txsdu_wt_us)

        except Exception as e:
            print(f"\n[RLC ERROR] {e}")
            traceback.print_exc()


# =====================================================================
# MAC Callback — PF Scheduler
#
# Key design decisions:
#
#   1. OAI-native TBS/RB — no reimplemented CQI/MCS/SE tables
#      dl_curr_tbs and dl_sched_rb are read directly from the MAC
#      indication.  OAI has already run its internal 3GPP tables to
#      produce these values, so there is no risk of CQI-range mismatch
#      (e.g. wb_cqi > 15 caused the EMA runaway in the previous version).
#
#   2. PF metric uses bits-per-RB as inst_rate
#      inst_rate_per_rb = dl_curr_tbs / max(dl_sched_rb, 1)
#      This is the per-RB rate OAI actually achieved — equivalent to
#      MATLAB's nrTBS(moduse, layers, 1, 14, tcr) used in pfMetric().
#
#   3. HARQ boost ×1.3 preserved (MATLAB line 1324–1325)
#
#   4. Multi-UE contiguous RB allocation — Resource Allocation Type 1
#      (3GPP TS 38.214): UEs sorted by PF metric descending; each gets
#      floor(available_rbs / n_remaining) RBs.
#
#   5. served_bits for the EMA uses inst_rate_per_rb × grant_rbs so
#      the EMA tracks what the scheduler intends to deliver, consistent
#      with how MATLAB computes avgThroughput (granted, not ACKed).
#
#   6. Reported throughput (CSV) applies URLLC deadline zeroing:
#      packets arriving after pdb_ms count as 0 Mbps delivered.
#      eMBB/mMTC throughput is always the raw served_bits rate.
#      The EMA always uses raw served_bits (no deadline zeroing) to
#      avoid distorting the PF metric — that zeroing belongs to LAPF.
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

            # ── 1. Read OAI MAC attributes ────────────────────────────────
            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]

                # ── se_bits_per_rb: persist across slots ───────────────────
                # dl_curr_tbs and dl_sched_rb are non-zero ONLY in the slot
                # OAI grants that UE RBs.  On all other slots they read 0.
                # Overwriting every slot collapses inst_rate_per_rb → 0 →
                # PF metric = 0 → served_bits = 0 on ~59% of rows.
                # Fix: update only when a real grant arrives (both > 0).
                try:
                    tbs_now   = int(ue.dl_curr_tbs)
                    sched_rbs = int(ue.dl_sched_rb)
                    if tbs_now > 0 and sched_rbs > 0:
                        st["se_bits_per_rb"] = tbs_now / sched_rbs
                    # else: retain previous se_bits_per_rb
                except Exception:
                    pass   # retain previous se_bits_per_rb

                # ── wb_cqi: persist across slots ───────────────────────────
                # wb_cqi is refreshed on the PUCCH cycle, not every MAC slot.
                # Writing it unconditionally overwrites valid values with 0.
                # Fix: update only when OAI reports a non-zero value;
                # fall back to MCS-derived CQI when wb_cqi is always 0.
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

                # HARQ pending flag
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

            # ── 2. PF Metric Computation ──────────────────────────────────
            # inst_rate_per_rb = se_bits_per_rb (persisted from last OAI grant)
            # pf_metric = inst_rate_per_rb / avg_throughput
            # UEs with se_bits_per_rb = 0 (cold-start, no grant yet) are
            # excluded from scheduling this slot to avoid ghost grants.
            pf_metrics = {}

            for rnti in active_ues:
                st = global_ue_state.states[rnti]

                if st["se_bits_per_rb"] <= 0.0:
                    pf_metrics[rnti] = -1.0   # sentinel: exclude from scheduling
                    continue

                inst_rate_per_rb = st["se_bits_per_rb"]
                metric = inst_rate_per_rb / max(st["avg_throughput"], EPSILON)

                if st["harq_pending"]:
                    metric *= 1.3   # MATLAB line 1324–1325

                pf_metrics[rnti] = metric

            # ── 3. PF Scheduling: multi-UE contiguous RB allocation ───────
            # Cold-start UEs (se_bits_per_rb = 0, metric = -1) are excluded.
            sorted_ues    = [u for u in sorted(active_ues,
                                               key=lambda x: pf_metrics[x],
                                               reverse=True)
                             if pf_metrics[u] >= 0.0]
            available_rbs = TOTAL_RBS
            pf_allocations = {}
            n_ues = len(sorted_ues)

            for idx, rnti in enumerate(sorted_ues):
                if available_rbs <= 0:
                    break

                n_remaining = n_ues - idx
                grant = max(1, available_rbs // n_remaining)
                grant = min(grant, available_rbs)

                st = global_ue_state.states[rnti]

                # served_bits: scale persisted bits-per-RB by granted RBs
                inst_rate_per_rb = st["se_bits_per_rb"]   # guaranteed > 0 here
                served_bits = inst_rate_per_rb * grant

                pf_allocations[rnti] = {
                    "rbs":         grant,
                    "metric":      pf_metrics[rnti],
                    "served_bits": served_bits,
                }

                available_rbs -= grant

                print(f"[PF] tstamp: {t_now} | UE: {rnti} ({st['label']}) | "
                      f"CQI: {st['cqi']} | Type: {st['type']} | "
                      f"RBs: {grant} | PF_Metric: {pf_metrics[rnti]:.4f}")

            # ── 4. Throughput EMA + CSV logging ───────────────────────────
            # EMA always uses raw served_bits (no deadline zeroing) so the
            # PF metric is not distorted by latency penalties.
            #
            # reported_tbs (CSV):
            #   URLLC  — 0 if latency > pdb_ms (late packet = useless)
            #   eMBB / mMTC — raw served_bits always
            log_rows = []

            for rnti in active_ues:
                st    = global_ue_state.states[rnti]
                alloc = pf_allocations.get(rnti, {"rbs": 0, "metric": 0.0, "served_bits": 0.0})

                lat_ms  = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]

                # EMA update — always raw_tbs, alpha=0.08
                st["avg_throughput"] = ((1 - ALPHA_EMA) * st["avg_throughput"]
                                        + ALPHA_EMA * raw_tbs)

                # Reported throughput: zero URLLC if deadline missed
                if st["type"] == "URLLC" and lat_ms > st["pdb_ms"]:
                    reported_tbs = 0.0
                else:
                    reported_tbs = raw_tbs

                throughput_mbps = (reported_tbs / 1e6) / SLOT_DURATION_S

                log_rows.append([
                    t_now, rnti, st["type"], st["5qi"], st["priority"],
                    st["cqi"], st["buffer_bytes"],
                    round(lat_ms, 2), st["pdb_ms"],
                    round(alloc["metric"], 6),
                    alloc["rbs"], round(reported_tbs, 0),
                    round(throughput_mbps, 4),
                ])

            if log_rows:
                with open(global_ue_state.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(log_rows)

        except Exception as e:
            print(f"\n[MAC ERROR] {e}")
            traceback.print_exc()


# =====================================================================
# Main
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
    for i in range(len(mac_hndlr)):
        ric.rm_report_mac_sm(mac_hndlr[i])
        ric.rm_report_rlc_sm(rlc_hndlr[i])
    print("[INFO] xApp stopped.")
