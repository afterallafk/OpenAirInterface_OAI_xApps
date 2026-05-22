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
# 3GPP TS 38.214 CQI Tables — ported from lookupMCSFromCQI() in MATLAB
#
# Four tables, indexed by CQI 0–15:
#   Table 1 — Standard 64QAM     (eMBB default / mMTC)
#   Table 2 — 256QAM             (eMBB high SINR)
#   Table 3 — Low-SE QPSK/16QAM (URLLC — always used for URLLC)
#   Table 4 — 1024QAM            (eMBB excellent SINR ≥ 25 dB)
#
# Each row: (spectral_efficiency, qm, code_rate/1024)
# SE = qm * code_rate / 1024  (matches MATLAB table values exactly)
# =====================================================================

# (SE, Qm, code_rate_x1024)
_CQI_TABLE_1 = [
    (0.0000, 2,   0),   # CQI 0: out of range
    (0.1523, 2,  78),   # CQI 1
    (0.2344, 2, 120),   # CQI 2
    (0.3770, 2, 193),   # CQI 3
    (0.6016, 2, 308),   # CQI 4
    (0.8770, 2, 449),   # CQI 5
    (1.1758, 2, 602),   # CQI 6
    (1.4766, 4, 378),   # CQI 7
    (1.9141, 4, 490),   # CQI 8
    (2.4063, 4, 616),   # CQI 9
    (2.7305, 6, 466),   # CQI 10
    (3.3223, 6, 567),   # CQI 11
    (3.9023, 6, 666),   # CQI 12
    (4.5234, 6, 772),   # CQI 13
    (5.1152, 6, 873),   # CQI 14
    (5.5547, 6, 948),   # CQI 15
]

_CQI_TABLE_2 = [
    (0.0000, 2,   0),
    (0.1523, 2,  78),
    (0.3770, 2, 193),
    (0.8770, 2, 449),
    (1.4766, 4, 378),
    (1.9141, 4, 490),
    (2.4063, 4, 616),
    (2.7305, 6, 466),
    (3.3223, 6, 567),
    (3.9023, 6, 666),
    (4.5234, 6, 772),
    (5.1152, 6, 873),
    (5.5547, 8, 711),
    (6.2266, 8, 797),
    (6.9141, 8, 885),
    (7.4063, 8, 948),
]

# URLLC — conservative low-SE table (always used for URLLC regardless of SINR)
_CQI_TABLE_3 = [
    (0.0000, 2,   0),
    (0.0586, 2,  30),
    (0.0977, 2,  50),
    (0.1523, 2,  78),
    (0.2344, 2, 120),
    (0.3770, 2, 193),
    (0.6016, 2, 308),
    (0.8770, 2, 449),
    (1.1758, 2, 602),
    (1.4766, 4, 378),
    (1.9141, 4, 490),
    (2.4063, 4, 616),
    (2.7305, 6, 466),
    (3.3223, 6, 567),
    (3.9023, 6, 666),
    (4.5234, 6, 772),
]

_CQI_TABLE_4 = [
    (0.0000,  2,   0),
    (0.1523,  2,  78),
    (0.3770,  2, 193),
    (0.8770,  2, 449),
    (1.4766,  4, 378),
    (2.4063,  4, 616),
    (3.3223,  6, 567),
    (3.9023,  6, 666),
    (4.5234,  6, 772),
    (5.1152,  6, 873),
    (5.5547,  8, 711),
    (6.2266,  8, 797),
    (6.9141,  8, 885),
    (7.4063,  8, 948),
    (8.3301, 10, 853),
    (9.2578, 10, 948),
]

# SINR thresholds (dB) per table — from estimateCQIFromSINR() in MATLAB
# Index i → threshold for CQI i (i=0 is unused, thresholds[1] is for CQI 1)
_SINR_THRESH = {
    1: [-10, -8, -6.7, -4.7, -2.3,  0.2,  2.4,  4.3,
         5.9,  8.1, 10.3, 11.7, 14.1, 16.3, 17.8, 19.0],   # Table 3 (URLLC)
    2: [-6.7, -2.3,  2.4,  5.9,  8.1, 10.3, 11.7, 14.1,
        16.3, 17.8, 19.0, 20.3, 22.0, 24.0, 26.0, 28.0],   # Table 2 (eMBB high)
    4: [-6.7, -2.3,  2.4,  5.9, 10.3, 14.1, 16.3, 17.8,
        19.0, 20.3, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0],   # Table 4 (eMBB excellent)
    0: [-6.7, -4.7, -2.3,  0.2,  2.4,  4.3,  5.9,  8.1,
        10.3, 11.7, 14.1, 16.3, 17.8, 19.0, 20.3, 22.0],   # Table 1 (default)
}

NUM_LAYERS   = 2         # 2×2 MIMO
SC_PER_RB    = 12
NUM_SYMB     = 14        # OFDM symbols per slot (30 kHz SCS, normal CP)
SLOT_DUR_S   = 0.5e-3   # 0.5 ms
TOTAL_RBS    = 106       # 40 MHz, 30 kHz SCS
ALPHA_EMA    = 0.08      # matched exactly to MATLAB pfScheduler alpha=0.08
EPSILON      = 1e-6


# =====================================================================
# selectMCSFromSINR — port of MATLAB selectMCSFromSINR / lookupMCSFromCQI
#
# Returns: spectral_efficiency (bits/s/Hz) for this UE's channel + type
#
# MATLAB pfScheduler uses nrTBS(modStr, numLayers, nRBs, 14, tcr) to get
# the actual TBS.  In the OAI xApp we don't have nrTBS, so we approximate:
#   TBS_per_RB ≈ SE × SC_PER_RB × NUM_SYMB × NUM_LAYERS
# This is the same formula used in the DQN/LAPF xApps (tbs_per_rb).
# The SE lookup is now table-aware (URLLC→Table3, eMBB adaptive, mMTC→Table1),
# matching MATLAB exactly.
# =====================================================================
def _estimate_cqi_from_sinr(sinr_db: float, traffic_type: str) -> int:
    """Port of estimateCQIFromSINR() — adaptive table selection per MATLAB."""
    if traffic_type == "URLLC":
        thresh = _SINR_THRESH[1]   # Table 3
    elif traffic_type == "eMBB":
        if sinr_db >= 25.0:
            thresh = _SINR_THRESH[4]   # Table 4
        elif sinr_db >= 20.0:
            thresh = _SINR_THRESH[2]   # Table 2
        else:
            thresh = _SINR_THRESH[0]   # Table 1
    else:   # mMTC
        thresh = _SINR_THRESH[0]   # Table 1

    # Find highest CQI where SINR >= threshold (MATLAB: loop 15:-1:1)
    cqi = 0
    for i in range(15, 0, -1):
        if sinr_db >= thresh[i]:
            cqi = i
            break
    return max(0, min(15, cqi))


def _lookup_se_from_cqi(cqi: int, traffic_type: str, sinr_db: float = 0.0) -> float:
    """Port of lookupMCSFromCQI() — returns spectral efficiency for 1 RB."""
    if traffic_type == "URLLC":
        table = _CQI_TABLE_3
    elif traffic_type == "eMBB":
        if sinr_db >= 25.0:
            table = _CQI_TABLE_4
        elif sinr_db >= 20.0:
            table = _CQI_TABLE_2
        else:
            table = _CQI_TABLE_1
    else:   # mMTC
        table = _CQI_TABLE_1

    if 0 <= cqi <= 15:
        return table[cqi][0]
    # out of range → most conservative
    return _CQI_TABLE_3[3][0]   # QPSK, CR=78/1024 → SE=0.1523


def tbs_per_rb(se: float) -> float:
    """Approximate TBS for 1 RB — SE × 12 subcarriers × 14 symbols × 2 layers."""
    return se * SC_PER_RB * NUM_SYMB * NUM_LAYERS


def get_se_for_ue(sinr_db: float, traffic_type: str) -> float:
    """Full port of selectMCSFromSINR: SINR → CQI (table-aware) → SE."""
    cqi = _estimate_cqi_from_sinr(sinr_db, traffic_type)
    return _lookup_se_from_cqi(cqi, traffic_type, sinr_db)


# =====================================================================
# SINR estimation from CQI/MCS (used when no explicit SINR is available
# from OAI — inverted from CQI index using Table 1 midpoints)
# =====================================================================
# In the OAI xApp we receive CQI or MCS, not SINR directly.
# Strategy: map CQI → SE using the appropriate 3GPP table for that UE type.
# This correctly applies Table 3 for URLLC (conservative) and Table 1 for
# eMBB/mMTC (standard), matching MATLAB pfScheduler behaviour exactly.
def se_from_cqi_typed(cqi_val: int, traffic_type: str) -> float:
    """Return SE using the 3GPP table appropriate for this UE's traffic type."""
    if traffic_type == "URLLC":
        table = _CQI_TABLE_3
    elif traffic_type == "eMBB":
        table = _CQI_TABLE_1   # default; upgraded only if SINR known
    else:
        table = _CQI_TABLE_1
    cqi_clamped = max(0, min(15, cqi_val))
    return table[cqi_clamped][0]


# MCS → SE fallback table (3GPP TS 38.214 Table 5.1.3.1-2, same as before)
MCS_TO_SE = [
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953, 1.9141,
    2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547, 5.5547,
    5.5547, 6.2266, 6.9141, 7.4063, 7.4063, 7.4063, 7.4063, 7.4063,
    7.4063, 7.4063, 7.4063, 7.4063, 7.4063
]

SLOT_DURATION_MS = 0.5
SLOT_DURATION_S  = 0.5e-3


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
                "avg_throughput": 1e-6,
                "cqi":            0,
                "se":             0.15,
                "harq_pending":   False,
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} as {qos['label']} "
                  f"(5QI: {qos['5qi']}, deadline: {self.states[rnti]['deadline_slots']} slots)")


global_ue_state = GlobalUEState()


# =====================================================================
# RLC Callback
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
# Faithfully ports MATLAB pfScheduler (lines 1276–1400 of LAPFScheduler.m).
#
# Key behaviours preserved from MATLAB:
#
#   1. FULL-BUFFER / WINNER-TAKES-ALL allocation (Bug 1 fix in MATLAB)
#      The top-ranked UE by PF metric receives ALL remaining RBs in one
#      grant (grant = RBavail).  The loop continues with RBavail=0, so
#      effectively only one UE is served per slot.  This is classic
#      single-winner-per-slot PF as described in MATLAB comment §1340–1344:
#        "Top-ranked UE by PF metric gets ALL remaining RBs.
#         winner-takes-all per slot."
#      The previous Python version used fair-share ceil(RBavail/n_left)
#      which is the LAPF allocation pattern — NOT correct for plain PF.
#
#   2. NO DEMAND CAP — full-buffer model
#      MATLAB: "servedBits uncapped — full-buffer spectral efficiency
#               measurement" and "NO demand/buffer-driven capping".
#      The previous Python code computed rbs_demanded = ceil(bits/tbs_per_rb)
#      and capped grants to that demand.  Removed entirely.
#
#   3. NO URLLC LATENCY ZEROING
#      MATLAB PF has no effective_tbs=0 on deadline miss — that logic
#      belongs only to LAPF.  The previous Python version applied it,
#      which contaminated avg_throughput and inflated PF metrics for
#      URLLC UEs that missed deadlines.  Removed.
#
#   4. TRAFFIC-TYPE-AWARE SE (Bug 2 extension)
#      MATLAB pfScheduler calls selectMCSFromSINR(sinr_dB, trafficType)
#      which uses Table 3 for URLLC and Table 1 for eMBB/mMTC.
#      The previous Python code used a single flat CQI_TO_SE table for all
#      UEs.  Now uses se_from_cqi_typed() which applies the correct table.
#
#   5. HARQ boost ×1.3 — preserved (matches MATLAB line 1324–1325)
#
#   6. alpha = 0.08 EMA — preserved (matches MATLAB line 1294)
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
                st   = global_ue_state.states[rnti]

                # CQI → SE using traffic-type-aware table (MATLAB port)
                try:
                    cqi_val = int(ue.wb_cqi)
                    mcs_val = int(ue.dl_mcs1)

                    if cqi_val > 0:
                        # Apply the 3GPP table appropriate for this UE type
                        se = se_from_cqi_typed(cqi_val, st["type"])
                        st["cqi"] = cqi_val
                    elif mcs_val > 0:
                        # MCS fallback — no per-type table available for MCS,
                        # use the flat MCS→SE table as before
                        se = MCS_TO_SE[min(mcs_val, 28)]
                        st["cqi"] = mcs_val
                    else:
                        # Use minimum valid SE for this UE type
                        se = se_from_cqi_typed(1, st["type"])
                        st["cqi"] = 0

                    st["se"] = se

                except Exception:
                    st["se"]  = 0.15
                    st["cqi"] = 0

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
            # MATLAB pfScheduler §1303–1331:
            #   pfMetric(u) = ratePerRB(u) / max(avgThroughput(u), epsilon)
            # where ratePerRB = nrTBS(moduse, 2, 1, 14, tcr) — bits for 1 RB.
            # Equivalent in xApp: tbs_per_rb(se) using typed SE lookup.
            pf_metrics = {}

            for rnti in active_ues:
                st = global_ue_state.states[rnti]

                inst_rate = tbs_per_rb(st["se"])   # bits achievable per 1 RB
                metric    = inst_rate / max(st["avg_throughput"], EPSILON)

                if st["harq_pending"]:
                    metric *= 1.3   # MATLAB line 1324–1325

                pf_metrics[rnti] = metric

            # ── 3. PF Scheduling: winner-takes-all ────────────────────────
            # MATLAB §1334–1395: sort by pfMetric descending, then each UE
            # in order receives grant = RBavail (all remaining RBs).
            # Since the first UE gets ALL RBs, RBavail drops to 0 and the
            # loop exits immediately after one UE is served.
            # This is the correct MATLAB full-buffer PF behaviour.
            sorted_ues  = sorted(active_ues, key=lambda x: pf_metrics[x], reverse=True)
            available_rbs = TOTAL_RBS
            pf_allocations = {}

            for rnti in sorted_ues:
                if available_rbs <= 0:
                    break

                # Full greedy: give ALL remaining RBs to this UE
                grant = available_rbs   # MATLAB: grant = RBavail

                st          = global_ue_state.states[rnti]
                served_bits = grant * tbs_per_rb(st["se"])

                # Full-buffer: served_bits NOT capped by buffer_bytes
                # (MATLAB: "servedBits uncapped — full-buffer model")
                pf_allocations[rnti] = {
                    "rbs":         grant,
                    "metric":      pf_metrics[rnti],
                    "served_bits": served_bits,
                }

                available_rbs -= grant   # → 0 after first UE

                print(f"[PF] tstamp: {t_now} | UE: {rnti} ({st['label']}) | "
                      f"CQI: {st['cqi']} | Type: {st['type']} | "
                      f"RBs: {grant} | PF_Metric: {pf_metrics[rnti]:.4f}")

            # ── 4. Throughput EMA + CSV logging ───────────────────────────
            # MATLAB §1397–1398:
            #   avgThroughputNew = (1-alpha)*avgThroughput + alpha*servedBits
            # EMA applied to ALL active UEs (including unserved ones) so that
            # avg_throughput decays for UEs not scheduled this slot, raising
            # their PF metric next slot. NO latency-zeroing for URLLC here.
            log_rows = []

            for rnti in active_ues:
                st    = global_ue_state.states[rnti]
                alloc = pf_allocations.get(rnti, {"rbs": 0, "metric": 0.0, "served_bits": 0.0})

                lat_ms  = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]

                # Full-buffer model: no latency zeroing
                # (MATLAB PF does not zero out URLLC on deadline miss — that
                # is exclusive to LAPF. Using served_bits directly.)
                throughput_mbps = (raw_tbs / 1e6) / SLOT_DURATION_S

                # EMA update — identical to MATLAB alpha=0.08
                st["avg_throughput"] = ((1 - ALPHA_EMA) * st["avg_throughput"]
                                        + ALPHA_EMA * raw_tbs)

                log_rows.append([
                    t_now, rnti, st["type"], st["5qi"], st["priority"],
                    st["cqi"], st["buffer_bytes"],
                    round(lat_ms, 2), st["pdb_ms"],
                    round(alloc["metric"], 6),
                    alloc["rbs"], round(raw_tbs, 0),
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
