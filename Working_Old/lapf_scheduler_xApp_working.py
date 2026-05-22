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

# CQI to Spectral Efficiency Mapping (Approximation for metric calc)
CQI_TO_SE = [0.0, 0.1523, 0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 
             1.9141, 2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547]

TOTAL_RBS = 106 # Adjust based on your OAI bandwidth (e.g., 106 for 40MHz, 273 for 100MHz)
ALPHA_EMA = 0.08 # Matches MATLAB EMA alpha

class GlobalUEState:
    def __init__(self):
        self.states = {}
        self.rnti_order = []
        self.csv_file = "lapf_results.csv"

        # Initialize CSV
        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "tstamp", "RNTI", "TrafficType", "5QI", "Priority", 
                "CQI", "Buffer_Bytes", "Latency_ms", "Deadline_ms", 
                "Metric", "RBs_Granted", "Served_Bits", "Throughput_Mbps"
            ])

    def register_ue(self, rnti):
        if rnti not in self.states:
            # FIX: Change modulo to 8
            idx = len(self.rnti_order) % 8 
            qos = QOS_MAPPING[idx]
            self.states[rnti] = {
                "type": qos["type"],
                "label": qos["label"], # <--- ADD THIS
                "5qi": qos["5qi"],
                "priority": qos["priority"],
                "pdb_ms": qos["pdb_ms"],
                "buffer_bytes": 0,
                "latency_us": 0,
                "avg_throughput": 1e-6,
                "cqi": 0,
                "se": 0.15,
                "harq_pending": False
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} as {qos['label']} (5QI: {qos['5qi']})")

global_ue_state = GlobalUEState()

# =====================================================================
# RLC Callback
# =====================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)
        
    def handle(self, ind):
        try:
            if len(ind.rb_stats) > 0:
                # 1. Reset all tracked UE buffers to 0 to prevent stale data
                # (A UE might have emptied its buffer since the last 10ms report)
                for rnti in global_ue_state.states.keys():
                    global_ue_state.states[rnti]["buffer_bytes"] = 0
                    global_ue_state.states[rnti]["latency_us"] = 0

                # 2. Aggregate current stats per RNTI
                for rb in ind.rb_stats:
                    rnti = rb.rnti
                    global_ue_state.register_ue(rnti)
                    
                    # Add bytes from ALL Radio Bearers (SRBs + DRBs) for this UE
                    global_ue_state.states[rnti]["buffer_bytes"] += rb.txbuf_occ_bytes
                    
                    # Track the maximum waiting time across all bearers to ensure strict PDB compliance
                    current_max_lat = global_ue_state.states[rnti]["latency_us"]
                    global_ue_state.states[rnti]["latency_us"] = max(current_max_lat, rb.txsdu_wt_us)
                    
        except Exception as e:
            print(f"\n[PYTHON RLC ERROR] {e}")
            traceback.print_exc()

# =====================================================================
# MAC Callback & Real LAPF Algorithm Execution
# =====================================================================

# 3GPP Approximation: Maps MCS index (0-28) to Spectral Efficiency (SE)
MCS_TO_SE = [
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953, 1.9141, # 0-7
    2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547, 5.5547, # 8-15
    5.5547, 6.2266, 6.9141, 7.4063, 7.4063, 7.4063, 7.4063, 7.4063, # 16-23
    7.4063, 7.4063, 7.4063, 7.4063, 7.4063                          # 24-28
]

class MACCallback(ric.mac_cb):
    def __init__(self):
        ric.mac_cb.__init__(self)
        
    def handle(self, ind):
        try:
            if len(ind.ue_stats) == 0:
                return
                
            t_now = ind.tstamp
            active_ues = []

            # 1. Update MAC attributes safely using attri.txt fields
            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)
                
                # Extract CQI and MCS safely
                try:
                    cqi_val = int(ue.wb_cqi)
                    mcs_val = int(ue.dl_mcs1)
                    
                    # INTELLIGENT SELECTION: Use CQI if valid, otherwise use OAI's active MCS
                    if cqi_val > 0:
                        se = CQI_TO_SE[min(cqi_val, 15)]
                        global_ue_state.states[rnti]["cqi"] = cqi_val # Log actual CQI
                    elif mcs_val > 0:
                        se = MCS_TO_SE[min(mcs_val, 28)]
                        global_ue_state.states[rnti]["cqi"] = mcs_val # Log MCS in place of missing CQI
                    else:
                        se = 0.15 # Absolute worst-case fallback
                        global_ue_state.states[rnti]["cqi"] = 0
                        
                    global_ue_state.states[rnti]["se"] = se
                except:
                    global_ue_state.states[rnti]["se"] = 0.15
                    global_ue_state.states[rnti]["cqi"] = 0

                try:
                    if isinstance(ue.dl_harq, (list, tuple)):
                        global_ue_state.states[rnti]["harq_pending"] = sum(ue.dl_harq) > 0
                    else:
                        global_ue_state.states[rnti]["harq_pending"] = int(ue.dl_harq) > 0
                except:
                    global_ue_state.states[rnti]["harq_pending"] = False

                if global_ue_state.states[rnti]["buffer_bytes"] > 0:
                    active_ues.append(rnti)

            if not active_ues:
                return

            # 2. LAPF Algorithm: Group by SDAP Priority Floors
            priority_floors = {}
            for rnti in active_ues:
                prio = global_ue_state.states[rnti]["priority"]
                if prio not in priority_floors:
                    priority_floors[prio] = []
                priority_floors[prio].append(rnti)

            sorted_priorities = sorted(priority_floors.keys()) 
            available_rbs = TOTAL_RBS
            lapf_allocations = {}

            # 3. Floor-by-Floor Scheduling
            for prio in sorted_priorities:
                if available_rbs <= 0:
                    break
                    
                floor_ues = priority_floors[prio]
                qmax = max([global_ue_state.states[u]["buffer_bytes"] for u in floor_ues]) + 1e-6
                metrics = {}

                for u in floor_ues:
                    st = global_ue_state.states[u]
                    se = st["se"]
                    rate_per_rb = se * 12 * 14

                    pf_term = rate_per_rb / max(st["avg_throughput"], 1e-6)
                    qnorm = st["buffer_bytes"] / qmax
                    
                    lat_ms = st["latency_us"] / 1000.0
                    urgency = 1.0 / max(st["pdb_ms"] - lat_ms,1e-3)

                    metric = pf_term * qnorm * urgency
                    if st["harq_pending"]:
                        metric *= 1.3 

                    metrics[u] = metric

                sorted_floor_ues = sorted(floor_ues, key=lambda x: metrics[x], reverse=True)

                for u in sorted_floor_ues:
                    if available_rbs <= 0:
                        break
                    
                    st = global_ue_state.states[u]
                    se = st["se"]
                    
                    bits_needed = st["buffer_bytes"] * 8
                    rbs_demanded = math.ceil(bits_needed / (se * 12 * 14))
                    
                    #n_left = sum(1 for x in sorted_floor_ues[sorted_floor_ues.index(u):] if global_ue_state.states[x]["buffer_bytes"] > 0)
                    #fair_share = math.ceil(available_rbs / max(n_left, 1))

                    #max_rbs_allowed = available_rbs
                    #if st["type"] == "URLLC":
                    #    max_rbs_allowed = int(TOTAL_RBS * 0.80) 
                    
                    #granted_rbs = min(rbs_demanded, fair_share, max_rbs_allowed, available_rbs)
                    
                    n_left = sum(1 for x in sorted_floor_ues[sorted_floor_ues.index(u):] if global_ue_state.states[x]["buffer_bytes"] > 0)
                    fair_share = math.ceil(available_rbs / max(n_left, 1))
                    
                    granted_rbs = min(rbs_demanded, fair_share, available_rbs)
                    
                    lapf_allocations[u] = {
                        "rbs": granted_rbs,
                        "metric": metrics[u],
                        "served_bits": granted_rbs * (se * 12 * 14)
                    }
                    available_rbs -= granted_rbs

		    # =======================================================
                    # ADD YOUR PRINT STATEMENT HERE
                    # =======================================================
                    if granted_rbs > 0:
                        print(f"[LAPF] tstamp: {t_now} | UE: {u} ({st['label']}) | CQI/MCS: {st['cqi']} | RBs: {granted_rbs} | Metric: {metrics[u]:.2f}")
                    # =======================================================

            # 4. Throughput Evaluation & Logging
            log_rows = []
            for rnti in active_ues:
                st = global_ue_state.states[rnti]
                alloc = lapf_allocations.get(rnti, {"rbs": 0, "metric": 0.0, "served_bits": 0.0})
                
                lat_ms = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]
                
                if st["type"] == "URLLC":
                    if lat_ms > st["pdb_ms"]:
                        effective_tbs = 0.0
                    else:
                        effective_tbs = raw_tbs
                else:
                    effective_tbs = raw_tbs

                # FIX: throughput is bits calculated per 5G slot (0.5ms)
                slot_duration_s = 0.0005 
                Throughput_Mbps = (effective_tbs / 1e6) / slot_duration_s

                st["avg_throughput"] = (1 - ALPHA_EMA) * st["avg_throughput"] + (ALPHA_EMA * effective_tbs)

                log_rows.append([
                    t_now, rnti, st["type"], st["5qi"], st["priority"],
                    st["cqi"], st["buffer_bytes"], round(lat_ms, 2), st["pdb_ms"],
                    round(alloc["metric"], 4), alloc["rbs"], round(raw_tbs, 0), round(Throughput_Mbps, 4)
                ])

            if log_rows:
                with open(global_ue_state.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(log_rows)

        except Exception as e:
            print(f"\n[PYTHON MAC ERROR] {e}")
            import traceback
            traceback.print_exc()

# =====================================================================
# Main Initialization Loop
# =====================================================================
ric.init()
conn = ric.conn_e2_nodes()
assert(len(conn) > 0), "Error: No E2 nodes connected!"

mac_hndlr = []
rlc_hndlr = []

for i in range(0, len(conn)):
    mac_cb = MACCallback()
    rlc_cb = RLCCallback()

    # Register both RLC and MAC indications at 10ms intervals
    hndlr_rlc = ric.report_rlc_sm(conn[i].id, ric.Interval_ms_10, rlc_cb)
    hndlr_mac = ric.report_mac_sm(conn[i].id, ric.Interval_ms_10, mac_cb)

    rlc_hndlr.append(hndlr_rlc)
    mac_hndlr.append(hndlr_mac)
    time.sleep(1)

print(f"LAPF Integrated Scheduler running INFINITELY. Logging to {global_ue_state.csv_file}...")
print("Press Ctrl+C to stop the xApp.")

try:
    # Forces the main thread to stay alive indefinitely
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Ctrl+C detected. Shutting down gracefully...")
finally:
    # Guarantees the subscriptions are removed when you finally kill it
    print("[INFO] Cleaning up MAC and RLC reports...")
    for i in range(0, len(mac_hndlr)):
        ric.rm_report_mac_sm(mac_hndlr[i])
        ric.rm_report_rlc_sm(rlc_hndlr[i])
    print("[INFO] xApp successfully stopped.")
