import xapp_sdk as ric
import time
import csv
import traceback

CSV_FILE = "internal_pf_logs.csv"
ALPHA_EMA = 0.08

# Initialize CSV with the new Latency and Throughput headers
with open(CSV_FILE, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        "tstamp", "RNTI", "CQI", "MCS", 
        "PF_Granted_RBs", "PF_Current_TBS_Bytes", "Buffer_Status_Report", 
        "Frame", "Slot", "Latency_ms", "Throughput_Mbps"
    ])

# =====================================================================
# State Tracker (Bridges RLC Latency to MAC Logging)
# =====================================================================
class GlobalUEState:
    def __init__(self):
        self.states = {}

    def register_ue(self, rnti):
        if rnti not in self.states:
            self.states[rnti] = {
                "latency_us": 0,
                "avg_throughput": 1e-6
            }

global_ue_state = GlobalUEState()

# =====================================================================
# RLC Callback (Extracts Latency)
# =====================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)
        
    def handle(self, ind):
        try:
            if len(ind.rb_stats) > 0:
                for rb in ind.rb_stats:
                    rnti = rb.rnti
                    global_ue_state.register_ue(rnti)
                    # Constantly update the real-time queue latency
                    global_ue_state.states[rnti]["latency_us"] = rb.txsdu_wt_us
        except Exception as e:
            print(f"\n[PYTHON RLC ERROR] {e}")
            traceback.print_exc()

# =====================================================================
# MAC Callback (Extracts PF Decisions & Calculates Throughput)
# =====================================================================
class MACCallback(ric.mac_cb):
    def __init__(self):
        ric.mac_cb.__init__(self)
        
    def handle(self, ind):
        try:
            if len(ind.ue_stats) == 0:
                return
                
            t_now = ind.tstamp
            log_rows = []

            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)
                st = global_ue_state.states[rnti]
                
                # Extract PF scheduler attributes
                cqi = int(ue.wb_cqi)
                mcs = int(ue.dl_mcs1)
                pf_rbs = int(ue.dl_sched_rb)
                pf_tbs_bytes = int(ue.dl_curr_tbs) # OAI TBS is in Bytes
                bsr = int(ue.bsr)
                frame = int(ue.frame)
                slot = int(ue.slot)

                # Only evaluate if the internal PF scheduler actually gave RBs
                if pf_rbs > 0:
                    # 1. Fetch real-time latency from RLC state
                    lat_ms = st["latency_us"] / 1000.0

                    # 2. Calculate Throughput (Bits per 0.5ms slot)
                    served_bits = pf_tbs_bytes * 8
                    slot_duration_s = 0.0005 
                    throughput_mbps = (served_bits / 1e6) / slot_duration_s

                    # Update EMA (Optional: useful if you want to track stability)
                    st["avg_throughput"] = (1 - ALPHA_EMA) * st["avg_throughput"] + (ALPHA_EMA * served_bits)

                    print(f"[OAI-PF] Slot: {frame}.{slot} | RNTI: {rnti} | RBs: {pf_rbs} | Latency: {lat_ms:.2f}ms | Throughput: {throughput_mbps:.2f} Mbps")
                    
                    log_rows.append([
                        t_now, rnti, cqi, mcs, pf_rbs, pf_tbs_bytes, bsr, 
                        frame, slot, round(lat_ms, 2), round(throughput_mbps, 4)
                    ])

            if log_rows:
                with open(CSV_FILE, mode='a', newline='') as f:
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
assert(len(conn) > 0), "Error: No E2 nodes connected!"

mac_hndlr = []
rlc_hndlr = []

for i in range(0, len(conn)):
    mac_cb = MACCallback()
    rlc_cb = RLCCallback()
    
    # Subscribe to BOTH MAC and RLC to fuse the metrics
    hndlr_rlc = ric.report_rlc_sm(conn[i].id, ric.Interval_ms_10, rlc_cb)
    hndlr_mac = ric.report_mac_sm(conn[i].id, ric.Interval_ms_10, mac_cb)
    
    rlc_hndlr.append(hndlr_rlc)
    mac_hndlr.append(hndlr_mac)
    time.sleep(1)

print(f"Monitoring Internal OAI PF Scheduler (With Latency & Throughput). Logging to {CSV_FILE}...")
print("Press Ctrl+C to stop the xApp.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Ctrl+C detected. Shutting down gracefully...")
finally:
    print("[INFO] Cleaning up MAC and RLC reports...")
    for i in range(0, len(mac_hndlr)):
        ric.rm_report_mac_sm(mac_hndlr[i])
        ric.rm_report_rlc_sm(rlc_hndlr[i])
    print("[INFO] xApp successfully stopped.")
