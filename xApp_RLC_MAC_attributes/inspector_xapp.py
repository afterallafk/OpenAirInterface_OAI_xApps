import xapp_sdk as ric
import time
import sys

# Flags to ensure we only print the structure once and don't flood the terminal
mac_inspected = False
rlc_inspected = False

# ==============================================================================
# MAC INSPECTOR
# ==============================================================================
class MACInspectorCallback(ric.mac_cb):
    def handle(self, ind):
        global mac_inspected
        if mac_inspected:
            return

        try:
            print("\n" + "="*60)
            print("🔍 MAC INDICATION STRUCTURE")
            print("="*60)
            
            # Print main MAC Indication attributes
            ind_attrs = [a for a in dir(ind) if not a.startswith('__') and not a.startswith('this')]
            print(f"Top-level Attributes:\n  {ind_attrs}\n")

            # Check for ue_stats
            if hasattr(ind, 'ue_stats') and len(ind.ue_stats) > 0:
                first_ue = ind.ue_stats[0]
                ue_attrs = [a for a in dir(first_ue) if not a.startswith('__') and not a.startswith('this')]
                print(f"UE Stats (ue_stats[0]) Attributes:\n  {ue_attrs}\n")
            else:
                print("⚠️ No UE stats found in this MAC report (is traffic running?)")

            mac_inspected = True

        except Exception as e:
            print(f"[MAC Error] {e}")

# ==============================================================================
# RLC INSPECTOR
# ==============================================================================
class RLCInspectorCallback(ric.rlc_cb):
    def handle(self, ind):
        global rlc_inspected
        if rlc_inspected:
            return

        try:
            print("\n" + "="*60)
            print("🔍 RLC INDICATION STRUCTURE")
            print("="*60)
            
            # Print main RLC Indication attributes
            ind_attrs = [a for a in dir(ind) if not a.startswith('__') and not a.startswith('this')]
            print(f"Top-level Attributes:\n  {ind_attrs}\n")

            # Check for rb_stats
            if hasattr(ind, 'rb_stats') and len(ind.rb_stats) > 0:
                first_rb = ind.rb_stats[0]
                rb_attrs = [a for a in dir(first_rb) if not a.startswith('__') and not a.startswith('this')]
                print(f"Radio Bearer Stats (rb_stats[0]) Attributes:\n  {rb_attrs}\n")
            else:
                print("⚠️ No RB stats found in this RLC report (is traffic running?)")

            rlc_inspected = True

        except Exception as e:
            print(f"[RLC Error] {e}")

# ==============================================================================
# INITIALIZATION & EXECUTION
# ==============================================================================
print("Starting FlexRIC Structure Inspector...")
ric.init()

conn = ric.conn_e2_nodes()
assert(len(conn) > 0), "Error: No E2 nodes connected!"
node_id = conn[0].id

mac_cb = MACInspectorCallback()
rlc_cb = RLCInspectorCallback()

print("Subscribing to MAC and RLC... waiting for data...")
mac_handle = ric.report_mac_sm(node_id, ric.Interval_ms_10, mac_cb)
rlc_handle = ric.report_rlc_sm(node_id, ric.Interval_ms_10, rlc_cb)

# Wait until both callbacks have successfully printed their data
timeout = 20
while (not mac_inspected or not rlc_inspected) and timeout > 0:
    time.sleep(1)
    timeout -= 1

print("\n" + "="*60)
if mac_inspected and rlc_inspected:
    print("✅ Inspection complete!")
else:
    print("⚠️ Timed out waiting for data. Ensure OAI and traffic are running.")
print("="*60)

# Clean up
ric.rm_report_mac_sm(mac_handle)
ric.rm_report_rlc_sm(rlc_handle)
sys.exit(0)
