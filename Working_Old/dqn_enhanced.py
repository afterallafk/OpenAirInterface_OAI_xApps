import xapp_sdk as ric
import time
import csv
import math
import os
import traceback
import random
import collections
import json

# =====================================================================
# Optional dependencies
# =====================================================================
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    print("[WARN] PyTorch not found – using pure-Python Q-table fallback.")

# =====================================================================
# SDAP / QoS Configuration  (8 UEs: 4 URLLC, 2 eMBB, 2 mMTC)
# Order matches traffic_gen8.sh tunnel assignment exactly
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

CQI_TO_SE = [0.0, 0.1523, 0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766,
             1.9141, 2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547]

MCS_TO_SE = [
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953, 1.9141,
    2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547, 5.5547,
    5.5547, 6.2266, 6.9141, 7.4063, 7.4063, 7.4063, 7.4063, 7.4063,
    7.4063, 7.4063, 7.4063, 7.4063, 7.4063
]

TOTAL_RBS  = 106    # 40 MHz → 106 PRBs
ALPHA_EMA  = 0.08
NUM_UES    = 8

# =====================================================================
# DQN Hyper-parameters
# =====================================================================
# State: per UE → [norm_buffer, deadline_fraction, norm_cqi, norm_tput, type_enc]
STATE_DIM       = NUM_UES * 5
# Action: per UE → {0=reduce 10%, 1=keep, 2=increase 10%}  (3 choices × 8 UEs)
ACTION_DIM      = NUM_UES * 3
HIDDEN_DIM      = 128
REPLAY_CAPACITY = 10_000
BATCH_SIZE      = 64
GAMMA           = 0.99
LR              = 1e-3
EPSILON_START   = 1.0
EPSILON_END     = 0.05
EPSILON_DECAY   = 0.995
TARGET_UPDATE   = 50        # hard copy policy → target every N steps
MIN_REPLAY      = 256       # don't learn until buffer has this many transitions

RB_DELTA_FRAC = [-0.10, 0.0, +0.10]   # action 0/1/2

# =====================================================================
# Pure-Python Q-table fallback (no PyTorch)
# =====================================================================
class FallbackQTable:
    def __init__(self):
        self.q  = collections.defaultdict(lambda: [0.0] * ACTION_DIM)
        self.lr = LR

    def predict(self, state_vec):
        return list(self.q[self._key(state_vec)])

    def update(self, state_vec, action_idx, target):
        k = self._key(state_vec)
        self.q[k][action_idx] += self.lr * (target - self.q[k][action_idx])

    @staticmethod
    def _key(vec):
        return tuple(int(min(v, 0.9999) * 5) for v in vec)


# =====================================================================
# PyTorch DQN network
# =====================================================================
if HAS_TORCH:
    class DQNNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(STATE_DIM,  HIDDEN_DIM), nn.ReLU(),
                nn.Linear(HIDDEN_DIM, HIDDEN_DIM), nn.ReLU(),
                nn.Linear(HIDDEN_DIM, ACTION_DIM),
            )
        def forward(self, x):
            return self.net(x)


# =====================================================================
# Replay Buffer
# =====================================================================
class ReplayBuffer:
    def __init__(self, capacity):
        self.buf = collections.deque(maxlen=capacity)

    def push(self, s, a, r, ns, done):
        self.buf.append((s, a, r, ns, done))

    def sample(self, n):
        return random.sample(self.buf, n)

    def __len__(self):
        return len(self.buf)


# =====================================================================
# DQN Agent
# =====================================================================
class DQNAgent:
    def __init__(self):
        self.epsilon      = EPSILON_START
        self.steps        = 0
        self.replay       = ReplayBuffer(REPLAY_CAPACITY)
        self._prev_state  = None
        self._prev_action = None
        self.reward_log   = []
        self.loss_log     = []

        # Per-UE RB fraction, initialised on first encounter
        self.rb_fractions = {}

        if HAS_TORCH:
            self.device  = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.policy  = DQNNet().to(self.device)
            self.target  = DQNNet().to(self.device)
            self.target.load_state_dict(self.policy.state_dict())
            self.target.eval()
            self.opt     = optim.Adam(self.policy.parameters(), lr=LR)
            self.loss_fn = nn.MSELoss()
            print(f"[DQN] PyTorch on {self.device} | state={STATE_DIM} action={ACTION_DIM}")
        else:
            self.qtable = FallbackQTable()
            print("[DQN] Pure-Python Q-table active.")

    # ------------------------------------------------------------------
    # Build normalised state vector from current UE states
    # ------------------------------------------------------------------
    def _state(self, ue_states, rnti_order):
        TYPE_ENC = {"URLLC": 0.0, "mMTC": 0.5, "eMBB": 1.0}
        vec = []
        for rnti in rnti_order[:NUM_UES]:
            st = ue_states[rnti]
            vec.append(min(st["buffer_bytes"] / 1e6, 1.0))
            vec.append(min((st["latency_us"] / 1000.0) / max(st["pdb_ms"], 1e-3), 1.0))
            vec.append(st["cqi"] / 15.0)
            vec.append(min(math.log10(st["avg_throughput"] + 1) / 10.0, 1.0))
            vec.append(TYPE_ENC.get(st["type"], 0.5))
        while len(vec) < STATE_DIM:
            vec.append(0.0)
        return vec[:STATE_DIM]

    # ------------------------------------------------------------------
    # ε-greedy action selection
    # ------------------------------------------------------------------
    def _select_action(self, state_vec):
        if random.random() < self.epsilon:
            return [random.randint(0, 2) for _ in range(NUM_UES)]
        return self._greedy(state_vec)

    def _greedy(self, state_vec):
        if HAS_TORCH:
            with torch.no_grad():
                s = torch.FloatTensor(state_vec).unsqueeze(0).to(self.device)
                q = self.policy(s).squeeze(0).cpu().tolist()
        else:
            q = self.qtable.predict(state_vec)
        return [max(range(3), key=lambda a: q[i*3 + a]) for i in range(NUM_UES)]

    # ------------------------------------------------------------------
    # Reward: penalise deadline misses, reward throughput + efficiency
    # ------------------------------------------------------------------
    def _reward(self, ue_states, rnti_order, allocations):
        reward         = 0.0
        total_rbs_used = sum(a.get("rbs", 0) for a in allocations.values())

        for rnti in rnti_order[:NUM_UES]:
            st    = ue_states[rnti]
            alloc = allocations.get(rnti, {"rbs": 0, "served_bits": 0.0})
            lat_ms = st["latency_us"] / 1000.0
            
            # FIXED: Converted to Mbps
            tput   = (alloc["served_bits"] / 1e6) / 0.0005 

            if st["type"] == "URLLC":
                if lat_ms > st["pdb_ms"]:
                    # FIXED: Removed the -10.0 penalty to prevent death spirals
                    reward += 0.0 
                else:
                    margin  = (st["pdb_ms"] - lat_ms) / st["pdb_ms"]
                    reward += tput * (1.0 + margin)
            else:
                # FIXED: Clamped to prevent minor negative dips from high latency
                reward += max(0.0, tput - min(lat_ms / st["pdb_ms"], 1.0))

        # Resource efficiency bonus (peaks at ~80% utilisation)
        util    = total_rbs_used / max(TOTAL_RBS, 1)
        reward += 2.0 * (1.0 - abs(util - 0.80) / 0.80)
        
        # FIXED: Final safety clamp to guarantee the DQN never sees a negative reward
        return float(max(0.0, reward))

    # ------------------------------------------------------------------
    # Bellman update with experience replay
    # ------------------------------------------------------------------
    def _learn(self):
        if len(self.replay) < MIN_REPLAY:
            return
        batch  = self.replay.sample(BATCH_SIZE)
        states, actions, rewards, next_states, dones = zip(*batch)

        if HAS_TORCH:
            s  = torch.FloatTensor(states).to(self.device)
            ns = torch.FloatTensor(next_states).to(self.device)
            r  = torch.FloatTensor(rewards).to(self.device)
            d  = torch.FloatTensor(dones).to(self.device)

            q_all = self.policy(s)
            with torch.no_grad():
                q_next = self.target(ns)

            q_target = q_all.clone().detach()
            for ue_i in range(NUM_UES):
                base      = ue_i * 3
                best_next = q_next[:, base:base+3].max(dim=1).values
                act_idx   = torch.tensor([a[ue_i] for a in actions],
                                          dtype=torch.long, device=self.device)
                td_target = r + GAMMA * best_next * (1 - d)
                q_target[torch.arange(BATCH_SIZE), base + act_idx] = td_target

            loss = self.loss_fn(q_all, q_target)
            self.opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 1.0)
            self.opt.step()
            self.loss_log.append(loss.item())

            if self.steps % TARGET_UPDATE == 0:
                self.target.load_state_dict(self.policy.state_dict())
        else:
            for (s, a, rew, ns, done) in batch:
                q_ns = self.qtable.predict(ns)
                for ue_i in range(NUM_UES):
                    base      = ue_i * 3
                    best_next = max(q_ns[base:base+3])
                    td_target = rew + GAMMA * best_next * (1 - done)
                    self.qtable.update(s, base + a[ue_i], td_target)

    # ------------------------------------------------------------------
    # Main entry per scheduling tick
    # ------------------------------------------------------------------
    def schedule(self, ue_states, rnti_order, prev_allocations):
        """Called every MAC tick. Returns dict: rnti → {rbs, served_bits}"""
        if not rnti_order:
            return {}

        state_vec = self._state(ue_states, rnti_order)

        # Store transition from last step into replay buffer
        if self._prev_state is not None:
            reward = self._reward(ue_states, rnti_order, prev_allocations)
            self.reward_log.append(reward)
            self.replay.push(self._prev_state, self._prev_action,
                             reward, state_vec, 0.0)
            self._learn()

        actions           = self._select_action(state_vec)
        self.epsilon      = max(EPSILON_END, self.epsilon * EPSILON_DECAY)
        self.steps       += 1
        self._prev_state  = state_vec
        self._prev_action = actions

        return self._actions_to_alloc(actions, rnti_order, ue_states)

    # ------------------------------------------------------------------
    # Translate per-UE action → integer RB counts
    # ------------------------------------------------------------------
    def _actions_to_alloc(self, actions, rnti_order, ue_states):
        frac_default = {"URLLC": 0.15, "eMBB": 0.12, "mMTC": 0.08}
        for rnti in rnti_order[:NUM_UES]:
            if rnti not in self.rb_fractions:
                self.rb_fractions[rnti] = frac_default.get(
                    ue_states[rnti]["type"], 0.10)

        # Apply deltas from this tick's actions
        for idx, rnti in enumerate(rnti_order[:NUM_UES]):
            delta = RB_DELTA_FRAC[actions[idx]]
            self.rb_fractions[rnti] = max(0.02,
                                          min(0.80,
                                              self.rb_fractions[rnti] + delta))

        # Renormalise so sum ≤ 1.0
        total = sum(self.rb_fractions[r] for r in rnti_order[:NUM_UES])
        if total > 1.0:
            for rnti in rnti_order[:NUM_UES]:
                self.rb_fractions[rnti] /= total

        alloc     = {}
        remaining = TOTAL_RBS
        for rnti in rnti_order[:NUM_UES]:
            if ue_states[rnti]["buffer_bytes"] > 0 and remaining > 0:
                rbs = max(1, min(int(self.rb_fractions[rnti] * TOTAL_RBS), remaining))
                se  = ue_states[rnti]["se"]
                alloc[rnti] = {"rbs": rbs, "served_bits": rbs * se * 12 * 14}
                remaining  -= rbs
            else:
                alloc[rnti] = {"rbs": 0, "served_bits": 0.0}
        return alloc


# =====================================================================
# Global UE State
# =====================================================================
class GlobalUEState:
    def __init__(self):
        self.states     = {}
        self.rnti_order = []
        self.csv_file   = "dqn_results.csv"

        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "tstamp", "RNTI", "TrafficType", "5QI", "Priority",
                "CQI", "Buffer_Bytes", "Latency_ms", "Deadline_ms",
                "DQN_Action", "RBs_Granted", "Served_Bits",
                "Throughput_Mbps", "Epsilon", "Reward"
            ])

    def register_ue(self, rnti):
        if rnti not in self.states:
            idx = len(self.rnti_order) % NUM_UES
            qos = QOS_MAPPING[idx]
            self.states[rnti] = {
                "type":           qos["type"],
                "label":          qos["label"],
                "5qi":            qos["5qi"],
                "priority":       qos["priority"],
                "pdb_ms":         qos["pdb_ms"],
                "buffer_bytes":   0,
                "latency_us":     0,
                "avg_throughput": 1e-6,
                "cqi":            0,
                "se":             0.15,
                "harq_pending":   False,
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} → {qos['label']} (5QI {qos['5qi']})")


global_ue_state = GlobalUEState()
dqn_agent       = DQNAgent()
_last_alloc     = {}   # carries previous allocations for reward computation


# =====================================================================
# RLC Callback – buffer occupancy & HOL latency
# =====================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        try:
            if len(ind.rb_stats) == 0:
                return

            # Clear stale data before aggregating fresh report
            for rnti in global_ue_state.states:
                global_ue_state.states[rnti]["buffer_bytes"] = 0
                global_ue_state.states[rnti]["latency_us"]   = 0

            for rb in ind.rb_stats:
                rnti = rb.rnti
                global_ue_state.register_ue(rnti)
                global_ue_state.states[rnti]["buffer_bytes"] += rb.txbuf_occ_bytes
                cur = global_ue_state.states[rnti]["latency_us"]
                global_ue_state.states[rnti]["latency_us"]   = max(cur, rb.txsdu_wt_us)

        except Exception as e:
            print(f"\n[RLC ERROR] {e}")
            traceback.print_exc()


# =====================================================================
# MAC Callback – DQN runs here every 10 ms
# =====================================================================
class MACCallback(ric.mac_cb):
    def __init__(self):
        ric.mac_cb.__init__(self)

    def handle(self, ind):
        global _last_alloc
        try:
            if len(ind.ue_stats) == 0:
                return

            t_now      = ind.tstamp
            active_ues = []

            # ── Update CQI / MCS / HARQ ───────────────────────────────
            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)

                try:
                    cqi_val = int(ue.wb_cqi)
                    mcs_val = int(ue.dl_mcs1)
                    if cqi_val > 0:
                        se  = CQI_TO_SE[min(cqi_val, 15)]
                        global_ue_state.states[rnti]["cqi"] = cqi_val
                    elif mcs_val > 0:
                        se  = MCS_TO_SE[min(mcs_val, 28)]
                        global_ue_state.states[rnti]["cqi"] = mcs_val
                    else:
                        se  = 0.15
                        global_ue_state.states[rnti]["cqi"] = 0
                    global_ue_state.states[rnti]["se"] = se
                except Exception:
                    global_ue_state.states[rnti]["se"]  = 0.15
                    global_ue_state.states[rnti]["cqi"] = 0

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

            # ── DQN scheduling ────────────────────────────────────────
            allocations = dqn_agent.schedule(
                global_ue_state.states,
                global_ue_state.rnti_order,
                _last_alloc
            )
            _last_alloc = allocations

            # ── Throughput EMA + CSV logging ──────────────────────────
            slot_dur_s   = 0.0005
            last_reward  = dqn_agent.reward_log[-1] if dqn_agent.reward_log else 0.0
            action_labels = ["REDUCE", "KEEP", "INCREASE"]
            action_map   = {}
            if dqn_agent._prev_action:
                for idx, rnti in enumerate(global_ue_state.rnti_order[:NUM_UES]):
                    action_map[rnti] = dqn_agent._prev_action[idx]

            log_rows = []
            for rnti in active_ues:
                st    = global_ue_state.states[rnti]
                alloc = allocations.get(rnti, {"rbs": 0, "served_bits": 0.0})

                lat_ms  = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]
                eff_tbs = 0.0 if (st["type"] == "URLLC" and lat_ms > st["pdb_ms"]) else raw_tbs
                tput    = (eff_tbs / 1e6) / slot_dur_s

                st["avg_throughput"] = ((1 - ALPHA_EMA) * st["avg_throughput"]
                                        + ALPHA_EMA * eff_tbs)

                act_str = action_labels[action_map.get(rnti, 1)]

                if alloc["rbs"] > 0:
                    print(f"[DQN] t={t_now} | {st['label']:8s} | "
                          f"CQI:{st['cqi']:2d} | RBs:{alloc['rbs']:3d} | "
                          f"Act:{act_str:8s} | Lat:{lat_ms:.2f}ms | "
                          f"ε={dqn_agent.epsilon:.3f}")

                log_rows.append([
                    t_now, rnti, st["type"], st["5qi"], st["priority"],
                    st["cqi"], st["buffer_bytes"], round(lat_ms, 3), st["pdb_ms"],
                    act_str, alloc["rbs"], round(raw_tbs, 0),
                    round(tput, 4), round(dqn_agent.epsilon, 4),
                    round(last_reward, 4)
                ])

            if log_rows:
                with open(global_ue_state.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(log_rows)

            # Periodic training summary every 100 steps
            if dqn_agent.steps % 100 == 0 and dqn_agent.loss_log:
                avg_loss   = sum(dqn_agent.loss_log[-100:]) / len(dqn_agent.loss_log[-100:])
                avg_reward = sum(dqn_agent.reward_log[-100:]) / len(dqn_agent.reward_log[-100:])
                print(f"[DQN-TRAIN] step={dqn_agent.steps} | "
                      f"ε={dqn_agent.epsilon:.3f} | "
                      f"loss={avg_loss:.4f} | "
                      f"reward={avg_reward:.3f}")

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

print(f"\n{'='*60}")
print(" DQN Scheduler xApp (DICCT 2025)")
print(f" Logging → {global_ue_state.csv_file}")
print(f" state={STATE_DIM} | action={ACTION_DIM} | γ={GAMMA} | ε={EPSILON_START}→{EPSILON_END}")
print(f"{'='*60}\n")
print("Press Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Shutting down...")
finally:
    for i in range(len(mac_hndlr)):
        ric.rm_report_mac_sm(mac_hndlr[i])
        ric.rm_report_rlc_sm(rlc_hndlr[i])

    if HAS_TORCH:
        torch.save(dqn_agent.policy.state_dict(), "dqn_policy.pt")
        print("[INFO] Saved model → dqn_policy.pt")

    with open("dqn_train_log.json", "w") as f:
        json.dump({
            "rewards":       dqn_agent.reward_log[-1000:],
            "losses":        dqn_agent.loss_log[-1000:],
            "final_epsilon": dqn_agent.epsilon,
            "total_steps":   dqn_agent.steps,
        }, f, indent=2)
    print("[INFO] Saved training log → dqn_train_log.json")
    print("[INFO] xApp stopped.")
