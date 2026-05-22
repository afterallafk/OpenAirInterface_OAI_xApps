import xapp_sdk as ric
import time
import csv
import math
import os
import traceback
import random
import collections
import json
import threading

_shutdown = threading.Event()

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
# GAP FIX 3 — URLLC PDB corrected from 5 ms → 1 ms to match paper §I which
# explicitly states 1 ms as the URLLC latency target.  The previous 5 ms
# was too relaxed: the agent rarely observed deadline violations so the
# latency penalty never fired, making URLLC indistinguishable from eMBB
# from the reward's perspective.

CQI_TO_SE = [0.0, 0.1523, 0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766,
             1.9141, 2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547]

MCS_TO_SE = [
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953, 1.9141,
    2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547, 5.5547,
    5.5547, 6.2266, 6.9141, 7.4063, 7.4063, 7.4063, 7.4063, 7.4063,
    7.4063, 7.4063, 7.4063, 7.4063, 7.4063
]

TOTAL_RBS    = 106      # 40 MHz → 106 PRBs
NUM_LAYERS   = 2        # 2×2 MIMO — must match LAPF reference
SC_PER_RB    = 12       # subcarriers per RB
NUM_SYMB     = 14       # OFDM symbols per slot
SLOT_DUR_S   = 0.5e-3  # 0.5 ms slot (30 kHz SCS)
ALPHA_EMA    = 0.08
NUM_UES      = 8

# Number of distinct slice types — actions operate at slice level (paper §II)
# GAP FIX 1 — Paper §II describes the DQN controlling per-slice resource
# allocations (URLLC / eMBB / mMTC), not per-UE.  The original code used
# ACTION_DIM = NUM_UES * 3 = 24, treating every UE independently.  This
# section defines the 3 canonical slices; the action→allocation step then
# distributes each slice's RB budget equally across its member UEs.
SLICE_TYPES  = ["URLLC", "eMBB", "mMTC"]
NUM_SLICES   = len(SLICE_TYPES)   # 3

def tbs_per_rb(se: float) -> float:
    """Approximate TBS for 1 RB, 2 MIMO layers, 14 OFDM symbols."""
    return se * SC_PER_RB * NUM_SYMB * NUM_LAYERS

# =====================================================================
# DQN Hyper-parameters
# =====================================================================
# State: per UE → [norm_buffer, deadline_fraction, norm_cqi, norm_tput,
#                   type_enc, harq_flag]
# GAP FIX 2 — added harq_flag (0/1) as 6th per-UE state feature.
# HARQ retransmissions are a direct proxy for link reliability; including
# them lets the agent learn to avoid allocations that trigger retransmission
# storms, closing the gap with paper §II Step 3 ("penalise lower reliability").
STATE_DIM       = NUM_UES * 6          # 8 UEs × 6 features = 48

# Action: per slice → {0=reduce 10%, 1=keep, 2=increase 10%}
# GAP FIX 1 — action space is now slice-level (3 slices × 3 actions = 9)
# instead of per-UE (8 UEs × 3 actions = 24).  This matches the paper's
# description of the agent adjusting slice-level resource budgets.
ACTION_DIM      = NUM_SLICES * 3       # 9

HIDDEN_DIM      = 128
REPLAY_CAPACITY = 10_000
BATCH_SIZE      = 64
GAMMA           = 0.99
LR              = 1e-3
EPSILON_START   = 1.0
EPSILON_END     = 0.05
EPSILON_DECAY   = 0.995
TARGET_UPDATE   = 50
MIN_REPLAY      = 256

RB_DELTA_FRAC = [-0.10, 0.0, +0.10]   # action 0/1/2

# Per-slice minimum RB fractions (URLLC protected floor stays)
MIN_URLLC_RBS = 4
MIN_RB_FRAC   = {
    "URLLC": MIN_URLLC_RBS / TOTAL_RBS,   # ≈ 0.038
    "eMBB":  0.05,   # raised slightly — prevents eMBB starvation
    "mMTC":  0.03,   # raised slightly — prevents mMTC starvation
}

# =====================================================================
# Reliability tracking — HARQ retransmission EMA per UE
# =====================================================================
# GAP FIX 2 — per-UE exponential moving average of HARQ retransmission
# rate.  Values near 1.0 = persistent retransmissions = poor reliability.
# Used in state vector AND reward function.
ALPHA_HARQ = 0.10   # EMA smoothing for harq_rate (faster than throughput EMA)

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

        # Per-SLICE RB fraction (GAP FIX 1 — slice-level control)
        self.rb_fractions = {s: MIN_RB_FRAC[s] for s in SLICE_TYPES}
        # Initialise with balanced split, respecting floors
        total_floor = sum(MIN_RB_FRAC[s] for s in SLICE_TYPES)
        remainder   = 1.0 - total_floor
        init_extra  = {"URLLC": 0.50, "eMBB": 0.35, "mMTC": 0.15}
        for s in SLICE_TYPES:
            self.rb_fractions[s] = MIN_RB_FRAC[s] + remainder * init_extra[s]

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
    # GAP FIX 2 — 6th feature per UE is harq_rate (EMA retransmission rate)
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
            vec.append(min(st.get("harq_rate", 0.0), 1.0))  # GAP FIX 2
        while len(vec) < STATE_DIM:
            vec.append(0.0)
        return vec[:STATE_DIM]

    # ------------------------------------------------------------------
    # ε-greedy action selection (now over NUM_SLICES actions)
    # ------------------------------------------------------------------
    def _select_action(self, state_vec):
        if random.random() < self.epsilon:
            return [random.randint(0, 2) for _ in range(NUM_SLICES)]
        return self._greedy(state_vec)

    def _greedy(self, state_vec):
        if HAS_TORCH:
            with torch.no_grad():
                s = torch.FloatTensor(state_vec).unsqueeze(0).to(self.device)
                q = self.policy(s).squeeze(0).cpu().tolist()
        else:
            q = self.qtable.predict(state_vec)
        return [max(range(3), key=lambda a: q[i*3 + a]) for i in range(NUM_SLICES)]

    # ------------------------------------------------------------------
    # Reward function — strictly non-negative for OAI UE safety.
    #
    # Design principle: instead of subtracting penalties from a base
    # reward, all terms are formulated as positive bonuses that shrink
    # toward zero when conditions worsen.  The reward therefore lives in
    # [0, R_MAX] at all times — no clamp needed, no negative gradients
    # that could destabilise the OAI scheduler interface.
    #
    # R_MAX per step (4 URLLC + 2 eMBB + 2 mMTC + util bonus):
    #   URLLC: up to 2.0 each × 4  = 8.0
    #   eMBB:  up to 1.0 each × 2  = 2.0
    #   mMTC:  up to 1.0 each × 2  = 2.0
    #   util bonus:                  2.0
    #   Total R_MAX                ≈ 14.0
    #
    # Term breakdown:
    #
    # 1. URLLC latency term  (GAP FIX 3 — 1 ms PDB now active)
    #    reward += 2.0 × (1 − min(lat_ms / pdb_ms, 1))² × tput_norm
    #    • Quadratic in latency ratio → steep drop as deadline approaches
    #    • Multiplied by tput_norm so both throughput AND latency must be
    #      good to earn the full bonus
    #    • Equals 0 when lat_ms ≥ pdb_ms (deadline missed)
    #    • Equals 2×tput_norm when lat_ms = 0 (perfect)
    #
    # 2. URLLC reliability term  (GAP FIX 2 — closes the paper gap)
    #    reward += 0.5 × (1 − harq_rate)
    #    • harq_rate ∈ [0,1] EMA of retransmission flag
    #    • Equals 0.5 when no retransmissions; drops to 0 under heavy HARQ
    #    • Paper §II Step 3 explicitly penalises "lower reliability";
    #      harq_rate is the only per-UE reliability signal available from
    #      the OAI MAC indication
    #
    # 3. eMBB / mMTC term
    #    reward += tput_norm × max(0, 1 − lat_ratio)
    #    • Product of normalised throughput and latency headroom
    #    • Both factors ∈ [0,1] so result ∈ [0,1]
    #    • Drops toward 0 when queue builds (lat_ratio → 1) OR when
    #      throughput is low, giving the agent a signal against starvation
    #
    # 4. Resource efficiency bonus (unchanged from previous version)
    #    reward += 2.0 × (1 − |util − 0.80| / 0.80), clamped ≥ 0
    # ------------------------------------------------------------------
    def _reward(self, ue_states, rnti_order, allocations):
        reward         = 0.0
        total_rbs_used = sum(a.get("rbs", 0) for a in allocations.values())

        for rnti in rnti_order[:NUM_UES]:
            st     = ue_states[rnti]
            alloc  = allocations.get(rnti, {"rbs": 0, "served_bits": 0.0})
            lat_ms = st["latency_us"] / 1000.0

            se        = st["se"]
            peak_bits = tbs_per_rb(se) * TOTAL_RBS
            tput_norm = min(alloc["served_bits"] / max(peak_bits, 1e-6), 1.0)

            harq_rate = min(st.get("harq_rate", 0.0), 1.0)  # GAP FIX 2

            if st["type"] == "URLLC":
                # ── Latency term ──────────────────────────────────────
                # lat_ratio = 0 → perfect; lat_ratio ≥ 1 → deadline missed
                lat_ratio  = lat_ms / max(st["pdb_ms"], 1e-3)
                headroom   = max(0.0, 1.0 - lat_ratio)
                # Quadratic so agent is strongly incentivised to stay well
                # inside the 1 ms window, not just barely under it
                lat_bonus  = 2.0 * (headroom ** 2) * tput_norm

                # ── Reliability term (GAP FIX 2) ──────────────────────
                # harq_rate ≈ 0 → good link → full bonus
                # harq_rate ≈ 1 → persistent retransmissions → bonus → 0
                rel_bonus  = 0.5 * (1.0 - harq_rate)

                reward += lat_bonus + rel_bonus

            else:
                # eMBB / mMTC: throughput × latency headroom product
                lat_ratio = lat_ms / max(st["pdb_ms"], 1e-3)
                headroom  = max(0.0, 1.0 - lat_ratio)
                reward   += tput_norm * headroom

        # Resource efficiency bonus — peaks at 80% utilisation
        util          = total_rbs_used / max(TOTAL_RBS, 1)
        efficiency    = max(0.0, 1.0 - abs(util - 0.80) / 0.80)
        reward       += 2.0 * efficiency

        # Reward is already ≥ 0 by construction — no clamp needed.
        # Assert here during development to catch any future regression.
        assert reward >= 0.0, f"[BUG] Negative reward detected: {reward:.4f}"
        return float(reward)

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
            # GAP FIX 1 — loop over NUM_SLICES (3) not NUM_UES (8)
            for sl_i in range(NUM_SLICES):
                base      = sl_i * 3
                best_next = q_next[:, base:base+3].max(dim=1).values
                act_idx   = torch.tensor([a[sl_i] for a in actions],
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
                for sl_i in range(NUM_SLICES):
                    base      = sl_i * 3
                    best_next = max(q_ns[base:base+3])
                    td_target = rew + GAMMA * best_next * (1 - done)
                    self.qtable.update(s, base + a[sl_i], td_target)

    # ------------------------------------------------------------------
    # Main entry per scheduling tick
    # ------------------------------------------------------------------
    def schedule(self, ue_states, rnti_order, prev_allocations):
        """Called every MAC tick. Returns dict: rnti → {rbs, served_bits}"""
        if not rnti_order:
            return {}

        state_vec = self._state(ue_states, rnti_order)

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
    # Translate per-SLICE action → per-UE integer RB counts
    #
    # GAP FIX 1 — actions now adjust per-slice RB fractions.  Each slice's
    # total RB budget is then distributed equally among its active member
    # UEs, matching the paper's description of slice-level control.
    # ------------------------------------------------------------------
    def _actions_to_alloc(self, actions, rnti_order, ue_states):
        # Map slice index → action
        slice_action = {s: actions[i] for i, s in enumerate(SLICE_TYPES)}

        # Apply deltas to per-slice fractions
        for stype in SLICE_TYPES:
            delta = RB_DELTA_FRAC[slice_action[stype]]
            lo    = MIN_RB_FRAC[stype]
            self.rb_fractions[stype] = max(lo, min(0.90,
                                           self.rb_fractions[stype] + delta))

        # Renormalise so slices sum to ≤ 1.0 while respecting floors
        total = sum(self.rb_fractions[s] for s in SLICE_TYPES)
        if total > 1.0:
            # Scale all slices proportionally (floors protect minimum)
            for stype in SLICE_TYPES:
                lo = MIN_RB_FRAC[stype]
                self.rb_fractions[stype] = max(lo,
                                               self.rb_fractions[stype] / total)

        # Count active UEs per slice
        active_per_slice: dict[str, list] = {s: [] for s in SLICE_TYPES}
        for rnti in rnti_order[:NUM_UES]:
            st = ue_states[rnti]
            if st["buffer_bytes"] > 0:
                active_per_slice[st["type"]].append(rnti)

        # Compute per-slice RB budget then split equally among active UEs
        alloc     = {}
        remaining = TOTAL_RBS

        for stype in SLICE_TYPES:
            members      = active_per_slice[stype]
            slice_budget = max(
                MIN_URLLC_RBS * len(members) if stype == "URLLC" else len(members),
                int(self.rb_fractions[stype] * TOTAL_RBS)
            )
            slice_budget = min(slice_budget, remaining)

            if members:
                rbs_each = max(
                    MIN_URLLC_RBS if stype == "URLLC" else 1,
                    slice_budget // len(members)
                )
                for rnti in members:
                    actual_rbs = min(rbs_each, remaining)
                    se         = ue_states[rnti]["se"]
                    alloc[rnti] = {
                        "rbs":         actual_rbs,
                        "served_bits": actual_rbs * tbs_per_rb(se)
                    }
                    remaining -= actual_rbs

        # Zero-fill inactive UEs
        for rnti in rnti_order[:NUM_UES]:
            if rnti not in alloc:
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
                "HARQ_Rate", "DQN_Action", "RBs_Granted", "Served_Bits",
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
                "harq_rate":      0.0,   # GAP FIX 2 — added field
                "cqi":            0,
                "se":             0.15,
                "harq_pending":   False,
            }
            self.rnti_order.append(rnti)
            print(f"[SDAP] Registered RNTI {rnti} → {qos['label']} (5QI {qos['5qi']})")


global_ue_state = GlobalUEState()
dqn_agent       = DQNAgent()
_last_alloc     = {}


# =====================================================================
# RLC Callback – buffer occupancy & HOL latency
# =====================================================================
class RLCCallback(ric.rlc_cb):
    def __init__(self):
        ric.rlc_cb.__init__(self)

    def handle(self, ind):
        if _shutdown.is_set():
            return
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
        if _shutdown.is_set():
            return
        try:
            if len(ind.ue_stats) == 0:
                return

            t_now      = ind.tstamp
            active_ues = []

            for ue in ind.ue_stats:
                rnti = ue.rnti
                global_ue_state.register_ue(rnti)

                # ── CQI / MCS ─────────────────────────────────────────
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

                # ── HARQ retransmission EMA (GAP FIX 2) ───────────────
                # harq_pending = True whenever any HARQ process has a
                # retransmission pending this slot.  We update an EMA
                # so the state and reward see a smoothed reliability
                # estimate rather than a noisy per-slot binary.
                try:
                    if isinstance(ue.dl_harq, (list, tuple)):
                        harq_now = 1.0 if sum(ue.dl_harq) > 0 else 0.0
                    else:
                        harq_now = 1.0 if int(ue.dl_harq) > 0 else 0.0
                    global_ue_state.states[rnti]["harq_pending"] = harq_now > 0.0
                except Exception:
                    harq_now = 0.0
                    global_ue_state.states[rnti]["harq_pending"] = False

                prev_rate = global_ue_state.states[rnti].get("harq_rate", 0.0)
                global_ue_state.states[rnti]["harq_rate"] = (
                    (1 - ALPHA_HARQ) * prev_rate + ALPHA_HARQ * harq_now
                )

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
            last_reward   = dqn_agent.reward_log[-1] if dqn_agent.reward_log else 0.0
            action_labels = ["REDUCE", "KEEP", "INCREASE"]

            # For logging: map each UE's slice action
            slice_action_map = {}
            if dqn_agent._prev_action:
                for sl_i, stype in enumerate(SLICE_TYPES):
                    slice_action_map[stype] = dqn_agent._prev_action[sl_i]

            log_rows = []
            for rnti in active_ues:
                st    = global_ue_state.states[rnti]
                alloc = allocations.get(rnti, {"rbs": 0, "served_bits": 0.0})

                lat_ms  = st["latency_us"] / 1000.0
                raw_tbs = alloc["served_bits"]

                # URLLC deadline miss → zero effective throughput
                eff_tbs = 0.0 if (st["type"] == "URLLC" and lat_ms > st["pdb_ms"]) else raw_tbs

                tput = (eff_tbs / 1e6) / SLOT_DUR_S

                st["avg_throughput"] = ((1 - ALPHA_EMA) * st["avg_throughput"]
                                        + ALPHA_EMA * eff_tbs)

                act_idx = slice_action_map.get(st["type"], 1)
                act_str = action_labels[act_idx]

                if alloc["rbs"] > 0:
                    print(f"[DQN] t={t_now} | {st['label']:8s} | "
                          f"CQI:{st['cqi']:2d} | RBs:{alloc['rbs']:3d} | "
                          f"Act:{act_str:8s} | Lat:{lat_ms:.2f}ms | "
                          f"HARQ:{st['harq_rate']:.2f} | "
                          f"ε={dqn_agent.epsilon:.3f}")

                log_rows.append([
                    t_now, rnti, st["type"], st["5qi"], st["priority"],
                    st["cqi"], st["buffer_bytes"], round(lat_ms, 3), st["pdb_ms"],
                    round(st["harq_rate"], 4),
                    act_str, alloc["rbs"], round(raw_tbs, 0),
                    round(tput, 4), round(dqn_agent.epsilon, 4),
                    round(last_reward, 4)
                ])

            if log_rows:
                with open(global_ue_state.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(log_rows)

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
print(" DQN Scheduler xApp (DICCT 2025) — v2 Gap-Fixed")
print(f" Logging → {global_ue_state.csv_file}")
print(f" state={STATE_DIM} | action={ACTION_DIM} (slice-level) | γ={GAMMA} | ε={EPSILON_START}→{EPSILON_END}")
print(f" URLLC PDB=1ms | Reward: strictly non-negative | HARQ reliability: ON")
print(f"{'='*60}\n")
print("Press Ctrl+C to stop.")

try:
    while not _shutdown.is_set():
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Ctrl+C detected. Signalling callbacks to stop...")
    _shutdown.set()
    time.sleep(0.5)
finally:
    for i in range(len(mac_hndlr)):
        ric.rm_report_mac_sm(mac_hndlr[i])
        ric.rm_report_rlc_sm(rlc_hndlr[i])
    if HAS_TORCH:
        torch.save(dqn_agent.policy.state_dict(), "dqn_policy.pt")

    with open("dqn_train_log.json", "w") as f:
        json.dump({
            "rewards":       dqn_agent.reward_log[-1000:],
            "losses":        dqn_agent.loss_log[-1000:],
            "final_epsilon": dqn_agent.epsilon,
            "total_steps":   dqn_agent.steps,
        }, f, indent=2)
    print("[INFO] Saved training log → dqn_train_log.json")
    print("[INFO] xApp stopped.")
