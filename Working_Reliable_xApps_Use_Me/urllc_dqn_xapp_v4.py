"""
urllc_dqn_xapp.py  –  v3.0
============================
xApp: Latency-Aware Network Slicing for 5G URLLC Applications
Based on: "Latency-Aware Network Slicing for 5G URLLC Applications:
           Design and Optimization Strategies" (DICCT 2025)

v3.0 changes (aligned to LAPF_Enhanced_v2.py patterns)
--------------------------------------------------------
  RNTI
    • register_ue() called directly inside both MAC and RLC callbacks,
      identical to LAPF's GlobalUEState.register_ue() pattern.
    • UE index assigned as  idx = len(rnti_order) % 8  (same as LAPF).
    • print() used for registration confirmation (same as LAPF).

  CSV
    • CSV opened once at startup in __init__ with header row written
      immediately (same as LAPF GlobalUEState.__init__).
    • Rows batched into log_rows list then written with writerows()
      inside an append-mode open() per MAC indication cycle (same as LAPF).
    • Column set matches LAPF exactly, plus DQN-specific additions
      (DQN_Action, Epsilon, Reward).

  Console logging
    • print() used throughout for live output, same as LAPF —
      no logging module indirection that can buffer.
    • [DQN] tag prefix on every console line, matching LAPF's
      [SDAP], [LAPF], [RLC ERROR], [MAC ERROR] style.
    • se_bits_per_rb and wb_cqi both persisted (stale-hold) exactly
      as in LAPF: only updated when OAI reports a non-zero value.
    • MCS→CQI fallback derivation copied verbatim from LAPF.
    • HARQ-pending flag detection copied from LAPF.

  Paper alignment (DICCT 2025 §II) — unchanged from v2
    Step 1  random weight init          DQNNet (Kaiming uniform)
    Step 2  ε-greedy                    select_action()
    Step 3  reward function             compute_reward()
    Step 4  replay + Bellman + target   learn()
    Step 5  continuous ε decay          every learn() call

Run:
    python3 -u urllc_dqn_xapp.py

8-UE Topology (matches traffic_gen8.sh)
  UE idx  Interface       Label      Type    5QI  Priority  PDB (ms)
  ------  --------------  ---------  ------  ---  --------  --------
  0       oaitun_ue_1     URLLC_1    URLLC   86   18        5.0
  1       oaitun_ue_2     eMBB_1     eMBB    9    90        300.0
  2       oaitun_ue_3     mMTC_1     mMTC    70   55        200.0
  3       oaitun_ue_4     URLLC_2    URLLC   86   18        5.0
  4       oaitun_ue_5     eMBB_2     eMBB    9    90        300.0
  5       oaitun_ue_6     mMTC_2     mMTC    70   55        200.0
  6       oaitun_ue_7     URLLC_3    URLLC   86   18        5.0
  7       oaitun_ue_8     URLLC_4    URLLC   86   18        5.0
"""

import sys
import os
# Unbuffered stdout — identical effect to running  python3 -u
os.environ.setdefault("PYTHONUNBUFFERED", "1")
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

import csv
import math
import re
import random
import signal
import subprocess
import time
import traceback
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional

import numpy as np

# ── Optional: xapp_sdk (NearRT-RIC) ──────────────────────────────────────────
try:
    import xapp_sdk as ric
    RIC_SDK_AVAILABLE = True
except ImportError:
    RIC_SDK_AVAILABLE = False

# ── Optional: PyTorch DQN ─────────────────────────────────────────────────────
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

# =============================================================================
# QoS MAPPING  –  fixed 10-UE topology
# =============================================================================
QOS_MAPPING = {
    0: {"type": "URLLC", "label": "URLLC_1", "5qi": 86, "priority": 18,  "pdb_ms": 5.0},
    1: {"type": "eMBB",  "label": "eMBB_1",  "5qi": 9,  "priority": 90,  "pdb_ms": 300.0},
    2: {"type": "mMTC",  "label": "mMTC_1",  "5qi": 70, "priority": 55,  "pdb_ms": 200.0},
    3: {"type": "URLLC", "label": "URLLC_2", "5qi": 86, "priority": 18,  "pdb_ms": 5.0},
    4: {"type": "eMBB",  "label": "eMBB_2",  "5qi": 9,  "priority": 90,  "pdb_ms": 300.0},
    5: {"type": "mMTC",  "label": "mMTC_2",  "5qi": 70, "priority": 55,  "pdb_ms": 200.0},
    6: {"type": "URLLC", "label": "URLLC_3", "5qi": 86, "priority": 18,  "pdb_ms": 5.0},
    7: {"type": "URLLC", "label": "URLLC_4", "5qi": 86, "priority": 18,  "pdb_ms": 5.0},
    8: {"type": "URLLC", "label": "URLLC_5", "5qi": 86, "priority": 18,  "pdb_ms": 5.0}, # NEW
    9: {"type": "URLLC", "label": "URLLC_6", "5qi": 86, "priority": 18,  "pdb_ms": 5.0}, # NEW
}

# =============================================================================
# CONSTANTS
# =============================================================================
SLOT_DURATION_MS = 0.5          # ms  (30 kHz SCS)
SLOT_DURATION_S  = 0.5e-3       # seconds
IFACE_PREFIX     = "oaitun_ue_"
NUM_UES          = 10
EPSILON_FLOAT    = 1e-6         # small float guard (not to be confused with DQN ε)

# CSV path
CSV_LOG_PATH = "urllc_dqn_log.csv"

# DQN hyper-parameters  (paper §II)
STATE_DIM      = 12
NUM_ACTIONS    = 10    # (action+1)×5 RBs → 5..50
GAMMA          = 0.99
LR             = 1e-3
BATCH_SIZE     = 64
REPLAY_SIZE    = 10_000
TARGET_UPDATE  = 50
EPSILON_START  = 1.0
EPSILON_END    = 0.05
EPSILON_DECAY  = 0.995

# Console print sampling: print a DQN decision line every N TTIs per UE.
# LAPF prints every grant (every active-UE MAC indication).
# Set PRINT_EVERY = 1 to match LAPF exactly (one line per UE per callback).
# Higher values reduce terminal spam on slow terminals; 1 is recommended
# for live monitoring so the console never appears frozen.
PRINT_EVERY = 1

# =============================================================================
# MCS → CQI lookup table (3GPP TS 38.214, SE-matched)
# -----------------------------------------------------------------------------
# Derived from:
#   MCS table : Table 5.1.3.2-1  (64QAM, indices 0-28)
#   CQI table : Table 5.1.3.1-1  (CQI indices 1-15)
#
# Method: for each MCS index compute SE = Qm × (R/1024), then select
# the CQI index with the closest SE value (min |SE_cqi - SE_mcs|).
#
# Used ONLY as a fallback for CQI logging when wb_cqi == 0 (not yet
# reported on this PUCCH cycle).  The RB allocator uses se_bits_per_rb
# = dl_curr_tbs / dl_sched_rb directly from OAI and is independent of
# this table.
#
# MCS_TO_CQI[mcs_index] → cqi_index  (mcs: 0..28, cqi: 1..13)
# =============================================================================
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

# =============================================================================
# INTERFACE WATCHER
# =============================================================================

def _get_iface_ip(iface: str) -> Optional[str]:
    """Return IPv4 address of a tunnel interface, or None."""
    try:
        out = subprocess.check_output(
            ["ip", "-4", "addr", "show", iface],
            stderr=subprocess.DEVNULL, text=True,
        )
        m = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)", out)
        return m.group(1) if m else None
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def wait_for_traffic(num_ues: int = NUM_UES) -> Dict[int, str]:
    """
    Block until ALL oaitun_ue_1..N have an IPv4 address.
    Prints a status line every second — terminal never looks frozen.
    Returns {ue_index: ip_address}.
    """
    print("=" * 62)
    print("  [DQN] Waiting for traffic_gen8.sh …")
    print(f"  [DQN] Monitoring {IFACE_PREFIX}1 .. {IFACE_PREFIX}{num_ues}")
    print("  [DQN] Start traffic_gen8.sh now  (Ctrl+C to abort)")
    print("=" * 62)

    while True:
        ip_map: Dict[int, str] = {}
        missing: List[str] = []

        for i in range(num_ues):
            iface = f"{IFACE_PREFIX}{i + 1}"
            ip    = _get_iface_ip(iface)
            if ip:
                ip_map[i] = ip
            else:
                missing.append(iface)

        if not missing:
            print(f"[DQN] ✓ All {num_ues} UE interfaces are UP:")
            for i, ip in ip_map.items():
                qos = QOS_MAPPING[i]
                print(f"  {IFACE_PREFIX}{i+1}  {qos['label']:<8}  "
                      f"{qos['type']:<5}  {ip}")
            return ip_map

        # Print every poll (1 s) — same as LAPF's continuous feedback
        up = num_ues - len(missing)
        print(f"  [DQN] {up}/{num_ues} UP – waiting for: {', '.join(missing)}")
        time.sleep(1.0)


# =============================================================================
# GLOBAL UE STATE  (mirrors LAPF's GlobalUEState)
# =============================================================================

class GlobalUEState:
    """
    Central per-UE state store — pattern identical to LAPF GlobalUEState.

    RNTI registration:
      register_ue(rnti) is called from both MAC and RLC callbacks whenever
      a new RNTI appears, exactly as LAPF does.  UE index is assigned as
        idx = len(rnti_order) % 8
      which maps RNTIs to QOS_MAPPING in attach order — same as LAPF.

    Persisted fields (stale-hold, copied from LAPF):
      se_bits_per_rb  — updated only when dl_curr_tbs > 0 and dl_sched_rb > 0
      cqi             — updated only when wb_cqi > 0; MCS→CQI fallback used
                        when wb_cqi is absent (verbatim LAPF logic)
      harq_pending    — from dl_harq list/scalar
    """

    def __init__(self, csv_path: str = CSV_LOG_PATH):
        self.states:     Dict[int, dict] = {}   # rnti → state dict
        self.rnti_order: List[int]       = []   # insertion order
        self.csv_file    = csv_path

        # Open CSV and write header — same pattern as LAPF __init__
        with open(self.csv_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "tstamp", "RNTI", "TrafficType", "5QI", "Priority",
                "CQI", "Buffer_Bytes", "Latency_ms", "Deadline_ms",
                "HARQ_Rate", "DQN_Action", "RBs_Granted",
                "Served_Bits", "Throughput_Mbps", "Epsilon", "Reward",
            ])
        print(f"[DQN] CSV logger → {os.path.abspath(csv_path)}")

    def register_ue(self, rnti: int):
        """
        Register a new RNTI if not already known.
        Prints a confirmation line matching LAPF's [SDAP] Registered message.
        """
        if rnti in self.states:
            return

        idx = len(self.rnti_order) % NUM_UES
        qos = QOS_MAPPING[idx]

        self.states[rnti] = {
            "type":           qos["type"],
            "label":          qos["label"],
            "5qi":            qos["5qi"],
            "priority":       qos["priority"],
            "pdb_ms":         qos["pdb_ms"],
            "deadline_slots": math.ceil(qos["pdb_ms"] / SLOT_DURATION_MS),
            # RLC-populated fields
            "buffer_bytes":   0,
            "latency_us":     0,
            # Persisted MAC fields (stale-hold — LAPF pattern)
            "cqi":            0,       # wb_cqi; updated only when > 0
            "se_bits_per_rb": 0.0,     # dl_curr_tbs/dl_sched_rb; updated only on real grant
            "harq_pending":   False,
            # DQN carry-over
            "prev_state":     None,
            "prev_action":    None,
            # Session stats
            "reward_sum":     0.0,
            "deadline_miss":  0,
            "tti_count":      0,
        }
        self.rnti_order.append(rnti)

        # Console — same format as LAPF's [SDAP] Registered line
        print(f"[SDAP] Registered RNTI {rnti:#06x} as {qos['label']} "
              f"(5QI: {qos['5qi']}, "
              f"deadline: {self.states[rnti]['deadline_slots']} slots, "
              f"pdb: {qos['pdb_ms']} ms)")


# Singleton — shared between MAC and RLC callbacks, same as LAPF
global_ue_state = GlobalUEState()


# =============================================================================
# MAC / RLC DATA CLASSES  (field names match attri.txt exactly)
# =============================================================================

@dataclass
class MACStats:
    rnti:               int   = 0
    frame:              int   = 0
    slot:               int   = 0
    dl_mcs1:            float = 0.0
    dl_mcs2:            float = 0.0
    ul_mcs1:            float = 0.0
    ul_mcs2:            float = 0.0
    dl_bler:            float = 0.0
    ul_bler:            float = 0.0
    dl_sched_rb:        int   = 0
    ul_sched_rb:        int   = 0
    dl_curr_tbs:        int   = 0
    ul_curr_tbs:        int   = 0
    dl_aggr_tbs:        int   = 0
    ul_aggr_tbs:        int   = 0
    dl_aggr_bytes_sdus: int   = 0
    ul_aggr_bytes_sdus: int   = 0
    dl_num_harq:        int   = 0
    ul_num_harq:        int   = 0
    dl_harq:            list  = field(default_factory=list)
    ul_harq:            list  = field(default_factory=list)
    wb_cqi:             int   = 0
    bsr:                int   = 0
    phr:                float = 0.0
    pucch_snr:          float = 0.0
    pusch_snr:          float = 0.0
    dl_aggr_sdus:       int   = 0
    ul_aggr_sdus:       int   = 0
    dl_aggr_prb:        int   = 0
    ul_aggr_prb:        int   = 0
    dl_aggr_retx_prb:   int   = 0
    ul_aggr_retx_prb:   int   = 0


@dataclass
class RLCStats:
    rnti:                 int   = 0
    rbid:                 int   = 0
    mode:                 str   = "AM"
    txpdu_bytes:          int   = 0
    txpdu_pkts:           int   = 0
    txpdu_retx_bytes:     int   = 0
    txpdu_retx_pkts:      int   = 0
    txpdu_wt_ms:          float = 0.0
    txpdu_dd_bytes:       int   = 0
    txpdu_dd_pkts:        int   = 0
    txpdu_segmented:      int   = 0
    txpdu_status_bytes:   int   = 0
    txpdu_status_pkts:    int   = 0
    txsdu_bytes:          int   = 0
    txsdu_pkts:           int   = 0
    txsdu_avg_time_to_tx: float = 0.0
    txsdu_wt_us:          float = 0.0
    txbuf_occ_bytes:      int   = 0
    txbuf_occ_pkts:       int   = 0
    rxpdu_bytes:          int   = 0
    rxpdu_pkts:           int   = 0
    rxpdu_dd_bytes:       int   = 0
    rxpdu_dd_pkts:        int   = 0
    rxpdu_dup_bytes:      int   = 0
    rxpdu_dup_pkts:       int   = 0
    rxpdu_ow_bytes:       int   = 0
    rxpdu_ow_pkts:        int   = 0
    rxpdu_status_bytes:   int   = 0
    rxpdu_status_pkts:    int   = 0
    rxsdu_bytes:          int   = 0
    rxsdu_pkts:           int   = 0
    rxsdu_dd_bytes:       int   = 0
    rxsdu_dd_pkts:        int   = 0
    rxbuf_occ_bytes:      int   = 0
    rxbuf_occ_pkts:       int   = 0


# =============================================================================
# FEATURE EXTRACTION  →  DQN state vector  (paper §II Step 1)
# =============================================================================

def extract_state(mac: MACStats, rlc: RLCStats) -> np.ndarray:
    """
    12-dimensional normalised state vector.

    Index  Feature                        Normalisation
    -----  -----------------------------  ---------------------------
    0      wb_cqi (persisted)             / 15
    1      dl_bler                        clip [0, 1]
    2      ul_bler                        clip [0, 1]
    3      dl_mcs1                        / 28
    4      ul_mcs1                        / 28
    5      bsr                            / 150 000   clip [0, 1]
    6      phr  (−23..+40 dBm)           (phr+23)/63  clip [0, 1]
    7      pusch_snr (−20..+30 dB)       (snr+20)/50  clip [0, 1]
    8      retx ratio                     clip [0, 1]
    9      txpdu_wt_ms                    / 100        clip [0, 1]
    10     txbuf_occ_bytes                / 200 000    clip [0, 1]
    11     txsdu_avg_time_to_tx           / 50         clip [0, 1]
    """
    retx_ratio = rlc.txpdu_retx_bytes / (rlc.txpdu_bytes + 1)
    return np.array([
        mac.wb_cqi / 15.0,
        np.clip(mac.dl_bler, 0.0, 1.0),
        np.clip(mac.ul_bler, 0.0, 1.0),
        mac.dl_mcs1 / 28.0,
        mac.ul_mcs1 / 28.0,
        np.clip(mac.bsr / 150_000.0, 0.0, 1.0),
        np.clip((mac.phr + 23.0) / 63.0, 0.0, 1.0),
        np.clip((mac.pusch_snr + 20.0) / 50.0, 0.0, 1.0),
        np.clip(retx_ratio, 0.0, 1.0),
        np.clip(rlc.txpdu_wt_ms / 100.0, 0.0, 1.0),
        np.clip(rlc.txbuf_occ_bytes / 200_000.0, 0.0, 1.0),
        np.clip(rlc.txsdu_avg_time_to_tx / 50.0, 0.0, 1.0),
    ], dtype=np.float32)


# =============================================================================
# REWARD FUNCTION  (paper §II Step 3)
# =============================================================================

def compute_reward(latency_ms: float, deadline_ms: float,
                   harq_rate: float, rbs_granted: int,
                   served_bits: float, traffic_type: str) -> float:
    """
    Reward is always in (0, 1] — no negative values, no UE crashes.

    Three weighted components (paper §II Step 3), all guaranteed in [0, 1]:

      0.5 × latency_score  =  max(0, 1 − clip(latency/deadline, 0, 1))
                               Clips the ratio at 1 (not 2) so the term
                               floors at 0 instead of going to −1 when the
                               deadline is missed.  Numerically identical to
                               the original for latency ≤ deadline.

      0.3 × harq_reward    =  harq_rate  (= 1 − dl_bler)      [unchanged]

      0.2 × eff_reward     =  clip(served_bits /               [unchanged]
                                   (RBs × 168 RE × 8), 0, 1)

    URLLC deadline-miss penalty (replaces the additive −2.0):
      When latency > deadline the entire base reward is scaled by 0.1.
      This keeps the reward positive while creating a ~23× distinguishability
      gap between miss (mean ≈ 0.031) and non-miss (mean ≈ 0.72) cases,
      giving the DQN a strong, stable gradient to prioritise late URLLC UEs.
    """
    latency_score  = max(0.0, 1.0 - np.clip(
                        latency_ms / max(deadline_ms, EPSILON_FLOAT), 0.0, 1.0))
    harq_reward    = harq_rate
    eff_reward     = np.clip(served_bits / (rbs_granted * 168 * 8 + 1), 0.0, 1.0)
    base_reward    = 0.5 * latency_score + 0.3 * harq_reward + 0.2 * eff_reward
    if traffic_type == "URLLC" and latency_ms > deadline_ms:
        base_reward *= 0.1                  # miss penalty: scales down, never negates
    return float(base_reward)


# =============================================================================
# DQN AGENT  (paper §II Steps 1–5)
# =============================================================================

if TORCH_AVAILABLE:

    class DQNNet(nn.Module):
        def __init__(self, in_dim: int, out_dim: int):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(in_dim, 128), nn.ReLU(),
                nn.Linear(128, 128),   nn.ReLU(),
                nn.Linear(128, out_dim),
            )

        def forward(self, x):
            return self.net(x)

    class DQNAgent:
        """DQN: ε-greedy · experience replay · Bellman · target-net."""

        def __init__(self):
            self.device     = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.policy_net = DQNNet(STATE_DIM, NUM_ACTIONS).to(self.device)
            self.target_net = DQNNet(STATE_DIM, NUM_ACTIONS).to(self.device)
            self.target_net.load_state_dict(self.policy_net.state_dict())
            self.target_net.eval()
            self.optimizer  = optim.Adam(self.policy_net.parameters(), lr=LR)
            self.loss_fn    = nn.MSELoss()
            self.memory     = deque(maxlen=REPLAY_SIZE)
            self.epsilon    = EPSILON_START
            self.step_count = 0
            print(f"[DQN] DQNAgent ready  [PyTorch {torch.__version__}  device={self.device}]")

        def select_action(self, state: np.ndarray) -> int:
            if random.random() < self.epsilon:
                return random.randrange(NUM_ACTIONS)
            s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            with torch.no_grad():
                return int(self.policy_net(s).argmax(dim=1).item())

        def remember(self, s, a, r, s2, done):
            self.memory.append((s, a, r, s2, done))

        def learn(self):
            if len(self.memory) < BATCH_SIZE:
                return
            batch = random.sample(self.memory, BATCH_SIZE)
            states, actions, rewards, next_states, dones = zip(*batch)
            S  = torch.FloatTensor(np.array(states)).to(self.device)
            A  = torch.LongTensor(actions).unsqueeze(1).to(self.device)
            R  = torch.FloatTensor(rewards).to(self.device)
            S2 = torch.FloatTensor(np.array(next_states)).to(self.device)
            D  = torch.FloatTensor(dones).to(self.device)
            q_vals   = self.policy_net(S).gather(1, A).squeeze(1)
            with torch.no_grad():
                q_next   = self.target_net(S2).max(dim=1)[0]
                q_target = R + GAMMA * q_next * (1.0 - D)
            loss = self.loss_fn(q_vals, q_target)
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
            self.step_count += 1
            self.epsilon = max(EPSILON_END, self.epsilon * EPSILON_DECAY)
            if self.step_count % TARGET_UPDATE == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())

else:
    class DQNAgent:   # type: ignore[no-redef]
        """Linear-Q fallback (no PyTorch)."""
        def __init__(self):
            self.W          = np.zeros((NUM_ACTIONS, STATE_DIM), dtype=np.float64)
            self.memory     = deque(maxlen=REPLAY_SIZE)
            self.epsilon    = EPSILON_START
            self.step_count = 0
            print("[DQN] WARNING: PyTorch not found – using linear-Q fallback")

        def _q(self, s):       return self.W @ s
        def select_action(self, state):
            return random.randrange(NUM_ACTIONS) if random.random() < self.epsilon \
                   else int(np.argmax(self._q(state)))
        def remember(self, s, a, r, s2, done):
            self.memory.append((s, a, r, s2, done))
        def learn(self):
            if len(self.memory) < BATCH_SIZE:
                return
            for s, a, r, s2, done in random.sample(self.memory, BATCH_SIZE):
                td = r + (0.0 if done else GAMMA * np.max(self._q(s2))) - self._q(s)[a]
                self.W[a] += LR * td * s
            self.step_count += 1
            self.epsilon = max(EPSILON_END, self.epsilon * EPSILON_DECAY)


# =============================================================================
# RLC CALLBACK  (mirrors LAPF RLCCallback)
# =============================================================================

if RIC_SDK_AVAILABLE:

    class RLCCallback(ric.rlc_cb):
        def __init__(self):
            ric.rlc_cb.__init__(self)

        def handle(self, ind):
            try:
                if len(ind.rb_stats) == 0:
                    return

                # Reset all known UE buffers before fresh read — same as LAPF
                for rnti in global_ue_state.states:
                    global_ue_state.states[rnti]["buffer_bytes"] = 0
                    global_ue_state.states[rnti]["latency_us"]   = 0

                for rb in ind.rb_stats:
                    rnti = rb.rnti
                    global_ue_state.register_ue(rnti)   # real RNTI registered here

                    global_ue_state.states[rnti]["buffer_bytes"] += rb.txbuf_occ_bytes

                    # Maximum HOL latency across bearers — identical to LAPF
                    current_max = global_ue_state.states[rnti]["latency_us"]
                    global_ue_state.states[rnti]["latency_us"] = max(
                        current_max, rb.txsdu_wt_us
                    )

            except Exception as e:
                print(f"\n[RLC ERROR] {e}")
                traceback.print_exc()


# =============================================================================
# MAC CALLBACK  (mirrors LAPF MACCallback; adds DQN decision layer)
# =============================================================================

if RIC_SDK_AVAILABLE:

    class MACCallback(ric.mac_cb):
        """
        Called by the RIC SDK every 10 ms with live MAC indications.

        Per-TTI flow:
          1. Read OAI MAC attributes — persist se_bits_per_rb, cqi, harq_pending
          2. Run DQN decision per active UE
          3. Write CSV rows (batch append, same as LAPF)
          4. Print sampled console lines
        """

        def __init__(self, agent: DQNAgent):
            ric.mac_cb.__init__(self)
            self._agent      = agent
            self._tti        = 0       # global TTI counter (ASCII only)
            self._t_start    = time.time()
            self._first_data = True    # banner flag: print once when traffic starts

        def handle(self, ind):
            try:
                if len(ind.ue_stats) == 0:
                    return

                t_now      = ind.tstamp
                log_rows   = []         # batched CSV rows — same as LAPF
                active_ues = []

                # ── 1. Read OAI MAC attributes ────────────────────────────────
                for ue in ind.ue_stats:
                    rnti = ue.rnti
                    global_ue_state.register_ue(rnti)   # real RNTI registered here
                    st = global_ue_state.states[rnti]

                    # se_bits_per_rb: stale-hold — update only on real grant
                    # (same fix as LAPF BUG FIX A)
                    try:
                        tbs_now   = int(ue.dl_curr_tbs)
                        sched_rbs = int(ue.dl_sched_rb)
                        if tbs_now > 0 and sched_rbs > 0:
                            st["se_bits_per_rb"] = tbs_now / sched_rbs
                        # else: retain last known value
                    except Exception:
                        pass

                    # wb_cqi: stale-hold + MCS_TO_CQI LUT fallback
                    # wb_cqi is refreshed on the PUCCH cycle, not every MAC
                    # slot.  Value is 0 between reporting cycles.
                    # When non-zero, clamp to [1, 15] and use directly.
                    # When zero, derive from dl_mcs1 via the 3GPP SE-matched
                    # MCS_TO_CQI LUT (valid range 0-28 only; values outside
                    # indicate no active DL grant and are skipped).
                    # In both cases, only store when a valid non-zero value
                    # is obtained -- preserving the last known CQI across
                    # slots where neither source is available.
                    try:
                        cqi_now = int(ue.wb_cqi)
                        if cqi_now > 0:
                            # wb_cqi valid this cycle -- clamp and use directly
                            st["cqi"] = min(max(cqi_now, 1), 15)
                        else:
                            # wb_cqi not yet reported -- fall back to dl_mcs1
                            mcs = int(ue.dl_mcs1)
                            if 0 <= mcs <= 28:      # valid MCS range only
                                st["cqi"] = MCS_TO_CQI[mcs]
                            # else: mcs out of range -> no active grant; retain
                            #       previous cqi unchanged
                    except Exception:
                        pass   # retain previous cqi on any read error

                    # HARQ pending flag (verbatim from LAPF)
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

                # ── Banner: first time we see real traffic ─────────────────────
                if self._first_data:
                    self._first_data = False
                    print("=" * 68)
                    print("  [DQN] Traffic detected — DQN decisions now flowing")
                    print(f"  [DQN] Logging to {global_ue_state.csv_file}")
                    print(f"  [DQN] Console: 1 line per UE per {PRINT_EVERY} MAC indication(s)")
                    print("=" * 68)

                # ── 2. DQN decision per active UE ─────────────────────────────
                # Build RNTI→UE-indication lookup from loop 1 so every UE
                # gets its own MAC stats (not the last ue in ind.ue_stats).
                ue_by_rnti = {ue.rnti: ue for ue in ind.ue_stats}

                for rnti in active_ues:
                    st = global_ue_state.states[rnti]

                    # Build MACStats / RLCStats from persisted state
                    # (buffer_bytes and latency_us come from RLC callback)
                    mac = _build_mac_from_state(ue_by_rnti[rnti], st)
                    rlc = _build_rlc_from_state(st)

                    state      = extract_state(mac, rlc)
                    action     = self._agent.select_action(state)
                    rbs_granted = (action + 1) * 5   # 5..50 RBs

                    # Latency estimate: HOL + service time + HARQ round-trips
                    lat_ms    = st["latency_us"] / 1000.0
                    harq_rtt  = mac.dl_num_harq * SLOT_DURATION_MS
                    latency_ms = max(0.1, lat_ms + harq_rtt)

                    harq_rate   = float(np.clip(1.0 - mac.dl_bler, 0.0, 1.0))
                    # served_bits uses persisted se_bits_per_rb (same as LAPF)
                    served_bits = rbs_granted * st["se_bits_per_rb"] if st["se_bits_per_rb"] > 0 \
                                  else st["buffer_bytes"] * 8
                    throughput_mbps = (served_bits / 1e6) / SLOT_DURATION_S

                    reward = compute_reward(
                        latency_ms, st["pdb_ms"], harq_rate,
                        rbs_granted, served_bits, st["type"],
                    )

                    # Experience replay — store transition; learn once per
                    # callback cycle (outside this loop) to avoid 8× ε decay.
                    if st["prev_state"] is not None:
                        self._agent.remember(
                            st["prev_state"], st["prev_action"],
                            reward, state, False,
                        )

                    st["prev_state"]  = state
                    st["prev_action"] = action

                    # Session stats
                    st["reward_sum"]    += reward
                    st["tti_count"]     += 1
                    if st["type"] == "URLLC" and latency_ms > st["pdb_ms"]:
                        st["deadline_miss"] += 1

                    # Console line — matches LAPF [LAPF] format exactly.
                    # Gated on global _tti so all UEs print together every
                    # PRINT_EVERY MAC-indication cycles (same rhythm as LAPF's
                    # per-grant print — continuous output once traffic flows).
                    if self._tti % PRINT_EVERY == 0:
                        miss = " *** MISS ***" if (st["type"] == "URLLC" and
                                                   latency_ms > st["pdb_ms"]) else ""
                        print(
                            f"[DQN] tstamp: {t_now} | UE: {rnti:#06x} ({st['label']}) | "
                            f"CQI: {st['cqi']} | lat: {latency_ms:.2f}/{st['pdb_ms']:.0f} ms | "
                            f"RBs: {rbs_granted} | action: {action} | "
                            f"reward: {reward:+.3f} | ε: {self._agent.epsilon:.4f}{miss}"
                        )

                    # Batch CSV row — same pattern as LAPF log_rows.append()
                    log_rows.append([
                        t_now,
                        f"{rnti:#06x}",
                        st["type"],
                        st["5qi"],
                        st["priority"],
                        st["cqi"],
                        st["buffer_bytes"],
                        round(latency_ms, 4),
                        st["pdb_ms"],
                        round(harq_rate, 4),
                        action,
                        rbs_granted,
                        round(served_bits, 0),
                        round(throughput_mbps, 6),
                        round(self._agent.epsilon, 6),
                        round(reward, 6),
                    ])

                # ── 3. Single Bellman update per callback cycle ────────────────
                self._agent.learn()

                # ── 4. Write CSV batch — same pattern as LAPF ─────────────────
                if log_rows:
                    with open(global_ue_state.csv_file, mode="a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerows(log_rows)

                # ── 5. Heartbeat every ~1 s (100 callbacks × 10 ms = 1 s) ──────
                self._tti += 1
                if self._tti % 100 == 0:
                    elapsed = time.time() - self._t_start
                    print(f"[DQN] TTI {self._tti:7d}  {elapsed:6.1f}s  "
                          f"ε={self._agent.epsilon:.4f}  "
                          f"mapped_UEs={len(global_ue_state.rnti_order)}/{NUM_UES}")

            except Exception as e:
                print(f"\n[MAC ERROR] {e}")
                traceback.print_exc()


# =============================================================================
# SIMULATION PROVIDER  (used when RIC SDK is not available)
# =============================================================================

class SimProvider:
    """
    Markov ON/OFF burst model — mirrors traffic_gen8.sh parameters.
    Calls global_ue_state.register_ue() exactly as the live callbacks do,
    so the RNTI→UE-index assignment path is identical in both modes.
    """

    BURST = {
        "URLLC": {"p_off2on": 0.05, "p_on2off": 0.30, "burst_base": 800,
                  "refill_delay": 2,  "lambda": 2400},
        "eMBB":  {"p_off2on": 0.03, "p_on2off": 0.15, "burst_base": 12000,
                  "refill_delay": 30, "lambda": 3000},
        "mMTC":  {"p_off2on": 0.01, "p_on2off": 0.40, "burst_base": 1000,
                  "refill_delay": 30, "lambda": 120},
    }

    def __init__(self, ip_map: Dict[int, str]):
        self._slot     = 0
        self._state_on: Dict[int, bool] = {}
        self._cooldown: Dict[int, int]  = {}

        # Register synthetic RNTIs in interface order so QoS is correct
        self._rntis: List[int] = []
        for i in sorted(ip_map.keys()):
            rnti = 0x1000 + i
            global_ue_state.register_ue(rnti)  # same call path as live callbacks
            self._rntis.append(rnti)
            self._state_on[rnti] = False
            self._cooldown[rnti] = 0

    @property
    def rntis(self) -> List[int]:
        return self._rntis

    def _burst_bytes(self, rnti: int) -> int:
        st  = global_ue_state.states[rnti]
        bm  = self.BURST[st["type"]]
        ttype = st["type"]

        if self._state_on[rnti]:
            if random.random() < bm["p_on2off"]:
                self._state_on[rnti] = False
        else:
            if random.random() < bm["p_off2on"]:
                self._state_on[rnti] = True

        if self._cooldown[rnti] > 0:
            self._cooldown[rnti] -= 1

        b = 0
        if self._state_on[rnti]:
            if self._cooldown[rnti] == 0:
                b = int(bm["burst_base"] * (0.8 + 0.4 * random.random()))
                self._cooldown[rnti] = bm["refill_delay"]
            if ttype == "mMTC" and b == 0:
                b = int(bm["lambda"] * random.random() * 0.5)
        else:
            self._cooldown[rnti] = 0
            if ttype == "URLLC" and random.random() < 0.05:
                b = int(64 * random.random())
        return b

    def get_mac(self, rnti: int) -> MACStats:
        st    = global_ue_state.states[rnti]
        ttype = st["type"]
        load  = 0.3 + 0.5 * abs(np.sin(self._slot / 20.0))

        if ttype == "URLLC":
            cqi, mcs = random.randint(10, 15), random.randint(18, 28)
            bler, bsr = random.uniform(0.00, 0.05), random.randint(500, 5_000)
        elif ttype == "eMBB":
            cqi, mcs = random.randint(6, 14), random.randint(10, 24)
            bler, bsr = random.uniform(0.00, 0.10), random.randint(5_000, 80_000)
        else:
            cqi, mcs = random.randint(2, 10), random.randint(2, 14)
            bler, bsr = random.uniform(0.00, 0.20), random.randint(100, 2_000)

        tbs = int(mcs * 50 * load)

        # Persist wb_cqi and se_bits_per_rb via stale-hold — same as live path
        if cqi > 0:
            global_ue_state.states[rnti]["cqi"] = cqi
        if tbs > 0 and int(mcs * load) > 0:
            global_ue_state.states[rnti]["se_bits_per_rb"] = tbs / max(int(mcs * load), 1)

        return MACStats(
            rnti=rnti, frame=self._slot // 10, slot=self._slot % 10,
            dl_mcs1=mcs,  dl_mcs2=max(0, mcs - 2),
            ul_mcs1=mcs,  ul_mcs2=max(0, mcs - 2),
            dl_bler=bler, ul_bler=bler,
            dl_sched_rb=int(mcs * load),      ul_sched_rb=int(mcs * load * 0.5),
            dl_curr_tbs=tbs,                  ul_curr_tbs=tbs // 2,
            dl_aggr_tbs=tbs * 10,             ul_aggr_tbs=tbs * 5,
            dl_aggr_bytes_sdus=tbs * 8,       ul_aggr_bytes_sdus=tbs * 4,
            dl_num_harq=random.randint(1, 4), ul_num_harq=random.randint(1, 4),
            wb_cqi=cqi, bsr=bsr,
            phr=random.uniform(10, 35),
            pucch_snr=random.uniform(5, 30),  pusch_snr=random.uniform(5, 30),
            dl_aggr_sdus=random.randint(10, 100),
            ul_aggr_sdus=random.randint(5, 50),
            dl_aggr_prb=int(mcs * load * 10), ul_aggr_prb=int(mcs * load * 5),
            dl_aggr_retx_prb=random.randint(0, 3),
            ul_aggr_retx_prb=random.randint(0, 2),
        )

    def get_rlc(self, rnti: int, mac: MACStats) -> RLCStats:
        st          = global_ue_state.states[rnti]
        ttype       = st["type"]
        burst_bytes = self._burst_bytes(rnti)
        tx_bytes    = min(burst_bytes, mac.dl_curr_tbs) if mac.dl_curr_tbs > 0 else burst_bytes
        retx_r      = mac.dl_bler * random.uniform(0.8, 1.2)
        buf         = mac.bsr + burst_bytes

        # Update buffer and latency in global state — same as RLC callback does
        global_ue_state.states[rnti]["buffer_bytes"] = buf
        if ttype == "URLLC":
            wt_us = random.uniform(100, 2000)
        elif ttype == "eMBB":
            wt_us = random.uniform(1000, 15000)
        else:
            wt_us = random.uniform(2000, 25000)
        global_ue_state.states[rnti]["latency_us"] = wt_us

        return RLCStats(
            rnti=rnti, rbid=1, mode="AM",
            txpdu_bytes=tx_bytes,
            txpdu_pkts=max(1, tx_bytes // 1400),
            txpdu_retx_bytes=int(tx_bytes * retx_r),
            txpdu_retx_pkts=max(0, int(tx_bytes * retx_r // 1400)),
            txpdu_wt_ms=wt_us / 1000.0,
            txsdu_bytes=tx_bytes,
            txsdu_pkts=max(1, tx_bytes // 1400),
            txsdu_avg_time_to_tx=(wt_us / 1000.0) * random.uniform(0.9, 1.1),
            txsdu_wt_us=wt_us * random.uniform(0.1, 0.5),
            txbuf_occ_bytes=buf,
            txbuf_occ_pkts=max(1, buf // 1400),
            rxpdu_bytes=tx_bytes // 2,
            rxpdu_pkts=max(1, tx_bytes // 2800),
            rxsdu_bytes=tx_bytes // 2,
            rxsdu_pkts=max(1, tx_bytes // 2800),
            rxbuf_occ_bytes=buf // 4,
            rxbuf_occ_pkts=max(0, buf // (4 * 1400)),
        )

    def advance_slot(self):
        self._slot += 1


# =============================================================================
# HELPER — build MACStats / RLCStats from persisted global state
# (used in live MACCallback where we don't have a full RLCStats object)
# =============================================================================

def _build_mac_from_state(ue_ind, st: dict) -> MACStats:
    """Reconstruct a MACStats from the live UE indication + persisted cqi."""
    try:
        return MACStats(
            rnti=ue_ind.rnti,
            frame=int(ue_ind.frame),      slot=int(ue_ind.slot),
            dl_mcs1=float(ue_ind.dl_mcs1), dl_mcs2=float(ue_ind.dl_mcs2),
            ul_mcs1=float(ue_ind.ul_mcs1), ul_mcs2=float(ue_ind.ul_mcs2),
            dl_bler=float(ue_ind.dl_bler), ul_bler=float(ue_ind.ul_bler),
            dl_sched_rb=int(ue_ind.dl_sched_rb),
            ul_sched_rb=int(ue_ind.ul_sched_rb),
            dl_curr_tbs=int(ue_ind.dl_curr_tbs),
            ul_curr_tbs=int(ue_ind.ul_curr_tbs),
            dl_num_harq=int(ue_ind.dl_num_harq),
            ul_num_harq=int(ue_ind.ul_num_harq),
            dl_harq=list(ue_ind.dl_harq),
            wb_cqi=st["cqi"],               # persisted value, never 0 after first update
            bsr=int(ue_ind.bsr),
            phr=float(ue_ind.phr),
            pucch_snr=float(ue_ind.pucch_snr),
            pusch_snr=float(ue_ind.pusch_snr),
        )
    except Exception:
        return MACStats(wb_cqi=st["cqi"])


def _build_rlc_from_state(st: dict) -> RLCStats:
    """Build a minimal RLCStats from the persisted buffer/latency state."""
    lat_ms = st["latency_us"] / 1000.0
    buf    = st["buffer_bytes"]
    return RLCStats(
        txbuf_occ_bytes=buf,
        txbuf_occ_pkts=max(1, buf // 1400),
        txpdu_wt_ms=lat_ms,
        txsdu_avg_time_to_tx=lat_ms,
        txsdu_wt_us=float(st["latency_us"]),
        txsdu_bytes=buf,
        txpdu_bytes=buf,
    )


# =============================================================================
# SIMULATION RUN LOOP  (used when RIC SDK is not available)
# =============================================================================

def run_sim(agent: DQNAgent, ip_map: Dict[int, str]):
    """
    Simulation loop — replicates the same MAC→DQN→CSV→print flow as the
    live MACCallback.  Runs at 0.5 ms / TTI with slot-accurate timing.

    Console output matches LAPF: one line per UE per PRINT_EVERY slots
    (default 1) so the terminal always shows live traffic — no silent gaps.
    """
    provider = SimProvider(ip_map)

    print("=" * 62)
    print(f"  [DQN] Simulation mode ACTIVE  –  {NUM_UES} UEs  –  Ctrl+C to stop")
    print(f"  [DQN] Console: 1 line per UE per {PRINT_EVERY} slot(s) — matches LAPF")
    print("=" * 62)
    for rnti in provider.rntis:
        st  = global_ue_state.states[rnti]
        idx = global_ue_state.rnti_order.index(rnti)
        ip  = ip_map.get(idx, "–")
        print(f"  UE{idx+1}  {rnti:#06x}  {st['label']:<8}  {st['type']:<5}  "
              f"5QI={st['5qi']:2d}  prio={st['priority']:2d}  "
              f"pdb={st['pdb_ms']:.0f} ms  IP={ip}")
    print("=" * 62)

    running    = True
    tti        = 0
    t_start    = time.time()
    next_slot  = time.perf_counter()
    first_data = True   # banner flag

    def _stop(sig, frame):
        nonlocal running
        print(f"\n[DQN] Signal {sig} – stopping after current TTI …")
        running = False

    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while running:
            t_now    = datetime.now(timezone.utc).isoformat(timespec="microseconds")
            log_rows = []

            for rnti in provider.rntis:
                mac = provider.get_mac(rnti)
                rlc = provider.get_rlc(rnti, mac)
                st  = global_ue_state.states[rnti]

                state       = extract_state(mac, rlc)
                action      = agent.select_action(state)
                rbs_granted = (action + 1) * 5

                harq_rtt   = mac.dl_num_harq * SLOT_DURATION_MS   # one slot per HARQ process (no ×8)
                latency_ms = max(0.1, rlc.txpdu_wt_ms + rlc.txsdu_avg_time_to_tx + harq_rtt)
                harq_rate  = float(np.clip(1.0 - mac.dl_bler, 0.0, 1.0))
                served_bits = (rbs_granted * st["se_bits_per_rb"]
                               if st["se_bits_per_rb"] > 0
                               else rlc.txsdu_bytes * 8)
                throughput_mbps = (served_bits / 1e6) / SLOT_DURATION_S

                reward = compute_reward(
                    latency_ms, st["pdb_ms"], harq_rate,
                    rbs_granted, served_bits, st["type"],
                )

                if st["prev_state"] is not None:
                    agent.remember(st["prev_state"], st["prev_action"], reward, state, False)

                st["prev_state"]  = state
                st["prev_action"] = action
                st["reward_sum"]  += reward
                st["tti_count"]   += 1
                if st["type"] == "URLLC" and latency_ms > st["pdb_ms"]:
                    st["deadline_miss"] += 1

                # Console — same continuous format as LAPF per-grant print.
                # Gated on global tti so all 8 UEs print together every
                # PRINT_EVERY slots — terminal never looks frozen.
                if tti % PRINT_EVERY == 0:
                    miss = " *** MISS ***" if (st["type"] == "URLLC" and
                                               latency_ms > st["pdb_ms"]) else ""
                    print(
                        f"[DQN] tstamp: {t_now} | UE: {rnti:#06x} ({st['label']}) | "
                        f"CQI: {st['cqi']} | lat: {latency_ms:.2f}/{st['pdb_ms']:.0f} ms | "
                        f"RBs: {rbs_granted} | action: {action} | "
                        f"reward: {reward:+.3f} | ε: {agent.epsilon:.4f}{miss}"
                    )

                log_rows.append([
                    t_now,
                    f"{rnti:#06x}",
                    st["type"],
                    st["5qi"],
                    st["priority"],
                    st["cqi"],
                    st["buffer_bytes"],
                    round(latency_ms, 4),
                    st["pdb_ms"],
                    round(harq_rate, 4),
                    action,
                    rbs_granted,
                    round(served_bits, 0),
                    round(throughput_mbps, 6),
                    round(agent.epsilon, 6),
                    round(reward, 6),
                ])

            # Single Bellman update per slot (outside per-UE loop)
            agent.learn()

            # Batch write — same as LAPF / live MACCallback
            if log_rows:
                with open(global_ue_state.csv_file, mode="a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerows(log_rows)

                # Banner: first TTI that actually produces rows = traffic flowing
                if first_data:
                    first_data = False
                    print("=" * 68)
                    print("  [DQN] Traffic flowing — DQN decisions active")
                    print(f"  [DQN] Logging to {global_ue_state.csv_file}")
                    print(f"  [DQN] Console: 1 line per UE per {PRINT_EVERY} slot(s)")
                    print("=" * 68)

            provider.advance_slot()
            tti += 1

            # Heartbeat every ~1 s (2000 slots × 0.5 ms = 1 s)
            if tti % 2000 == 0:
                elapsed = time.time() - t_start
                print(f"[DQN] TTI {tti:7d}  {elapsed:6.1f}s  "
                      f"ε={agent.epsilon:.4f}  "
                      f"mapped_UEs={len(global_ue_state.rnti_order)}/{NUM_UES}")

            # Slot-accurate timing
            next_slot += SLOT_DURATION_S
            sleep_s    = next_slot - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            elif sleep_s < -0.050:
                next_slot = time.perf_counter()

    except KeyboardInterrupt:
        pass

    _shutdown(agent, tti, t_start)


def _shutdown(agent: DQNAgent, tti: int, t_start: float):
    elapsed = time.time() - t_start

    # Flush CSV
    print("")
    print("=" * 62)
    print("  [DQN] SESSION SUMMARY")
    print(f"  Duration  : {elapsed:.1f} s")
    print(f"  TTIs      : {tti}  ({tti / max(elapsed, 1e-6):.0f} slots/s)")
    print(f"  DQN steps : {agent.step_count}  ε={agent.epsilon:.4f}")
    print("")
    print(f"  {'UE':<4}  {'RNTI':<8}  {'Label':<8}  {'Type':<6}  "
          f"{'Avg Rwrd':>8}  {'Miss/TTI':>10}  {'Total Miss':>10}")

    for rnti in global_ue_state.rnti_order:
        st   = global_ue_state.states[rnti]
        avg  = st["reward_sum"] / max(st["tti_count"], 1)
        miss = st["deadline_miss"]
        rate = miss / max(st["tti_count"], 1)
        idx  = global_ue_state.rnti_order.index(rnti)
        print(f"  UE{idx+1:<2}  {rnti:#06x}  {st['label']:<8}  {st['type']:<6}  "
              f"{avg:+8.4f}  {rate:10.6f}  {miss:10d}")

    print("=" * 62)
    print(f"[DQN] CSV → {os.path.abspath(global_ue_state.csv_file)}")
    print("[DQN] xApp stopped.")


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":

    # Step 1: wait for all 8 UE tunnel interfaces to come up
    ip_map = wait_for_traffic(NUM_UES)

    # Step 2: create DQN agent
    agent = DQNAgent()

    if RIC_SDK_AVAILABLE:
        # ── Live RIC SDK path ─────────────────────────────────────────────────
        print("[DQN] RIC SDK found – connecting to E2 nodes …")
        ric.init()
        conn = ric.conn_e2_nodes()
        assert len(conn) > 0, "[DQN] ERROR: No E2 nodes connected!"

        mac_hndlr = []
        rlc_hndlr = []

        for i in range(len(conn)):
            rlc_cb  = RLCCallback()
            mac_cb  = MACCallback(agent)
            hndlr_r = ric.report_rlc_sm(conn[i].id, ric.Interval_ms_10, rlc_cb)
            hndlr_m = ric.report_mac_sm(conn[i].id, ric.Interval_ms_10, mac_cb)
            rlc_hndlr.append(hndlr_r)
            mac_hndlr.append(hndlr_m)
            time.sleep(1)

        print(f"[DQN] DQN xApp running. Logging to {global_ue_state.csv_file} …")
        print("[DQN] Press Ctrl+C to stop.")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[DQN] Ctrl+C detected. Shutting down …")
        finally:
            print("[DQN] Cleaning up MAC and RLC reports …")
            for i in range(len(mac_hndlr)):
                ric.rm_report_mac_sm(mac_hndlr[i])
                ric.rm_report_rlc_sm(rlc_hndlr[i])
            print("[DQN] xApp stopped.")

    else:
        # ── Simulation path ───────────────────────────────────────────────────
        print("[DQN] RIC SDK not found – running Markov burst simulator")
        run_sim(agent, ip_map)
