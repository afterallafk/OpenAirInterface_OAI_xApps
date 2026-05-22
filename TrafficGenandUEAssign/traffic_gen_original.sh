#!/bin/bash

echo "Auto-detecting 5G UE IP addresses..."

get_ip() {
    ip -4 addr show "$1" 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}'
}

# 3 UE setup
IP_URLLC=$(get_ip "oaitun_ue_1")
IP_EMBB=$(get_ip  "oaitun_ue_2")
IP_MMTC=$(get_ip  "oaitun_ue_3")

if [ -z "$IP_URLLC" ] || [ -z "$IP_EMBB" ] || [ -z "$IP_MMTC" ]; then
    echo "[ERROR] Could not detect all 3 UEs. Are they connected?"
    exit 1
fi

echo "------------------------------------------------"
echo "  UE1 URLLC : $IP_URLLC"
echo "  UE2 eMBB  : $IP_EMBB"
echo "  UE3 mMTC  : $IP_MMTC"
echo "------------------------------------------------"

# ====================================================================
# AUTO-INSTALL PYTHON IN CONTAINER IF MISSING
# ====================================================================
echo "Checking for Python3 inside oai-ext-dn container..."
if ! docker exec oai-ext-dn command -v python3 &> /dev/null; then
    echo "Python3 not found. Installing it now (this will take a moment)..."
    docker exec oai-ext-dn apt-get update -y
    docker exec oai-ext-dn apt-get install -y python3
else
    echo "Python3 is ready."
fi
# ====================================================================

# Stop any old instances running in the container
docker exec oai-ext-dn pkill -f traffic_gen.py 2>/dev/null

# Inject the MATLAB-aligned Python Traffic Generator into the container
docker exec -i oai-ext-dn sh -c 'cat > /tmp/traffic_gen.py' << 'EOF'
import sys
import time
import random
import socket

ip = sys.argv[1]
traffic_type = sys.argv[2]
port = 9999

# Native UDP socket to prevent process exhaustion
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# MATLAB 'Burst' Model Parameters
if traffic_type == "URLLC":
    p_off2on = 0.05
    p_on2off = 0.30
    burst_base = 800      # <--- REDUCED 10x (800 bytes per burst)
    refill_delay = 2
    lambda_val = 2400
elif traffic_type == "eMBB":
    p_off2on = 0.03
    p_on2off = 0.15
    burst_base = 12000    # <--- REDUCED 10x (12,000 bytes per burst)
    refill_delay = 30
    lambda_val = 3000
elif traffic_type == "mMTC":
    p_off2on = 0.01
    p_on2off = 0.40
    burst_base = 1000
    refill_delay = 30
    lambda_val = 120
else:
    print("Unknown TYPE")
    sys.exit(1)

state_on = False
cooldown = 0
slot_duration = 0.0005 # 0.5ms per slot (30 kHz SCS)

print(f"Starting {traffic_type} traffic to {ip}:{port}...")

next_slot_time = time.time()

while True:
    # 1. State Transitions (ON/OFF)
    if state_on:
        if random.random() < p_on2off:
            state_on = False
    else:
        if random.random() < p_off2on:
            state_on = True

    # 2. Cooldown Update
    if cooldown > 0:
        cooldown -= 1

    # 3. Payload Generation
    bytes_to_send = 0
    if state_on:
        if cooldown == 0:
            multiplier = 0.8 + 0.4 * random.random()
            bytes_to_send = int(burst_base * multiplier)
            cooldown = refill_delay
        
        # mMTC background noise
        if traffic_type == "mMTC" and bytes_to_send == 0:
            bytes_to_send = int(lambda_val * random.random() * 0.5)
    else:
        cooldown = 0
        # URLLC background noise
        if traffic_type == "URLLC" and random.random() < 0.05:
            bytes_to_send = int(64 * random.random())

    # 4. Transmit Payload (Chunked to prevent IP fragmentation)
    if bytes_to_send > 0:
        chunk_size = 1400
        sent = 0
        payload = b'A' * chunk_size
        while sent < bytes_to_send:
            send_now = min(chunk_size, bytes_to_send - sent)
            try:
                sock.sendto(payload[:send_now], (ip, port))
            except Exception:
                pass
            sent += send_now

    # 5. Perfect Timing Synchronization
    next_slot_time += slot_duration
    sleep_time = next_slot_time - time.time()
    
    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        # Self-healing: If Linux kernel scheduler falls behind by >50ms, 
        # reset target time to avoid flooding the network trying to "catch up"
        if sleep_time < -0.05: 
            next_slot_time = time.time()
EOF

echo "Injecting MATLAB-aligned Burst models into OAI network..."
docker exec -d oai-ext-dn python3 /tmp/traffic_gen.py $IP_URLLC URLLC
docker exec -d oai-ext-dn python3 /tmp/traffic_gen.py $IP_EMBB eMBB
docker exec -d oai-ext-dn python3 /tmp/traffic_gen.py $IP_MMTC mMTC

echo "Traffic flowing! Press Ctrl+C to stop."

# Automatically kill the background traffic generators when you stop this script
trap 'echo -e "\nStopping traffic..."; docker exec oai-ext-dn pkill -f traffic_gen.py; exit 0' INT

while true; do
    sleep 1
done
