#!/bin/bash

echo "================================================"
echo "      5G UE Traffic Gen (9-UE via oai-ext-dn)   "
echo "================================================"

# ====================================================================
# 1. AUTO-DETECT CONNECTED UEs
# ====================================================================
declare -a UE_IPS

echo "Scanning for oaitun_ue interfaces..."
for intf in $(ip -o link show | awk -F': ' '{print $2}' | grep 'oaitun_ue'); do
    ip_addr=$(ip -4 addr show "$intf" 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
    if [ -n "$ip_addr" ]; then
        UE_IPS+=("$ip_addr")
        echo "  Detected $intf -> $ip_addr"
    fi
done

NUM_UES=${#UE_IPS[@]}

if [ "$NUM_UES" -eq 0 ]; then
    echo "[ERROR] No UEs detected. Are they connected?"
    exit 1
fi

echo "Total UEs detected: $NUM_UES"
echo "------------------------------------------------"

# ====================================================================
# 2. FIXED ROLE SEQUENCE (Aligned with MAC Scheduler)
# ====================================================================
FIXED_ROLES=(
    "URLLC" "URLLC" "URLLC" "URLLC" "URLLC"
    "eMBB"  "eMBB"  "eMBB"
    "mMTC"
)

# ====================================================================
# 3. ENVIRONMENT PREPARATION (Inside oai-ext-dn)
# ====================================================================
echo -e "\nChecking for iperf3 inside oai-ext-dn container..."
if ! docker exec oai-ext-dn command -v iperf3 &> /dev/null; then
    echo "iperf3 not found. Installing it now (this will take a moment)..."
    docker exec oai-ext-dn apt-get update -y > /dev/null
    docker exec oai-ext-dn apt-get install -y iperf3 > /dev/null
else
    echo "iperf3 is ready inside the Data Network."
fi

# Stop any lingering instances inside the container
docker exec oai-ext-dn pkill -f iperf3 2>/dev/null

# ====================================================================
# 4. EXECUTE TRAFFIC GENERATION
# ====================================================================
echo -e "\n===== INJECTING IPERF3 TRAFFIC INTO OAI CORE ====="

for i in "${!UE_IPS[@]}"; do
    ip="${UE_IPS[$i]}"
    
    role_idx=$((i % ${#FIXED_ROLES[@]}))
    role="${FIXED_ROLES[$role_idx]}"
    
    if [ "$role" == "URLLC" ]; then
        cmd="iperf3 -c $ip -u -b 500K -l 100 -t 0"
    elif [ "$role" == "eMBB" ]; then
        cmd="iperf3 -c $ip -u -b 5M -l 1000 -t 0"
    elif [ "$role" == "mMTC" ]; then
        cmd="iperf3 -c $ip -u -b 10K -l 50 -t 0"
    fi
    
    echo "Launching $role DL flow from Core to $ip ($cmd)"
    # Execute the command INSIDE the ext-dn container
    docker exec -d oai-ext-dn $cmd
done

echo -e "\nTraffic generation started successfully in the background!"

# Automatically kill the containerized traffic generators when script is stopped
trap 'echo -e "\nStopping iperf3 traffic in oai-ext-dn..."; docker exec oai-ext-dn pkill -f iperf3; exit 0' INT

echo "Traffic flowing indefinitely! Press Ctrl+C to stop."
while true; do
    sleep 1
done
