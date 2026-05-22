#!/bin/bash

echo "================================================"
echo "      5G UE Auto-Detection & Traffic Gen        "
echo "================================================"

# ====================================================================
# 1. AUTO-DETECT CONNECTED UEs
# ====================================================================
declare -a UE_IPS
declare -a UE_ROLES

echo "Scanning for oaitun_ue interfaces..."
# Find all interfaces matching 'oaitun_ue' and extract their IP addresses
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
# 2. SCENARIO SELECTION (MATLAB-ALIGNED)
# ====================================================================
echo -e "\n===== TRAFFIC SCENARIO SELECTION ====="
echo "1. All URLLC"
echo "2. All eMBB"
echo "3. All mMTC"
echo "4. Balanced    (33% URLLC / 33% eMBB / 34% mMTC)"
echo "5. URLLC-Heavy (60% URLLC / 20% eMBB / 20% mMTC)"
echo "6. eMBB-Heavy  (20% URLLC / 60% eMBB / 20% mMTC)"
echo "7. CUSTOM"
read -p "Select scenario (1-7): " choice

case $choice in
    1) P_URLLC=100; P_EMBB=0; P_MMTC=0 ;;
    2) P_URLLC=0; P_EMBB=100; P_MMTC=0 ;;
    3) P_URLLC=0; P_EMBB=0; P_MMTC=100 ;;
    4) P_URLLC=33; P_EMBB=33; P_MMTC=34 ;;
    5) P_URLLC=60; P_EMBB=20; P_MMTC=20 ;;
    6) P_URLLC=20; P_EMBB=60; P_MMTC=20 ;;
    7) 
        read -p "Enter % URLLC: " P_URLLC
        read -p "Enter % eMBB: " P_EMBB
        read -p "Enter % mMTC: " P_MMTC
        
        # Simple validation
        total=$((P_URLLC + P_EMBB + P_MMTC))
        if [ "$total" -ne 100 ]; then
            echo "[ERROR] Percentages must add up to 100. You entered $total."
            exit 1
        fi
        ;;
    *) echo "[ERROR] Invalid choice. Exiting."; exit 1 ;;
esac

# ====================================================================
# 3. RANDOM ROLE ASSIGNMENT
# ====================================================================
echo -e "\n===== ASSIGNING ROLES ====="
for ip in "${UE_IPS[@]}"; do
    rand=$((RANDOM % 100)) # Generates a number 0-99
    
    if [ "$rand" -lt "$P_URLLC" ]; then
        role="URLLC"
    elif [ "$rand" -lt "$((P_URLLC + P_EMBB))" ]; then
        role="eMBB"
    else
        role="mMTC"
    fi
    
    UE_ROLES+=("$role")
    printf "  Assigning %-15s -> %s\n" "$ip" "$role"
done

# ====================================================================
# 4. ENVIRONMENT PREPARATION
# ====================================================================
echo -e "\nChecking for iperf3 inside oai-ext-dn container..."
if ! docker exec oai-ext-dn command -v iperf3 &> /dev/null; then
    echo "iperf3 not found. Installing it now (this will take a moment)..."
    docker exec oai-ext-dn apt-get update -y > /dev/null
    docker exec oai-ext-dn apt-get install -y iperf3 > /dev/null
else
    echo "iperf3 is ready."
fi

# Stop any lingering instances
docker exec oai-ext-dn pkill -f iperf3 2>/dev/null

# ====================================================================
# 5. EXECUTE TRAFFIC GENERATION
# ====================================================================
echo -e "\n===== INJECTING IPERF3 TRAFFIC INTO OAI ====="

for i in "${!UE_IPS[@]}"; do
    ip="${UE_IPS[$i]}"
    role="${UE_ROLES[$i]}"
    
    # Define iperf3 profiles (UDP traffic, -t 0 for infinite duration)
    # Traffic sizes heavily reduced
    if [ "$role" == "URLLC" ]; then
        cmd="iperf3 -c $ip -u -b 500K -l 100 -t 0"
    elif [ "$role" == "eMBB" ]; then
        cmd="iperf3 -c $ip -u -b 5M -l 1000 -t 0"
    elif [ "$role" == "mMTC" ]; then
        cmd="iperf3 -c $ip -u -b 10K -l 50 -t 0"
    fi
    
    echo "Launching $role flow to $ip ($cmd)"
    docker exec -d oai-ext-dn $cmd
done

echo -e "\nTraffic generation started successfully in the background!"

# Automatically kill the background traffic generators when script is stopped
trap 'echo -e "\nStopping iperf3 traffic..."; docker exec oai-ext-dn pkill -f iperf3; exit 0' INT

echo "Traffic flowing indefinitely! Press Ctrl+C to stop."
while true; do
    sleep 1
done
