#!/bin/bash

############################################
# OAI FULL AUTOMATION SCRIPT
# Author: ChatGPT
# Updated: Support for up to 10 UEs
############################################

#########################
# CONFIGURATION SECTION
#########################

OAI_CN_PATH="$HOME/oai-cn5g-fed/docker-compose"

GNB_CONF="$HOME/openairinterface5g/targets/PROJECTS/GENERIC-NR-5GC/CONF/gnb.sa.band78.fr1.106PRB.usrpb210.conf"

RIC_PATH="$HOME/openairinterface5g/openair2/E2AP/flexric/build/examples/ric"

RAN_BUILD_PATH="$HOME/openairinterface5g/cmake_targets/ran_build/build"

# Tunnel changer script path
TUNNEL_SCRIPT="$HOME/Desktop/OAI_Results/Working/TrafficGenandUEAssign/rename_ue.sh"

############################################
# FUNCTION : YES/NO PROMPT
############################################

ask_yes_no() {
    while true; do
        read -p "$1 (y/n): " yn
        case $yn in
            [Yy]* ) return 0;;
            [Nn]* ) return 1;;
            * ) echo "Please answer yes or no.";;
        esac
    done
}

############################################
# START CORE NETWORK
############################################

if ask_yes_no "Start Core Network?"; then

    echo "Starting Core Network..."

    cd "$OAI_CN_PATH" || exit

    docker compose -f docker-compose-basic-nrf.yaml up -d

    echo "Waiting 15 seconds for AMF..."
    sleep 15

    echo "Fetching AMF IP..."

    AMF_IP=$(docker inspect oai-amf | grep '"IPAddress"' | head -1 | awk -F '"' '{print $4}')

    echo "AMF IP Detected: $AMF_IP"

    echo "Updating gNB configuration..."

    sed -i "s/ipv4 = \".*\";/ipv4 = \"$AMF_IP\";/g" "$GNB_CONF"

    echo "gNB config updated successfully."

fi

############################################
# START RIC SERVER
############################################

if ask_yes_no "Start E2 RIC Server?"; then

    echo "Starting nearRT-RIC..."

    gnome-terminal -- bash -c "
    cd $RIC_PATH;
    ./nearRT-RIC;
    exec bash"

fi

############################################
# START GNB
############################################

if ask_yes_no "Start gNB?"; then

    echo "Starting gNB..."

    gnome-terminal -- bash -c "
    cd $RAN_BUILD_PATH;
    sudo ./nr-softmodem \
    -O ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/gnb.sa.band78.fr1.106PRB.usrpb210.conf \
    --rfsim --sa \
    --gNBs.[0].min_rxtxtime 4 \
    --gNBs.[0].enable_sdap 1;
    exec bash"

fi

############################################
# START UEs
############################################

if ask_yes_no "Start UEs?"; then

    # UPDATE: Changed prompt limit to 10
    read -p "How many UEs do you want to start (1-10)? : " NUM_UE

    # UPDATE: Changed conditional check to greater than 10
    if [[ $NUM_UE -lt 1 || $NUM_UE -gt 10 ]]; then
        echo "Invalid UE count."
        exit 1
    fi

    ############################################
    # START DEDICATED TUNNEL TERMINAL
    ############################################

    TMP_FIFO="/tmp/oai_tunnel_fifo"

    rm -f "$TMP_FIFO"
    mkfifo "$TMP_FIFO"

    gnome-terminal --title="OAI Tunnel Renamer" -- bash -c "
    while true
    do
        if read line < $TMP_FIFO; then
            eval \"\$line\"
        fi
    done
    exec bash"

    sleep 2

    ############################################
    # START UE LOOP
    ############################################

    for ((i=1; i<=NUM_UE; i++))
    do

        echo "======================================="
        echo "Starting UE $i"
        echo "======================================="

        if [[ $i -eq 1 ]]; then
            UE_CONF="ue.conf"
        else
            UE_CONF="ue${i}.conf"
        fi

        ############################################
        # START UE TERMINAL
        ############################################

        gnome-terminal --title="UE-$i" -- bash -c "
        cd $RAN_BUILD_PATH;
        sudo ./nr-uesoftmodem \
        -r 106 \
        --numerology 1 \
        --band 78 \
        -C 3619200000 \
        --rfsim \
        --sa \
        -O ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/$UE_CONF;
        exec bash"

        ############################################
        # WAIT FOR UE INIT
        ############################################

        echo "Waiting 15 seconds for UE $i..."
        sleep 15

        ############################################
        # RUN TUNNEL RENAME SCRIPT
        ############################################

        echo "Executing rename_ue.sh $i"

        echo "cd \"$HOME/Desktop/OAI_Results/Working/TrafficGenandUEAssign\" && ./rename_ue.sh $i" > "$TMP_FIFO"

        ############################################
        # WAIT BEFORE NEXT UE
        ############################################

        echo "Waiting 5 seconds before next UE..."
        sleep 5

    done

fi

############################################
# FINISHED
############################################

echo "======================================="
echo " OAI AUTOMATION COMPLETED SUCCESSFULLY "
echo "======================================="
