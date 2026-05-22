#!/bin/bash
if [ -z "$1" ]; then
  echo "Usage: ./rename_ue.sh <number>"
  exit 1
fi

NEW_NAME="oaitun_ue_$1"
echo "Renaming oaitun_ue1 to $NEW_NAME..."

sudo ip link set dev oaitun_ue1 down
sudo ip link set dev oaitun_ue1 name $NEW_NAME
sudo ip link set dev $NEW_NAME up

echo "Done! You can now launch the next UE."
