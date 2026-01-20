#!/usr/bin/env bash
# Sends an heartbeat message over CAN bus to indicate Raspberry is running.
IFACE="eduart-can2"
CAN_ID="580"         # Power Management Board Output ID
SEQ_NUM=01           # Heartbeat Sequence Number
SLEEP_SEC=1          # 1 second interval
CAN_SYS_HEARTBEAT=FD # Heartbeat Command Byte

# Sending until interrupted
while true; do
  printf -v SEQ_NUM_HEX "%02X" "$SEQ_NUM"
  cansend "$IFACE" "${CAN_ID}#${CAN_SYS_HEARTBEAT}${SEQ_NUM_HEX}"
  SEQ_NUM=$(( (SEQ_NUM + 1) & 0xFF ))
  sleep "$SLEEP_SEC"
done
