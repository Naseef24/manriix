#!/bin/bash
# =============================================================================
# Manriix CAN Shutdown Script
# =============================================================================
# Placed in /lib/systemd/system-shutdown/
# Runs on every shutdown/reboot before devices are torn down.
# Brings down CAN2 and CAN3 interfaces cleanly.
# =============================================================================

/sbin/ip link set can2 down 2>/dev/null
/sbin/ip link set can3 down 2>/dev/null
