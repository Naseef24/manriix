#!/bin/bash
# =============================================================================
# Manriix Startup Automation Script
# =============================================================================
# Checks ~/.manriix_automate flag. If set to "true", guides the operator
# through CAN2/CAN3 bring-up and launches the full navigation stack.
#
# Usage (manual testing):    bash manriix_startup.sh
# Systemd end goal:          See manriix_startup.service
# =============================================================================

FLAG_FILE="$HOME/.manriix_automate"
CAN_DETECT_TIMEOUT=60        # Seconds to wait for USB-to-CAN adapter to appear
CAN_POLL_INTERVAL=1          # Poll every N seconds
# ROS2_SETUP="/opt/ros/humble/setup.bash"
# MANRIIX_WS="$HOME/manriix2_ws/install/setup.bash"

# --- Colours for terminal output ---------------------------------------------
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()    { echo -e "${GREEN}[Manriix]${NC} $1"; }
warn()   { echo -e "${YELLOW}[Manriix]${NC} $1"; }
error()  { echo -e "${RED}[Manriix]${NC} $1"; }

# =============================================================================
# 1. CHECK AUTOMATE FLAG
# =============================================================================
if [[ ! -f "$FLAG_FILE" ]]; then
    warn "Flag file '$FLAG_FILE' not found. Automation disabled."
    warn "Run 'manriix_set_automate.sh true' to enable."
    exit 0
fi

FLAG_VALUE=$(cat "$FLAG_FILE" | tr -d '[:space:]' | tr '[:upper:]' '[:lower:]')

if [[ "$FLAG_VALUE" != "true" ]]; then
    warn "Automate flag is set to '$FLAG_VALUE'. Automation disabled. Exiting."
    exit 0
fi

log "Automate flag is TRUE — starting Manriix bring-up sequence."

# =============================================================================
# 2. HELPER: Wait for a CAN interface to appear in ip link
#    Returns 0 on success, 1 on timeout
# =============================================================================
wait_for_can_interface() {
    local iface="$1"
    local elapsed=0

    log "Polling for interface '$iface' (timeout: ${CAN_DETECT_TIMEOUT}s)..."

    while [[ $elapsed -lt $CAN_DETECT_TIMEOUT ]]; do
        if ip link show "$iface" &>/dev/null; then
            log "Interface '$iface' detected."
            return 0
        fi
        sleep "$CAN_POLL_INTERVAL"
        elapsed=$(( elapsed + CAN_POLL_INTERVAL ))
    done

    error "Timeout: '$iface' did not appear after ${CAN_DETECT_TIMEOUT}s."
    return 1
}

# =============================================================================
# 3. HELPER: Bring up a CAN interface at 1 Mbit/s
# =============================================================================
bring_up_can() {
    local iface="$1"

    log "Bringing up $iface at 1 Mbit/s..."
    if sudo ip link set "$iface" type can bitrate 1000000 && \
       sudo ip link set "$iface" up; then
        log "$iface is UP."
        return 0
    else
        error "Failed to bring up $iface."
        return 1
    fi
}

# =============================================================================
# 4. CAN2 BRING-UP
# =============================================================================
zenity --info \
    --title="Manriix — CAN2 Setup" \
    --text="Please connect the <b>CAN2</b> USB adapter now.\n\nClick OK when the cable is physically connected." \
    --width=350 2>/dev/null

log "Waiting for CAN2 USB adapter..."

if ! wait_for_can_interface "can2"; then
    zenity --error \
        --title="Manriix — CAN2 Error" \
        --text="CAN2 interface did not appear within ${CAN_DETECT_TIMEOUT} seconds.\n\nCheck the USB adapter and try again." \
        --width=350 2>/dev/null
    error "Aborting startup."
    exit 1
fi

if ! bring_up_can "can2"; then
    zenity --error \
        --title="Manriix — CAN2 Error" \
        --text="Failed to configure CAN2.\n\nCheck sudo permissions and adapter status." \
        --width=350 2>/dev/null
    exit 1
fi

zenity --info \
    --title="Manriix — CAN2 Ready" \
    --text="✅  <b>CAN2 is up successfully!</b>\n\nNow connect the CAN3 cable." \
    --width=350 2>/dev/null

# =============================================================================
# 5. CAN3 BRING-UP
# =============================================================================
zenity --info \
    --title="Manriix — CAN3 Setup" \
    --text="Please connect the <b>CAN3</b> USB adapter now.\n\nClick OK when the cable is physically connected." \
    --width=350 2>/dev/null

log "Waiting for CAN3 USB adapter..."

if ! wait_for_can_interface "can3"; then
    zenity --error \
        --title="Manriix — CAN3 Error" \
        --text="CAN3 interface did not appear within ${CAN_DETECT_TIMEOUT} seconds.\n\nCheck the USB adapter and try again." \
        --width=350 2>/dev/null
    error "Aborting startup."
    exit 1
fi

if ! bring_up_can "can3"; then
    zenity --error \
        --title="Manriix — CAN3 Error" \
        --text="Failed to configure CAN3.\n\nCheck sudo permissions and adapter status." \
        --width=350 2>/dev/null
    exit 1
fi

zenity --info \
    --title="Manriix — CAN3 Ready" \
    --text="✅  <b>CAN3 is up successfully!</b>\n\nLaunching navigation stack now..." \
    --width=350 2>/dev/null

# =============================================================================
# 6. LAUNCH ROS2 NAVIGATION STACK
# =============================================================================
# log "Sourcing ROS2 and workspace..."

# if [[ ! -f "$ROS2_SETUP" ]]; then
#     error "ROS2 setup not found at $ROS2_SETUP"
#     exit 1
# fi

# source "$ROS2_SETUP"

# if [[ -f "$MANRIIX_WS" ]]; then
#     source "$MANRIIX_WS"
# else
#     warn "Manriix workspace setup not found at $MANRIIX_WS — skipping."
# fi

# log "Launching robot_nav.launch.py (hardware_mode=real, camera_mode=full)..."

# ros2 launch manriix_bringup robot_nav.launch.py \
#     hardware_mode:=real \
#     camera_mode:=full

# =============================================================================
# END
# =============================================================================