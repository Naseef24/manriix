#!/bin/bash
# =============================================================================
# Manriix Full Startup Script
# =============================================================================
# Fully automated — no user input required.
# Brings up CAN2 and CAN3, verifies them, then launches RC teleop.
# All popups are informational and auto-close.
# Skips bring-up if interface is already UP (e.g. after manual testing).
#
# Usage: bash manriix_startup_full.sh
# =============================================================================

ROS2_SETUP="/opt/ros/humble/setup.bash"
MANRIIX_WS="$HOME/manriix2_ws/install/setup.bash"
CAN_DETECT_TIMEOUT=30
CAN_POLL_INTERVAL=1

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()   { echo -e "${GREEN}[Manriix]${NC} $1"; }
warn()  { echo -e "${YELLOW}[Manriix]${NC} $1"; }
error() { echo -e "${RED}[Manriix]${NC} $1"; }

# =============================================================================
# HELPER: Wait for a CAN interface to appear
# =============================================================================
wait_for_can_interface() {
    local iface="$1"
    local elapsed=0

    if ip link show "$iface" &>/dev/null; then
        log "Interface '$iface' already present."
        return 0
    fi

    warn "Waiting for '$iface'..."
    while [[ $elapsed -lt $CAN_DETECT_TIMEOUT ]]; do
        sleep "$CAN_POLL_INTERVAL"
        elapsed=$(( elapsed + CAN_POLL_INTERVAL ))
        if ip link show "$iface" &>/dev/null; then
            log "Interface '$iface' detected after ${elapsed}s."
            return 0
        fi
    done

    error "Timeout: '$iface' did not appear after ${CAN_DETECT_TIMEOUT}s."
    return 1
}

# =============================================================================
# HELPER: Bring up a CAN interface — skips if already UP
# =============================================================================
bring_up_can() {
    local iface="$1"
    local state
    state=$(ip link show "$iface" 2>/dev/null | grep -oP "state \K\w+")

    if [[ "$state" == "UP" ]]; then
        log "$iface is already UP — skipping configuration."
        return 0
    fi

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
# 1. NOTIFY: CAN bring-up starting (auto-closes after 4 seconds)
# =============================================================================
zenity --info \
    --title="Manriix — Starting Up" \
    --text="🔄  Bringing up CAN interfaces...\n\nThis will complete automatically." \
    --timeout=4 \
    --width=350 2>/dev/null &

# =============================================================================
# 2. BRING UP CAN2 AND CAN3
# =============================================================================
log "Starting CAN bring-up..."

if ! wait_for_can_interface "can2"; then
    zenity --error --title="Manriix — Error" \
        --text="CAN2 not detected. Check the USB adapter." \
        --width=350 2>/dev/null
    exit 1
fi
bring_up_can "can2" || { zenity --error --title="Manriix — Error" --text="Failed to bring up CAN2." --width=350 2>/dev/null; exit 1; }

if ! wait_for_can_interface "can3"; then
    zenity --error --title="Manriix — Error" \
        --text="CAN3 not detected. Check the USB adapter." \
        --width=350 2>/dev/null
    exit 1
fi
bring_up_can "can3" || { zenity --error --title="Manriix — Error" --text="Failed to bring up CAN3." --width=350 2>/dev/null; exit 1; }

# =============================================================================
# 3. VERIFY BOTH INTERFACES
# =============================================================================
sleep 2   # Allow interfaces to fully transition to UP state
CAN2_STATUS=$(ip link show can2 2>/dev/null | grep -oP "state \K\w+")
CAN3_STATUS=$(ip link show can3 2>/dev/null | grep -oP "state \K\w+")

if [[ "$CAN2_STATUS" != "UP" || "$CAN3_STATUS" != "UP" ]]; then
    zenity --error \
        --title="Manriix — CAN Verification Failed" \
        --text="CAN verification failed.\n\nCAN2: ${CAN2_STATUS:-unknown}\nCAN3: ${CAN3_STATUS:-unknown}" \
        --width=350 2>/dev/null
    exit 1
fi

log "Both CAN interfaces verified — CAN2: $CAN2_STATUS | CAN3: $CAN3_STATUS"

# =============================================================================
# 4. NOTIFY: CAN is up (auto-closes after 5 seconds)
# =============================================================================
zenity --info \
    --title="Manriix — CAN Ready" \
    --text="✅  <b>CAN interfaces are up successfully!</b>\n\nCAN2: ${CAN2_STATUS}\nCAN3: ${CAN3_STATUS}" \
    --timeout=5 \
    --width=350 2>/dev/null

# =============================================================================
# 5. LAUNCH RC TELEOP
# =============================================================================
log "Sourcing ROS2 and workspace..."

[[ -f "$ROS2_SETUP" ]] && source "$ROS2_SETUP" || { error "ROS2 setup not found."; exit 1; }
[[ -f "$MANRIIX_WS" ]] && source "$MANRIIX_WS" || warn "Manriix workspace setup not found — skipping."

log "Waiting 5 seconds before launching RC teleop..."
sleep 5

log "Launching manriix_rc_teleop..."
ros2 run manriix_rc_teleop manriix_rc_teleop