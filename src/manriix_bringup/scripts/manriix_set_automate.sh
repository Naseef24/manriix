#!/bin/bash
# =============================================================================
# Manriix Automate Flag Manager
# =============================================================================
# Usage:
#   bash manriix_set_automate.sh true    → Enable startup automation
#   bash manriix_set_automate.sh false   → Disable startup automation
#   bash manriix_set_automate.sh status  → Check current setting
# =============================================================================

FLAG_FILE="$HOME/.manriix_automate"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

case "$1" in
    true)
        echo "true" > "$FLAG_FILE"
        echo -e "${GREEN}[Manriix]${NC} Automation ENABLED → $FLAG_FILE"
        ;;
    false)
        echo "false" > "$FLAG_FILE"
        echo -e "${YELLOW}[Manriix]${NC} Automation DISABLED → $FLAG_FILE"
        ;;
    status)
        if [[ -f "$FLAG_FILE" ]]; then
            VALUE=$(cat "$FLAG_FILE" | tr -d '[:space:]')
            echo -e "[Manriix] Current automate flag: ${GREEN}${VALUE}${NC}"
        else
            echo -e "[Manriix] Flag file not found — automation is ${YELLOW}DISABLED${NC}"
        fi
        ;;
    *)
        echo "Usage: $0 {true|false|status}"
        exit 1
        ;;
esac
