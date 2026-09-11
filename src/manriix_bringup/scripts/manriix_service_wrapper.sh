#!/bin/bash
# =============================================================================
# Manriix Service Wrapper
# =============================================================================
# Called by the systemd service on boot.
# Checks the ~/.manriix_automate flag — only proceeds if set to true.
# Does NOT affect manual use of manriix_startup_full.sh directly.
# =============================================================================

FLAG_FILE="/home/hype/.manriix_automate"

if [[ ! -f "$FLAG_FILE" ]]; then
    echo "[Manriix Service] Automate flag not found — skipping startup."
    exit 0
fi

FLAG_VALUE=$(cat "$FLAG_FILE" | tr -d '[:space:]' | tr '[:upper:]' '[:lower:]')
if [[ "$FLAG_VALUE" != "true" ]]; then
    echo "[Manriix Service] Automate flag is '$FLAG_VALUE' — skipping startup."
    exit 0
fi

echo "[Manriix Service] Automate flag is TRUE — launching full startup..."
exec /bin/bash /home/hype/manriix2_ws/src/manriix_bringup/scripts/manriix_startup_full.sh
