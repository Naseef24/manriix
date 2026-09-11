# Manriix CAN Startup Automation

## Overview

This document covers the development of the CAN interface bring-up automation system for the **Manriix** robot platform. The system automates the configuration of two USB-to-CAN adapters (`can2`, `can3`) and optionally launches the full ROS2 navigation stack on startup.

---

## Background

Previously, bringing up Manriix required running the following commands manually each time the robot was powered on:

```bash
# CAN2
sudo ip link set can2 type can bitrate 1000000 && sudo ip link set can2 up

# CAN3
sudo ip link set can3 type can bitrate 1000000 && sudo ip link set can3 up

# Launch navigation stack
ros2 launch manriix_bringup robot_nav.launch.py hardware_mode:=real camera_mode:=full
```

The goal was to automate this process while keeping the flexibility for developers to bypass automation during individual development sessions.

---

## Hardware

| Component | Details |
|---|---|
| Compute | Jetson AGX Orin |
| CAN Adapters | bytewerk candleLight USB-to-CAN adapters (×2) |
| Interface names | `can2`, `can3` (can0/can1 are Jetson built-in Tegra CAN) |
| CAN bitrate | 1 Mbit/s |
| Termination | 120Ω both ends |

---

## Why USB-to-CAN Detection Works

The Jetson has two built-in Tegra CAN controllers (`can0`, `can1`) that are **always present** in `ip link` regardless of whether a cable is connected — making physical detection impossible on those interfaces without bus traffic.

`can2` and `can3` however are **USB-to-CAN adapters**. The Linux kernel only creates the network interface when the USB device is physically enumerated. This means:

- **No adapter plugged in** → interface does not exist in `ip link`
- **Adapter plugged in** → interface appears in `ip link` within ~1 second

This makes detection as simple as polling `ip link show can2` — no bus traffic or ACKs required.

---

## Interface Name Assignment & udev Rules

### The Problem

Without udev rules, interface names (`can2`, `can3`) are assigned purely by **plug-in order**:

- First USB-to-CAN adapter plugged in → `can2`
- Second USB-to-CAN adapter plugged in → `can3`

This means plugging adapters in the wrong order swaps the interface names, potentially connecting the wrong CAN bus to the wrong interface.

### The Fix — udev Rules

Each adapter has a **unique USB serial number**, which allows udev to permanently assign the correct interface name regardless of plug-in order.

Serial numbers identified via:
```bash
udevadm info /sys/class/net/can2 | grep -E "ID_SERIAL|ID_USB_SERIAL|ID_PATH"
udevadm info /sys/class/net/can3 | grep -E "ID_SERIAL|ID_USB_SERIAL|ID_PATH"
```

| Interface | Adapter Serial |
|---|---|
| `can2` | `0024003B5734570320353132` |
| `can3` | `004000464642570E20333033` |

**Rule file:** `/etc/udev/rules.d/80-manriix-can.rules`

```
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="0024003B5734570320353132", NAME="can2"
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="004000464642570E20333033", NAME="can3"
```

**Install:**
```bash
sudo cp 80-manriix-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug both adapters. Plug-in order no longer matters.

---

## Automate Flag

To allow developers to bypass automation during individual development sessions, the system uses a simple flag file:

| Flag value | Behaviour |
|---|---|
| `true` | Full startup automation runs on boot via the systemd service |
| `false` or missing | Service exits silently — no automation, nothing is touched |

**Flag file location:** `~/.manriix_automate`

**Manage the flag:**
```bash
# Enable automation (will run on next boot)
bash manriix_set_automate.sh true

# Disable automation (safe for dev sessions)
bash manriix_set_automate.sh false

# Check current state
bash manriix_set_automate.sh status
```

Since all developers share a single Linux user account on the Jetson, this flag applies to everyone on the machine. The standard practice before a dev session is to run `manriix_set_automate.sh false` so the service does nothing on the next reboot.

> **Important:** The automate flag is only checked by the systemd service via the wrapper script. Running `manriix_startup_full.sh` directly always executes regardless of the flag — this is intentional.

---

## Scripts

All scripts live in:
```
~/manriix2_ws/src/manriix_bringup/scripts/
```

### `manriix_startup.sh`
**Guided bring-up with user interaction via zenity dialogs.**

- Checks the `~/.manriix_automate` flag — exits silently if not `true`
- Detects whether CAN adapters are already plugged in at startup
- If not plugged in — shows a zenity popup asking the user to connect each adapter
- Polls `ip link show` until the interface appears (60s timeout)
- Brings up each interface at 1 Mbit/s
- Shows a success popup for each interface
- Launches `robot_nav.launch.py hardware_mode:=real camera_mode:=full`

**Use case:** Backup/fallback script. Kept in case `manriix_startup_full.sh` has issues.

```bash
bash ~/manriix2_ws/src/manriix_bringup/scripts/manriix_startup.sh
```

---

### `manriix_startup_full.sh`
**Fully automatic — no user input required.**

- Does NOT check the automate flag — always runs when called directly
- Shows an auto-closing zenity popup: *"Bringing up CAN interfaces..."* (4 seconds)
- Checks if each interface is already `UP` — skips `ip link set` if so (avoids "Device or resource busy" error)
- Detects both interfaces, brings them up, waits 2 seconds for state to settle
- Verifies both are `UP` via `ip link show`
- Shows an auto-closing popup with verification status (5 seconds)
- Shows an auto-closing popup: *"Robot navigation stack is starting!"* (5 seconds)
- Launches `robot_nav.launch.py hardware_mode:=real camera_mode:=full`

**Use case:** Primary script — called by the systemd service and for manual full bring-up.

```bash
bash ~/manriix2_ws/src/manriix_bringup/scripts/manriix_startup_full.sh
```

> **Note:** A `sleep 2` before verification is intentional — `can3` needs time to fully transition to `UP` state after `ip link set ... up` is called.

---

### `manriix_service_wrapper.sh`
**Flag check layer between the systemd service and the full startup script.**

- Called exclusively by the systemd service at boot
- Reads `~/.manriix_automate` — exits silently if not `true`
- If `true` — calls `manriix_startup_full.sh`
- Keeps the flag logic completely separate from the script itself

**Use case:** Internal — not intended for manual use.

---

### `manriix_set_automate.sh`
**Toggle the automate flag.**

```bash
bash manriix_set_automate.sh true    # Enable
bash manriix_set_automate.sh false   # Disable
bash manriix_set_automate.sh status  # Check
```

---

## Startup Flow

```
Reboot
 └── systemd: manriix_startup.service (after graphical.target + 10s delay)
      └── manriix_service_wrapper.sh
           ├── automate = false → exits silently, nothing happens
           └── automate = true
                └── manriix_startup_full.sh
                     ├── Check if can2/can3 already UP
                     ├── Bring up CAN interfaces
                     ├── Verify both UP
                     └── Launch robot_nav.launch.py
```

**Manual use (bypasses flag entirely):**
```
bash manriix_startup_full.sh
 ├── Bring up CAN interfaces
 ├── Verify both UP
 └── Launch robot_nav.launch.py
```

---

## Repository Structure

```
manriix_bringup/
├── scripts/
│   ├── manriix_startup.sh            # Guided bring-up (backup/fallback)
│   ├── manriix_startup_full.sh       # Fully automatic bring-up (primary)
│   ├── manriix_service_wrapper.sh    # Automate flag check for service
│   └── manriix_set_automate.sh       # Automate flag manager
├── systemd/
│   └── manriix_startup.service       # Systemd service file
└── config/
    └── 80-manriix-can.rules          # udev rules reference copy
```

---

## First-Time Setup

Run these once on the Jetson before using any of the scripts:

```bash
# 1. Install zenity
sudo apt install zenity -y

# 2. Set up passwordless sudo for ip link commands
echo "$(whoami) ALL=(ALL) NOPASSWD: /sbin/ip link set can2 *, /sbin/ip link set can3 *" \
    | sudo tee /etc/sudoers.d/manriix-can

# 3. Install udev rules
sudo cp ~/manriix2_ws/src/manriix_bringup/config/80-manriix-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. Make scripts executable
chmod +x ~/manriix2_ws/src/manriix_bringup/scripts/manriix_startup.sh
chmod +x ~/manriix2_ws/src/manriix_bringup/scripts/manriix_startup_full.sh
chmod +x ~/manriix2_ws/src/manriix_bringup/scripts/manriix_startup_wrapper.sh
chmod +x ~/manriix2_ws/src/manriix_bringup/scripts/manriix_set_automate.sh

# 5. Unplug and replug both CAN adapters to apply udev rules

# 6. Install the systemd service
sudo cp ~/manriix2_ws/src/manriix_bringup/systemd/manriix_startup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable manriix_startup.service
```

---

## Systemd Service

The service is installed at `/etc/systemd/system/manriix_startup.service` and runs after `graphical.target` with a 10 second pre-delay to ensure the desktop and display are fully ready before zenity attempts to show popups.

**Useful commands:**

```bash
# Check service status
sudo systemctl status manriix_startup.service

# View live logs
journalctl -u manriix_startup.service -f

# Manually start the service
sudo systemctl start manriix_startup.service

# Restart the service
sudo systemctl restart manriix_startup.service
```

---

## Stopping / Disabling Bring-Up

### Why `systemctl stop` Alone Was Not Enough

When the service is stopped, systemd previously only killed the parent bash process — leaving `ros2 launch` and all its child nodes still running. The fix was adding `KillMode=control-group` and `KillSignal=SIGINT` to the service file. This ensures systemd kills every process in the service's cgroup (including ROS2 and all its nodes) when stopped, using SIGINT which is the correct graceful shutdown signal for ROS2 launch files.

**Stop the service and all running ROS2 processes:**
```bash
sudo systemctl stop manriix_startup.service
```

**If ROS2 is still running after stopping (legacy — should no longer be needed):**
```bash
pkill -f "ros2 launch"
```

**Bring down CAN interfaces:**
```bash
sudo ip link set can2 down
sudo ip link set can3 down
```

**Prevent automation on the next reboot (service still installed):**
```bash
bash ~/manriix2_ws/src/manriix_bringup/scripts/manriix_set_automate.sh false
```

**Permanently disable the service from booting:**
```bash
sudo systemctl disable manriix_startup.service
```

**Re-enable the service:**
```bash
sudo systemctl enable manriix_startup.service
```

---

## Why `systemctl stop` Did Not Kill ROS2

`ros2 launch` is a child process of the startup script. When systemd stops the service it only kills the parent bash process, leaving the launch still running.

**Quick fix — kill it manually:**
```bash
pkill -f "ros2 launch"
```

**Permanent fix** — the service file now includes `KillMode=control-group` and `KillSignal=SIGINT`:

- `KillMode=control-group` — kills every process in the service's cgroup, including `ros2 launch` and all its children
- `KillSignal=SIGINT` — sends SIGINT (Ctrl+C) instead of SIGTERM, which is the correct way to gracefully shut down a ROS2 launch file

After copying the updated service file, `sudo systemctl stop manriix_startup.service` will cleanly kill everything including ROS2:
```bash
sudo cp ~/manriix2_ws/src/manriix_bringup/systemd/manriix_startup.service /etc/systemd/system/
sudo systemctl daemon-reload
```

---

## CAN Shutdown on Reboot/Poweroff

To automatically bring down CAN interfaces on every shutdown or reboot — regardless of whether the manriix service is running or the automate flag is set — a dedicated shutdown script is placed in `/lib/systemd/system-shutdown/`.

### Why Not a Separate Service?

A `manriix_can_down.service` using `ExecStop` was tried first but did not work reliably. The reason is timing — by the time systemd runs `ExecStop` during shutdown, the USB subsystem has already started tearing down, so the `ip link set` commands either fail silently or have no effect.

### The Fix — System Shutdown Script

Scripts placed in `/lib/systemd/system-shutdown/` are executed directly by systemd just before the final reboot/poweroff call — at that point all services are already stopped but hardware is still accessible.

**Script:** `manriix_can_down.sh`

```bash
/sbin/ip link set can2 down 2>/dev/null
/sbin/ip link set can3 down 2>/dev/null
```

**Install:**
```bash
# Remove the old service if installed
sudo systemctl disable manriix_can_down.service
sudo rm /etc/systemd/system/manriix_can_down.service

# Install the shutdown script
sudo cp ~/manriix2_ws/src/manriix_bringup/scripts/manriix_can_down.sh /lib/systemd/system-shutdown/
sudo chmod +x /lib/systemd/system-shutdown/manriix_can_down.sh
```

After this, every reboot or shutdown will automatically bring down `can2` and `can3` before the system goes down. If the interfaces are already down, the script exits silently with no errors.

---

## Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| Interface name swapped | Adapters plugged in wrong order | udev rules fix this permanently |
| `can3` shows DOWN during verification | Kernel hasn't finished transitioning state | `sleep 2` before verify — already in script |
| "Device or resource busy" on ip link set | Interface already UP from previous session | Script now checks state before configuring |
| Service exits silently on boot | Automate flag is `false` or missing | Run `manriix_set_automate.sh status` to check |
| `sudo ip link` asks for password | sudoers entry missing | Re-run the sudoers setup step above |
| zenity popup doesn't appear | Display not ready when service started | 10s pre-sleep handles this — check `DISPLAY=:0` |
| Interface doesn't appear after plugging in | USB not enumerated | Unplug and replug; check `dmesg` for USB errors |
| Warning about graphical-session.target | Old service file still installed | Ensure updated service file uses `graphical.target` |

---

## Key Learnings

- **USB-to-CAN adapters enumerate dynamically** — the network interface only exists when the USB device is plugged in, making `ip link show` a reliable detection method
- **Built-in Tegra CAN controllers are different** — `can0`/`can1` exist permanently regardless of cable connection; detection on those requires bus traffic
- **udev serial-based rules** are the correct fix for non-deterministic USB enumeration order
- **State transition delay** — after `ip link set ... up`, the interface needs ~2 seconds to fully reach `UP` state before verification
- **"Device or resource busy"** occurs when trying to reconfigure an interface already in the `UP` state — always check state before running `ip link set`
- **graphical.target vs graphical-session.target** — the service must use `graphical.target` (system-level) not `graphical-session.target` (user-level) to avoid the non-existent unit warning and ensure correct boot ordering
- **Wrapper pattern** — separating the flag check into a wrapper script keeps `manriix_startup_full.sh` clean and always manually runnable, while the service path still respects the automate flag