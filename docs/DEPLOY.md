# Onboard deployment

Deploy and run `wbc_g1_ctrl` on the Unitree G1 onboard PC. For project overview, build steps, dependencies, joystick controls, and config layout, see the [README](../README.md).

Network setup below is adapted from the [LeRobot Unitree G1 guide](https://huggingface.co/docs/lerobot/en/unitree_g1#connect-to-the-physical-robot).

## Overview

1. Connect your laptop to the G1 over Ethernet and SSH in.
2. Share your laptop’s WiFi to the robot (needed for `git clone` and package installs).
3. Optionally enable onboard WiFi for wireless SSH.
4. Build and run the controller on the G1 — see [Build and run on the G1](#build-and-run-on-the-g1).

## Connect to the physical robot

The G1’s Ethernet IP is fixed at `192.168.123.164`. Your machine must use a static IP on the same subnet: `192.168.123.x` where `x ≠ 164`.

On your laptop, replace `LAPTOP_ETH_IFC` with your Ethernet interface name (check with `ip a`):

```bash
sudo ip addr flush dev LAPTOP_ETH_IFC
sudo ip addr add 192.168.123.200/24 dev LAPTOP_ETH_IFC
sudo ip link set LAPTOP_ETH_IFC up
```

SSH into the robot:

```bash
ssh unitree@192.168.123.164
# Password: 123
```

## Share internet via Ethernet

The G1 needs internet access to clone repos and install packages. Share your laptop’s WiFi connection over Ethernet.

**On your laptop:**

```bash
sudo sysctl -w net.ipv4.ip_forward=1

# Replace LAPTOP_WIFI_IFC with your WiFi interface name
sudo iptables -t nat -A POSTROUTING -o LAPTOP_WIFI_IFC -s 192.168.123.0/24 -j MASQUERADE
sudo iptables -A FORWARD -i LAPTOP_WIFI_IFC -o LAPTOP_ETH_IFC -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i LAPTOP_ETH_IFC -o LAPTOP_WIFI_IFC -j ACCEPT
```

**On the G1:**

```bash
sudo ip route del default 2>/dev/null || true
sudo ip route add default via 192.168.123.200 dev eth0
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf

# Verify
ping -c 3 8.8.8.8
```

## Onboard WiFi (optional)

For wireless SSH, enable WiFi on the G1 (disabled by default):

```bash
sudo rfkill unblock all
sudo ip link set wlan0 up
sudo nmcli radio wifi on
sudo nmcli device set wlan0 managed yes
sudo systemctl restart NetworkManager
```

Connect to a network:

```bash
nmcli device wifi list

sudo nmcli connection add type wifi ifname wlan0 con-name "YourNetwork" ssid "YourNetwork"
sudo nmcli connection modify "YourNetwork" wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify "YourNetwork" wifi-sec.psk "YourPassword"
sudo nmcli connection modify "YourNetwork" connection.autoconnect yes
sudo nmcli connection up "YourNetwork"

ip a show wlan0
```

Then SSH over WiFi:

```bash
ssh unitree@<ROBOT_WIFI_IP>
# Password: 123
```

## Build and run on the G1

Install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) if it is not already present. CMake checks `/opt/unitree_robotics` and `$UNITREE_SDK_PREFIX` — see [Dependencies](../README.md#dependencies) in the README.

Clone the repo and build (same steps as [Build and run](../README.md#build-and-run)):

```bash
git clone https://github.com/wbc-mjlab/wbc-g1-deploy.git
cd wbc-g1-deploy
scripts/bootstrap_thirdparty.sh
mkdir -p build && cd build && cmake .. && make -j
```

Run with the robot NIC (usually `eth0`, not `lo`):

```bash
./wbc_g1_ctrl --network=eth0
```

Use `tmux` so the controller keeps running if SSH disconnects:

```bash
tmux new -s wbc
./wbc_g1_ctrl --network=eth0
# Detach: Ctrl-b then d
```

Joystick and FSM controls are documented in the [README](../README.md#joystick). For system design, see [Architecture](ARCHITECTURE.md).
