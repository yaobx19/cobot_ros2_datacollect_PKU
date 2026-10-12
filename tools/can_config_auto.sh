#!/usr/bin/env bash
set -euo pipefail

BITRATE=${CAN_BITRATE:-1000000}
LEFT_SERIAL=${CAN_LEFT_SERIAL:-0027003B5547571120343930}
RIGHT_SERIAL=${CAN_RIGHT_SERIAL:-003F002C4148571320343133}
LEFT_BUS=${CAN_LEFT_BUS:-}
RIGHT_BUS=${CAN_RIGHT_BUS:-}
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage: sudo bash can_config_auto.sh [--dry-run]

Automatically maps the two known USB-CAN adapters to can_left/can_right.
Environment overrides:
  CAN_LEFT_SERIAL, CAN_RIGHT_SERIAL  Stable USB adapter serial numbers
  CAN_LEFT_BUS, CAN_RIGHT_BUS        Exact bus-info overrides (for example 3-4:1.0)
  CAN_BITRATE                        CAN bitrate (default: 1000000)
EOF
}

case "${1:-}" in
  "") ;;
  --dry-run) DRY_RUN=1 ;;
  -h|--help) usage; exit 0 ;;
  *) echo "[can-auto] unknown option: $1" >&2; usage >&2; exit 2 ;;
esac

[[ "$BITRATE" =~ ^[1-9][0-9]*$ ]] || {
  echo "[can-auto] CAN_BITRATE must be a positive integer" >&2
  exit 2
}
if (( DRY_RUN == 0 && EUID != 0 )); then
  echo "[can-auto] run as root: sudo bash $0" >&2
  exit 2
fi

declare -a IFACES=()
declare -A IFACE_TO_BUS=()
declare -A IFACE_TO_SERIAL=()
declare -A BUS_TO_IFACE=()
declare -A SERIAL_TO_IFACE=()

for net_path in /sys/class/net/*; do
  [[ -r "$net_path/type" ]] || continue
  [[ "$(<"$net_path/type")" == 280 ]] || continue
  iface=${net_path##*/}
  device_path=$(readlink -f "$net_path/device")
  bus=${device_path##*/}
  serial=""
  [[ -r "$device_path/../serial" ]] && serial=$(<"$device_path/../serial")
  IFACES+=("$iface")
  IFACE_TO_BUS["$iface"]="$bus"
  IFACE_TO_SERIAL["$iface"]="$serial"
  BUS_TO_IFACE["$bus"]="$iface"
  [[ -z "$serial" ]] || SERIAL_TO_IFACE["$serial"]="$iface"
done

if [[ "${#IFACES[@]}" -ne 2 ]]; then
  echo "[can-auto] expected 2 CAN interfaces, found ${#IFACES[@]}: ${IFACES[*]:-none}" >&2
  exit 2
fi

find_by_port_suffix() {
  local suffix=$1
  local match=""
  local iface
  for iface in "${IFACES[@]}"; do
    if [[ "${IFACE_TO_BUS[$iface]}" == *"-${suffix}:1.0" ]]; then
      [[ -z "$match" ]] || return 1
      match=$iface
    fi
  done
  [[ -n "$match" ]] || return 1
  printf '%s\n' "$match"
}

select_interface() {
  local role=$1
  local bus_override=$2
  local serial=$3
  local port_suffix=$4
  local iface=""

  if [[ -n "$bus_override" ]]; then
    iface=${BUS_TO_IFACE[$bus_override]:-}
    [[ -n "$iface" ]] || {
      echo "[can-auto] $role bus $bus_override was not found" >&2
      return 1
    }
  elif [[ -n "$serial" && -n "${SERIAL_TO_IFACE[$serial]:-}" ]]; then
    iface=${SERIAL_TO_IFACE[$serial]}
  elif iface=$(find_by_port_suffix "$port_suffix"); then
    echo "[can-auto] warning: $role serial $serial was not found; using physical USB port *-${port_suffix}:1.0" >&2
  else
    echo "[can-auto] cannot identify the $role adapter" >&2
    echo "[can-auto] set CAN_${role^^}_SERIAL or CAN_${role^^}_BUS explicitly" >&2
    return 1
  fi
  printf '%s\n' "$iface"
}

left_iface=$(select_interface left "$LEFT_BUS" "$LEFT_SERIAL" 4)
right_iface=$(select_interface right "$RIGHT_BUS" "$RIGHT_SERIAL" 6)
[[ "$left_iface" != "$right_iface" ]] || {
  echo "[can-auto] left and right resolved to the same interface: $left_iface" >&2
  exit 2
}

echo "[can-auto] detected adapters:"
for iface in "${IFACES[@]}"; do
  echo "[can-auto]   $iface bus=${IFACE_TO_BUS[$iface]} serial=${IFACE_TO_SERIAL[$iface]:-unknown}"
done
echo "[can-auto] left:  ${IFACE_TO_BUS[$left_iface]} serial=${IFACE_TO_SERIAL[$left_iface]:-unknown} -> can_left ($left_iface)"
echo "[can-auto] right: ${IFACE_TO_BUS[$right_iface]} serial=${IFACE_TO_SERIAL[$right_iface]:-unknown} -> can_right ($right_iface)"

if (( DRY_RUN == 1 )); then
  echo "[can-auto] dry-run passed; no interfaces were changed"
  exit 0
fi

modprobe gs_usb

for temporary_name in can_tmp_l can_tmp_r; do
  [[ ! -e "/sys/class/net/$temporary_name" ]] || {
    echo "[can-auto] stale temporary interface exists: $temporary_name" >&2
    exit 2
  }
done

# Temporary names make reruns and a previously swapped left/right mapping safe.
ip link set "$left_iface" down 2>/dev/null || true
ip link set "$right_iface" down 2>/dev/null || true
ip link set "$left_iface" name can_tmp_l
ip link set "$right_iface" name can_tmp_r

ip link set can_tmp_l type can bitrate "$BITRATE"
ip link set can_tmp_l name can_left
ip link set can_left up

ip link set can_tmp_r type can bitrate "$BITRATE"
ip link set can_tmp_r name can_right
ip link set can_right up

for iface in can_left can_right; do
  details=$(ip -details link show "$iface")
  [[ "$details" == *"bitrate $BITRATE"* ]] || {
    echo "[can-auto] $iface bitrate verification failed" >&2
    exit 2
  }
  [[ "$details" == *"UP"* ]] || {
    echo "[can-auto] $iface is not UP" >&2
    exit 2
  }
done

echo "[can-auto] configured can_left/can_right at $BITRATE bit/s"
ip -details -br link show can_left
ip -details -br link show can_right
