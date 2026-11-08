#!/bin/bash
# Save the Solo controller's Wi-Fi on the Pi, preferred over the phone hotspot when both are around.
#   ./add_sololink.sh "SoloLink_XXXX" [password]
SSID="$1"; PASS="${2:-sololink}"
[ -z "$SSID" ] && { echo "usage: $0 SSID [password]"; exit 1; }
run() { "$@" 2>/dev/null || sudo "$@"; }
run nmcli connection delete "$SSID" >/dev/null 2>&1
run nmcli connection add type wifi con-name "$SSID" ifname wlan0 ssid "$SSID" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASS" connection.autoconnect yes connection.autoconnect-priority 20
# keep the hotspot as the fallback
HOT=$(nmcli -t -f NAME,TYPE connection show | grep -i ':802-11-wireless' | grep -v "^$SSID:" | head -1 | cut -d: -f1)
[ -n "$HOT" ] && run nmcli connection modify "$HOT" connection.autoconnect-priority 5
nmcli -t -f NAME,AUTOCONNECT-PRIORITY connection show | grep -v "^lo"
