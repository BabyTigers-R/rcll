#!/bin/bash
sudo nmcli connection del Hotspot
# sudo nmcli device wifi Hotspot ifname wlp1s0 password "ryukokuacjp" ssid ryukoku-NucBoxG5 autoconnect yes
sudo nmcli connection add type wifi ifname wlp1s0 con-name Hotspot autoconnect yes ssid ryukoku-NucBoxG5
sudo nmcli connection modify Hotspot 802-11-wireless.mode ap 802-11-wireless.band a ipv4.method shared
sudo nmcli connection modify Hotspot wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify Hotspot wifi-sec.psk "ryukokuacjp"
sudo nmcli connection modify Hotspot 802-11-wireless-security.wps-method disabled
sudo nmcli connection up Hotspot

