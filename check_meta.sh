#!/bin/bash
# Print the list of connected Android devices
adb devices | grep -v "List of devices attached" | awk '{print $1}' | while read -r device; do
    echo "Device: $device"
done

# Check ip route for the device
adb shell ip route

# restart wifi tcpip port
adb tcpip 5555
