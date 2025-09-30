#!/bin/bash

# Fall Detection + GPS Emergency Alert System Build Script
# For Nordic Thingy:91 X

echo "=== Fall Detection + GPS Emergency Alert System ==="
echo "Building firmware for Thingy:91 X..."
echo

# Clean previous build
echo "Cleaning previous build..."
rm -rf build
echo "✓ Build directory cleaned"

# Build for Thingy:91 X
echo "Building firmware..."
west build -b thingy91x/nrf9151/ns

if [ $? -eq 0 ]; then
    echo
    echo "✅ Build successful!"
    echo
    echo "Generated files:"
    ls -la build/zephyr/*.hex build/zephyr/*.bin 2>/dev/null || echo "Hex files not found"
    echo
    echo "To flash the firmware:"
    echo "nrfutil device program --serial-number <DEVICE_SERIAL> --firmware build/zephyr/app_signed.hex --options target=nRF91"
    echo
    echo "Or use merged.hex for complete flashing:"
    echo "nrfutil device program --serial-number <DEVICE_SERIAL> --firmware build/zephyr/merged.hex"
    echo
else
    echo
    echo "❌ Build failed!"
    echo "Check the build log above for errors."
    exit 1
fi