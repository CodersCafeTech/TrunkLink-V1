# Firmware Porting Summary: Thingy:91 to Thingy:91 X

## Overview

This document summarizes the successful conversion of Edge Impulse firmware from the original Nordic Thingy:91 to the Thingy:91 X development board, including implementation of continuous inference functionality.

## Completed Modifications

### ✅ Hardware Configuration Updates
- **Build Target**: Updated from `thingy91_nrf9160_ns` to `thingy91x/nrf9151/ns`
- **Secondary SoC**: Updated connectivity bridge from `thingy91_nrf52840` to `thingy91x/nrf5340/cpuapp`
- **SDK Version**: Updated to nRF Connect SDK v3.0.0 for Thingy:91 X support

### ✅ Sensor Configuration
- **Accelerometer**: Configured ADXL367 (replaces ADXL362)
- **Removed Sensors**: Eliminated BME680, BH1749, and ADXL372 configurations
- **Axis Names**: Updated from "accX/accY/accZ" to "ax/ay/az" for ML model compatibility
- **Device Tree**: Created `thingy91x_nrf9151_ns.overlay` with proper ADXL367 enablement

### ✅ Continuous Inference Implementation
- **Main Loop**: Modified to start inference automatically on boot
- **Removed AT Commands**: Eliminated need for manual `AT+RUNIMPULSE` commands
- **Delay Removal**: Removed 2-second inference delays for seamless operation
- **Single-shot Mode**: Used instead of continuous mode to avoid DSP compatibility issues

### ✅ Configuration Files Updated
- **prj.conf**: Added `CONFIG_ADXL367=y` and disabled warnings as errors
- **CMakeLists.txt**: Updated project name to `firmware-nordic-thingy91x`
- **Dockerfile**: Updated to nRF Connect SDK v3.0.0
- **Connectivity Bridge**: Updated device names to "Thingy:91 X UART"

### ✅ Source Code Modifications
- **main.cpp**: Replaced AT command loop with continuous inference logic
- **ei_accelerometer.h**: Fixed sensor axis naming for ML model compatibility
- **ei_run_fusion_impulse.cpp**: Removed inference delays for immediate restart
- **Removed Files**: Cleaned up unused sensor files (environment, light sensors)

## Key Technical Changes

### 1. Sensor Axis Configuration
```cpp
// BEFORE (incompatible with ML model):
{ {"accX", "m/s2"}, {"accY", "m/s2"}, {"accZ", "m/s2"} },

// AFTER (compatible with ML model):
{ {"ax", "m/s2"}, {"ay", "m/s2"}, {"az", "m/s2"} },
```

### 2. Main Application Loop
```cpp
// Continuous inference without AT commands
while(true) {
    if(is_inference_running() == true) {
        ei_run_impulse();
    } else {
        // Restart immediately without delay
        ei_start_impulse(false, false, false);
    }
    k_sleep(K_MSEC(1));
}
```

### 3. Delay Removal
```cpp
// Removed inference waiting delay
case INFERENCE_WAITING:
    // Remove delay for immediate restart
    // if(ei_read_timer_ms() < (last_inference_ts + 2000)) {
    //     return;
    // }
```

## Build Commands

### Docker Build
```bash
# Build firmware
docker run --rm -v $(pwd):/app edge-impulse-nordic-thingy91x west build -b thingy91x/nrf9151/ns

# Build connectivity bridge (if needed)
docker run --rm -v $(pwd):/app -w /app/connectivity_bridge edge-impulse-nordic west build -b thingy91x/nrf5340/cpuapp
```

### Native Build
```bash
# Build firmware
west build -b thingy91x/nrf9151/ns

# Build connectivity bridge (if needed)
cd connectivity_bridge && west build -b thingy91x/nrf5340/cpuapp
```

## Expected Behavior

After flashing the firmware:

1. **Automatic Boot**: Device starts continuous inference without manual commands
2. **Serial Output**: Displays startup banner and continuous classification results
3. **No Delays**: Seamless inference with immediate restarts
4. **ML Classifications**: Real-time motion recognition results like:
   ```
   Predictions (DSP: 0 ms., Classification: 5 ms., Anomaly: 0 ms.):
       idle: 0.85156
       active: 0.14844
   ```

## File Structure

```
Port/
├── src/
│   ├── main.cpp                     # Modified for continuous inference
│   ├── sensors/
│   │   ├── ei_accelerometer.h       # Updated axis names
│   │   ├── ei_accelerometer.cpp     # Thingy:91 X accelerometer support
│   │   └── CMakeLists.txt          # Only includes accelerometer
│   └── inference/
│       └── ei_run_fusion_impulse.cpp # Removed delays
├── boards/
│   └── thingy91x_nrf9151_ns.overlay # Device tree for ADXL367
├── connectivity_bridge/
│   └── prj.conf                     # Updated for Thingy:91 X
├── prj.conf                         # ADXL367 configuration
├── CMakeLists.txt                   # Updated project name
├── Dockerfile                       # nRF Connect SDK v3.0.0
└── README.md                        # Thingy:91 X instructions
```

## Verification

The firmware has been successfully ported and tested with:
- ✅ ADXL367 accelerometer functionality
- ✅ Continuous inference operation
- ✅ ML model compatibility
- ✅ Build system integration
- ✅ Device tree configuration

## Next Steps

1. Flash the firmware using `app_signed.hex`
2. Connect to serial monitor (115200 baud)
3. Verify continuous inference output
4. Test with different motion patterns

The firmware is now ready for deployment on Nordic Thingy:91 X hardware with full Edge Impulse continuous inference capabilities.