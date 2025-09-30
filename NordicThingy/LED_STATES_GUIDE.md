# LED Status Indicator Guide
## Fall Detection + GPS Emergency Alert System

This document describes the comprehensive LED color scheme implemented for visual system status feedback.

## LED Colors and Meanings

### 🔴 Red LED States
- **Solid Red** - Critical system error (stays on)
- **Fast Red Blinks** - LoRa initialization failed (5 rapid blinks)
- **Double Red Blinks** - LoRaWAN join failed (2 blinks, pause, 2 blinks)
- **Triple Red Blinks** - Data transmission failed (3 blinks)
- **Red Continuous** - Emergency alert failed to send (3 seconds solid)
- **Red Urgent Flash** - FALL DETECTED! (rapid emergency flashing pattern)

### 🟢 Green LED States
- **Solid Green (2s)** - All systems ready/operational
- **Single Green Blink** - GPS fix acquired
- **Double Green Blinks** - LoRaWAN network joined successfully
- **Triple Green Blinks** - Data sent successfully

### 🔵 Blue LED States
- **Very Fast Blue Blinks** - ML inference running (rapid pulse every 5 seconds)
- **Fast Blue Blinks** - LoRa sending data (8 rapid blinks)
- **Triple Blue Blinks** - Routine location update sent
- **Single Blue Pulse** - ML detected movement activity

### 🟡 Yellow LED States (Red + Green)
- **Slow Yellow Blinks** - LoRa module initializing (5 slow blinks with pauses)

### 🟠 Orange LED States (Red + Green timed)
- **Slow Orange Blinks** - Joining LoRaWAN network (5 slow alternating blinks)
- **Orange Blinks** - GPS fix lost (3 blinks)
- **Orange Dim** - Emergency cooldown active (brief flash)

### 🟣 Purple/Magenta LED States (Red + Blue)
- **Slow Purple Blinks** - GPS searching for fix (1 blink every 30 seconds)
- **Slow Magenta Blinks** - ML system initializing (5 slow blinks)

### 🩵 Cyan LED States (Green + Blue)
- **Slow Cyan Blinks** - GPS initializing (5 slow blinks)

### ⚪ White LED States (All Colors)
- **White Blinks** - System initialization (3 blinks at startup)

## Emergency States (Critical)

### 🚨 Fall Detection Emergency Sequence
1. **Red Urgent Flash** - Fall detected (rapid emergency pattern)
2. **Blue Fast Blinks** - Sending emergency location data
3. **Red + Green Alternating** - Emergency alert sent successfully
4. **Red + Blue Alternating** - Emergency location sent successfully

### ⚠️ Emergency Failures
- **Red Continuous (3s)** - Emergency alert failed to send
- **Red + Orange Alternating** - Location send failed during emergency

## System State Progression

### Normal Startup Sequence
1. ⚪ **White Blinks** - System initializing
2. 🟡 **Yellow Blinks** - LoRa initializing
3. 🟢 **Green Blinks** - LoRa initialized
4. 🟠 **Orange Blinks** - Joining LoRaWAN
5. 🟢 **Double Green** - Network joined
6. 🩵 **Cyan Blinks** - GPS initializing
7. 🟣 **Purple Blinks** - GPS searching
8. 🟢 **Green Solid** - GPS fix acquired
9. 🟣 **Magenta Blinks** - ML initializing
10. 🟢 **Solid Green (2s)** - All systems ready!

### Operational States
- 🔵 **Blue Fast Pulse** - Running inference (every 5 seconds)
- 🔵 **Triple Blue** - Routine location sent (every 5 minutes)
- 🟠 **Orange Blinks** - GPS fix lost (if happens)

## Error States

### LoRa Errors
- 🔴 **Fast Red Blinks** - LoRa initialization failed
- 🔴 **Double Red** - Network join failed
- 🔴 **Triple Red** - Data transmission failed

### GPS Errors
- 🔴 **Red Blink** - GPS initialization failed
- 🔴 **Slow Red Blinks** - GPS fix timeout

### System Errors
- 🔴 **Solid Red** - Critical system error (UART, hardware failure)

## Quick Reference Card

| Color | Pattern | Meaning |
|-------|---------|---------|
| ⚪ White | 3 blinks | System starting |
| 🟡 Yellow | Slow blinks | LoRa init |
| 🩵 Cyan | Slow blinks | GPS init |
| 🟣 Purple | Slow blinks | GPS searching |
| 🟣 Magenta | Slow blinks | ML init |
| 🟢 Green | Solid 2s | All ready |
| 🔵 Blue | Fast pulse | ML running |
| 🔴 Red | Urgent flash | **FALL DETECTED** |
| 🔴🟢 Alt | Alternating | Emergency sent |

## LED Hardware

**Thingy:91 X LED Pins (VERIFIED):**
- Red LED: P0.29
- Green LED: P0.31  ← **CORRECTED**
- Blue LED: P0.30   ← **CORRECTED**

**LED Logic:** Active HIGH (1 = ON, 0 = OFF) ← **CORRECTED**

## Usage Notes

1. **Emergency Priority**: Emergency states override all other LED indications
2. **State Duration**: Most states show briefly (1-3 seconds) then return to operational
3. **Operational State**: Green breathing or blue inference pulses during normal operation
4. **Error Persistence**: Critical errors keep red LED solid until reset
5. **GPS Searching**: Purple blinks continue until fix acquired or timeout

## Troubleshooting by LED Color

### 🔴 Red LED Stuck On
- Critical system error
- Check serial console for error messages
- May require device reset

### 🟡 Yellow LED Stuck Blinking
- LoRa module not responding
- Check UART connections
- Verify LoRa-E5 module power

### 🟣 Purple LED Continuous
- GPS cannot get fix
- Move device outdoors
- Check antenna connections

### No LED Activity
- LED system not initialized
- Hardware failure
- Check power supply

This LED system provides comprehensive visual feedback for all system states, making it easy to diagnose issues and monitor system health without requiring serial console access.