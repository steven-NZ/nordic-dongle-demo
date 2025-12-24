# Nordic nRF52840 Dongle - System Design

**Note**: This is a demonstration project showcasing wireless sensor-to-USB HID bridging.

## Overview

A wireless receiver dongle that receives sensor data (nordic airpad) via Enhanced ShockBurst (ESB) protocol and presents as a USB HID composite device (mouse + keyboard) to the host computer.

## Protocol

**Enhanced ShockBurst (ESB)**: Nordic's proprietary wireless protocol running at 1 Mbps with +8 dBm TX power, receiving 14-byte packets containing button states, touch electrode data, and IMU quaternions.

**USB HID**: Composite device with two report IDs - Report ID 1 for mouse (buttons, absolute X/Y, scroll wheel) and Report ID 2 for keyboard (modifiers, 6 keycodes).

## Functionality

**IMU Mouse Control**: Quaternion data from a 3D IMU sensor is converted to Euler angles, smoothed, and mapped to absolute screen coordinates (0-32767 range) with ±60° sensitivity.

**Touch Keyboard**: Four capacitive touch electrodes (North/South/East/West) map to keys A/B/C/D respectively using edge-triggered detection.

**Mouse Buttons**: Three physical buttons map directly to left/right/middle mouse clicks.

**Scroll Wheel**: Capacitive airwheel velocity (0-63) scales linearly to scroll wheel values (±127) with directional control.

**LED Feedback**: GPIO LED toggles on each received ESB packet for visual confirmation.

**Vibration Feedback**: Haptic response sent to transmitter via ESB ACK packets with intensity control (0-255). Each touch electrode triggers different intensity levels: North=255, East=220, South=200, West=180.

## Data Flow

```
ESB RX Interrupt → Parse sensor_data_t → Process (quaternion→Euler→position,
touch→keys, buttons, scroll) → Queue reports → Main loop dequeues →
USB HID write (Report ID 1: Mouse, Report ID 2: Keyboard)
                 ↓
         Update vibration intensity → Build response_data_t → ESB ACK TX
```

## Packet Structure

```c
struct sensor_data_t {
    uint8_t  btn_state;    // 3 button bits
    uint16_t mgc_state;    // Touch electrodes + airwheel
    int16_t  quat_w;       // Quaternion components
    int16_t  quat_x;
    int16_t  quat_y;
    int16_t  quat_z;
} // 14 bytes total
```

**ESB Response Payload** (sent via ACK):
```c
struct response_data_t {
    uint8_t vibration_intensity;  // 0=off, 1-255=motor intensity
    int8_t  reserved[7];          // Future expansion
} // 8 bytes total
```

## Key Configuration

- **ESB**: ESB_PROTOCOL_ESB_DPL, 1 Mbps, PRX mode, +8 dBm
- **IMU**: MAX_ANGLE=60°, SMOOTHING_FACTOR=0.5, CENTER_DEADZONE=±5°
- **Screen**: SCREEN_MAX=32767 (16-bit absolute positioning)
- **Vibration Intensity**: Touch N=255, E=220, S=200, W=180 (0=off)
- **Demo Addresses**: base_addr_0={0xE7,0xE7,0xE7,0xE7}, base_addr_1={0xC2,0xC2,0xC2,0xC2}

## Development Notes

This implementation is a demonstration focused on the AirPad sensor integration. For production development, refer to the Linux dongle architecture and design a complete system specification before extending functionality. 
