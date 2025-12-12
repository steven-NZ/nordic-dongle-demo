# Build Instructions for nRF52840 Dongle

This guide provides step-by-step instructions for building and programming firmware on the **nRF52840 Dongle**.

> **Note**: This guide is specifically for the nRF52840 **dongle**, not development kits. The dongle uses a DFU (Device Firmware Update) bootloader for programming, which differs from the J-Link debugger used with development kits.

## Prerequisites

Before you begin, download and install the following software:

### 1. nRF Connect for VS Code
- Install the **nRF Connect for VS Code** extension from the VS Code marketplace
- This provides the build environment and toolchain management

### 2. nRF Connect for Desktop
- Download and install **nRF Connect for Desktop** from [Nordic Semiconductor's website](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop)
- After installation, open nRF Connect for Desktop and install these apps:
  - **Programmer** - For flashing firmware to the dongle
  - **Serial Terminal** - For viewing console logs

## Setup Build Configuration

1. Open this project folder in VS Code
2. Open the nRF Connect extension panel (sidebar)
3. Click **"Add Build Configuration"** or select an existing configuration
4. Configure the build settings:
   - **Board**: `nrf52840dongle_nrf52840`
   - Other settings can use defaults
5. The configuration will be created in the `build/` directory

## Building the Firmware

1. In the nRF Connect extension panel, locate the **Actions** section
2. Click the **"Build"** button
3. Wait for the build process to complete (check the terminal output)
4. Upon successful build, the firmware file will be generated at:
   ```
   build/zephyr/zephyr.hex
   ```

## Programming the Dongle

### Step 1: Enter Bootloader Mode
- Locate the **reset button** on the dongle (small button on the side)
- Press the reset button **from the side** to enter DFU bootloader mode
- The dongle's LED should start pulsing, indicating bootloader mode

### Step 2: Open Programmer App
- Launch the **Programmer** app from nRF Connect for Desktop

### Step 3: Select Device
- In the Programmer app, look for the **"Select device"** dropdown
- You should see **"Open DFU Bootloader"** listed as an available device
- Click to select it

### Step 4: Add Firmware Files

You need to add **two** hex files:

1. **Application Firmware**:
   - Click **"Add file"** in the Programmer
   - Navigate to and select: `build/zephyr/zephyr.hex`

2. **Bootloader** (Required):
   - Click **"Add file"** again
   - Navigate to the `bootloader/` directory in this repository
   - Select the bootloader hex file
   - The bootloader is required to enable future programming sessions

> **Important**: The bootloader hex file is included in the `bootloader/` directory of this repository.

### Step 5: Program the Dongle
- Click the **"Write"** or **"Program"** button in the Programmer app
- Wait for the programming process to complete
- The dongle will automatically reset and start running the new firmware

## Viewing Console Logs

To view debug output and console logs from the dongle:

1. Open the **Serial Terminal** app from nRF Connect for Desktop
2. The dongle should appear as a COM port in the device list
3. Select the correct COM port
4. Console output from the application will appear in the terminal

## Troubleshooting

### Dongle not appearing in Programmer
- Ensure the reset button is pressed properly (from the side, not top)
- Try unplugging and replugging the dongle, then press reset again
- Check that the LED is pulsing (indicates bootloader mode)

### Programming fails
- Verify both hex files are added (application + bootloader)
- Ensure you selected "Open DFU Bootloader" as the device
- Try entering bootloader mode again

### No console output in Serial Terminal
- Verify the correct COM port is selected
- Check the baud rate setting (typically 115200)
- Ensure the firmware has logging enabled (check `prj.conf`)
- Try disconnecting and reconnecting to the COM port
