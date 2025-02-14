# 213MM4-R0-example

## Features

* **Continuous Mode**: Capture data continuously for a specified duration and bandwidth.

* **Trigger Mode**: Capture data based on a trigger value.

## Prerequisites

- Python 3.x
- `pyserial` library for serial communication
- A device connected to a serial port (e.g., `COM1` on Windows or `/dev/ttyUSB0` on Linux)

You can install `pyserial` via pip if you don't have it installed yet:
```bash
pip install pyserial
```

## Installation

Clone this repo into your `code` directory:

```
mkdir code
git clone https://github.com/TEXOL-DOWNLOAD/213MM4-R0-example.git code
```


## Usage

### 1.Importing the Class

To use the `InstructionSet`, you need to create an instance of it, passing an initialized `SerialCommunication` instance to the constructor.

If you're working on a Windows system, the serial port would likely be `COM1` (or another COM port such as `COM2`, `COM3`, etc., depending on the configuration of your system).

```bash
# Create SerialCommunication instance (replace with actual parameters)
serial_comm = SerialCommunication(port='/dev/ttyUSB0', baudrate=115200)
serial_comm.connect()

# Initialize InstructionSet with the SerialCommunication instance
instruction_set = InstructionSet(serial_comm)
```

### 2. Start Data Capture (Continuous Mode)

You can start the data capture in continuous mode by calling the start_data_capture() method and providing the necessary parameters.

```bash
mode = "continuous"  # or "trigger"
bandwidth = 2  # 1: 1K, 2: 1.25K, etc.
length = 10  # Duration in seconds
unit = 0x00  # 0x00 for acceleration, 0x01 for velocity

# Start data capture
data_size, captured_data = instruction_set.start_data_capture(mode, bandwidth, length, unit)

if data_size:
    print("Data Capture Completed:")
    print(f"Data Size: {data_size}")
    print(f"Captured Data: {captured_data}")
else:
    print("Data capture failed.")
```

### 3. Start Data Capture (Trigger Mode)

Trigger mode is similar to continuous mode, but it will capture data based on a trigger value.

```bash
mode = "trigger"
bandwidth = 2  # 1: 1K, 2: 1.25K, etc.
length = 10  # Duration in seconds
unit = 0x00  # 0x00 for acceleration, 0x01 for velocity
trigger_value = 500  # Trigger value (scaled by 0.001)

# Start trigger-based data capture
data_size, captured_data = instruction_set.start_data_capture(mode, bandwidth, length, unit, trigger=trigger_value)

if data_size:
    print("Data Capture Completed:")
    print(f"Data Size: {data_size}")
    print(f"Captured Data: {captured_data}")
else:
    print("Data capture failed.")
```

### 4. Interrupt Data Capture

To interrupt the current data capture and stop the operation:

```bash
success = instruction_set.cmd_interrupt_mode()

if success:
    print("Data capture interrupted successfully.")
else:
    print("Failed to interrupt data capture.")
```

### 5. Read Temperature

To read the current temperature from the device:

```bash
temperature = instruction_set.cmd_read_temperature()
logging.info(f"Temperature: {temperature}")
```

### 6. Read Sensor ID

To read the sensor ID from the device:

```bash
sensor_id = instruction_set.cmd_read_sensor_id()
logging.info(f"Sensor ID: {sensor_id}")
```

### 5. Read Firmware Version

To read the firmware version of the device:

```bash
version = instruction_set.cmd_read_firmware_version()
logging.info(f"Firmware Version: {version}")
```




## Linux serial port Cmd
```bash
sudo dmesg | grep tty
sudo chmod 777 /dev/ttyUSB0
```

## Serial Connection Setup

The **parity setting** is based on the firmware version:

- **Firmware Version 1006**: Use `serial.PARITY_NONE`.
- **Firmware Version 1005**: Use `serial.PARITY_EVEN`.

**Important**: To change the parity setting based on your firmware version, manually modify the `parity` in the `connect` method, depending on the firmware version of your device.

### Example:

```python
# Modify parity manually based on your firmware version
parity = serial.PARITY_NONE  # For fw version 1006
# parity = serial.PARITY_EVEN  # For fw version 1005

