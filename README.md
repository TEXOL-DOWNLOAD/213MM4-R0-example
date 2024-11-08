# 213MM4-R0-example

## Features
**Continuous Mode**: Capture data continuously for a specified duration and bandwidth.
**Trigger Mode**: Capture data based on a trigger value.

## Usage
### 1.Importing the Class
You can copy the class **SerialCommunication** and **InstructionSet** in your code.

#### SerialCommunication Class
The **SerialCommunication** class is used to establish a connection to the serial port, send commands, and read data from the connected hardware device.
ˋˋˋbash
class SerialCommunication:  
ˋˋˋ

#### InstructionSet Class
The **InstructionSet** class interacts with the **SerialCommunication** class to control the data capture system. It includes methods to start data capture, read operation statuses, and retrieve raw data.
ˋˋˋbash
class InstructionSet:
ˋˋˋ

To use the InstructionSet, you need to create an instance of it, passing an initialized SerialCommunication instance to the constructor.
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




## Linux serial port Cmd
```bash
sudo dmesg | grep tty
sudo chmod 777 /dev/ttyUSB0
python  main.py
```