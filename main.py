import serial
import logging
import time
import struct
import csv
from typing import List
import numpy as np

# Configure logging at the start of your application
logging.basicConfig(level=logging.INFO,  # DEBUG, INFO, WARNING, ERROR ,CRITICAL
                    format='%(asctime)s - %(levelname)s - %(message)s')

class SerialCommunication:
    # Initialize the serial communication with port and baudrate
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate         
        self.ser = None
        
    # Establish a connection to the serial port
    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port, 
                self.baudrate,
                bytesize=8, # 8 data bits
                parity=serial.PARITY_NONE, # For fw version 1006
                # parity = serial.PARITY_EVEN  # For fw version 1005                 
                stopbits=1, # 1 stop bit
                timeout=5 # Set timeout for read operations to 5 seconds
            )
            logging.info(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            logging.error(f"Error connecting to serial port: {e}")
    
    def close(self):
        if self.ser is not None:
            self.ser.close()
            logging.info("Serial port closed.")        

    # Write data to the serial port
    def write(self, data: bytes):
        if self.ser is not None:
            try:
                b = self._combined_send_command(data)
                self.ser.write(b)
                logging.debug(f"write: {b.hex().upper()}")
            except Exception as e:
                logging.error(f"Error writing to serial port: {e}")
        else:
            logging.warning("Serial port is not open.")    

    # Read data from the serial port until a complete message is received
    def read_until(self,lengthBytes: int = 1) -> tuple:
        if self.ser is not None: 
            try:
                start_byte = b'\x5B' # Hexadecimal for '[' (start byte)
                stop_byte = b'\x5D'  # Hexadecimal for ']' (stop byte)
                
                response = bytearray() # Initialize an empty response                               
                command_length = 0  
                
                while True:                    
                    # Look for the start byte in the response
                    if not response: # If response is empty, start looking for the start byte                        
                        if not self._find_start_byte(response,start_byte):   
                            logging.error("Read start byte timed out.")
                            return b''  # Return empty bytes on timeout                         
                           
                        logging.debug(f"Response message start_byte: {response.hex().upper()}")

                    else:
                        # Update command length based on the read response
                        command_length = self._read_command_length(response,lengthBytes)
                        
                        if command_length == 0:
                            byte = self.ser.read(1)
                            if byte:
                                response.extend(byte)
                                continue
                            else:
                                logging.error("Read command length timed out.")
                                return b''  # Return empty bytes on timeout 
                            
                        logging.debug(f"Response message command_length: {response.hex().upper()}")                              

                        # Continue reading until the complete message is received
                        byte = self.ser.read(command_length-len(response))

                        if byte:
                            response.extend(byte)
                            if len(response) != command_length: # Continue reading if message is incomplete
                                continue
                        else:
                            logging.error("Read command timed out.")
                            return b''  # Return empty bytes on timeout 
                        
                        logging.debug(f"Response message: {response.hex().upper()}")

                        # Check if the response ends with the stop byte and verify CRC
                        if  response[-1] == stop_byte[0]:
                            crc_expected = self._crc16(response[:-3]) # Calculate expected CRC (excluding last 3 bytes)
                            crc_got = response[-3:-1] # Extract received CRC

                            if crc_expected == crc_got: # If the CRC matches, return the valid response                               
                                return bytes(response[1+lengthBytes:-3]) # Return the response data excluding start byte and CRC
                            else:
                                logging.error(f"checksum: expected {crc_expected.hex().upper()},got {crc_got.hex().upper()}")
                                return b''  # Return empty bytes if checksum mismatch
                        
                        # If stop byte is not found, continue processing
                        response = response[1:] # Remove the first byte and continue searching for start byte

                        if start_byte in response: # If start byte is found in the response                          
                            start_index = response.index(start_byte)
                            response = response[start_index:]  # Keep from the start byte onward  
                        else:
                            response = bytearray()
            
            except Exception as e:
                logging.error(f"Error reading from serial port: {e}")
                return b''
        
        else:
            logging.warning("Serial port is not open.")
            return b''     

    # Helper function to find the start byte in the response
    def _find_start_byte(self, response: bytearray,start_byte: bytes) -> bool:        
        while True: 
            byte = self.ser.read(1)

            if byte:                
                if byte == start_byte: # If the byte matches the start byte, return True
                    response.extend(byte) 
                    return True
            else:                
                return False # Return False if no byte is read
    
    # Helper function to determine the command length based on the lengthBytes argument
    def _read_command_length(self, response: bytearray, lengthBytes: int ) -> int: 
        if len(response) >= lengthBytes+1: # Ensure the response has enough data            
            if lengthBytes == 1: 
                # Command length (1 byte)
                return response[1]                
            elif lengthBytes == 2:
                # Command length (2 bytes)
                return int.from_bytes(response[1:3], byteorder='big')
            
        return 0
    
    def _crc16(self,data: bytes, poly=0xA001):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= poly
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')
    
    # Prepare and combine the command with header, payload, checksum, and stop bytes
    def _combined_send_command(self,payload: bytes)-> bytearray:
        cmd = bytearray([0x53,0x53,0x00])
        stop_bytes = bytes([0x53, 0x54])

        cmd.extend(payload)

        cmd[2] = len(cmd) + 2 + 2 # Update the command length (include checksum and stop bytes)

        cmd.extend(self._crc16(cmd))
        cmd.extend(stop_bytes)

        return cmd # Return the complete command to send
    

class InstructionSet:
    def __init__(self, SerialCommunication):
        """
        Initializes the InstructionSet class with a SerialCommunication object.
        
        Args:
            SerialCommunication (SerialCommunication): An instance of the SerialCommunication class 
                                                        used for sending and receiving data over serial.
        """
        self.serial_comm = SerialCommunication

    def start_data_capture(self, mode: str, bandWidth: int, length: int, unit: int, trigger: int = None) -> tuple:
        """
        Starts the data capture in either continuous or trigger mode.

        Args:
            mode (str): Capture mode, either "continuous" or "trigger".
            bandWidth (U8): Bandwidth for the capture (1: 1K, 2: 1.25K, 3: 1.667K, 4: 2.5K, 5: 5K, 6: 10K)
            length (U8): Duration of the capture in seconds.
            unit (int): Unit for the measurement (0x00 for acceleration, 0x01 for velocity).
            trigger (int, optional): Trigger value for trigger mode (only applicable in "trigger" mode). (scale: 0.001)

        Returns:
            tuple: A tuple containing the data size array and captured data (if successful), 
                   or None values in case of failure.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None, None

        try:
            # Start capture based on the mode: continuous or trigger
            if mode == "continuous":
                command_success = self.cmd_continuous_mode_start(bandWidth, length, unit)
            elif mode == "trigger":
                command_success = self.cmd_trigger_mode_start(bandWidth, length, unit, trigger)
            else:
                logging.error("Error: Invalid mode specified.")
                return None, None

            # If command fails to start, return None
            if not command_success:
                return None, None

            # Wait for data capture to complete (polling)
            while True:
                # Read current operation status
                status = self.cmd_read_operation_status()
                logging.debug(f"Current status: {status:#04x}")

                # If capture is complete (status 0x02), read the captured data
                if status == 0x02:
                    logging.info("Data capture completed.")
                    status, part, data_size_array = self.cmd_read_operation_details()
                    if status:
                        allData = self.cmd_read_rawdata(part, 3)
                        if allData:
                            return data_size_array, np.array(allData).T.tolist()

                    return None, None # Return None if data retrieval failed
                
                time.sleep(0.5)

        except Exception as e:
            logging.error(f"Data capture Error: {e}")
            return None, None  # Return None values on exception
    
    def cmd_continuous_mode_start(self, bandWidth: int, length: int, unit: int) -> bool:
        """
        Starts the data capture in continuous mode.
        
        Args:
            bandWidth (U8): Bandwidth for the capture (1: 1K, 2: 1.25K, 3: 1.667K, 4: 2.5K, 5: 5K, 6: 10K)
            length (U8): Duration of the capture in seconds.
            unit (int): Unit for the measurement (0x00 for acceleration, 0x01 for velocity).
        
        Returns:
            bool: True if the command was successful, False otherwise.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return False
        
        try:
            logging.info("Run cmd_continuous_mode_start")

            # Command identifier for starting continuous mode
            cmd = bytes([0xBA, 0x45])            
            
            # Construct the payload for the continuous mode start command            
            payload = bytearray(cmd ) + bytes([bandWidth, length, unit])
            
            logging.debug(f"Send Payload: {payload.hex().upper()}")
            self.serial_comm.write(payload) # Send the command over serial
            
            retries = 2  # Set a retry limit for response handling
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    # Analyze the response command
                    status_string = ["OK", "Wrong band width or length.", "Fail"]
                    bool,current_status,_ = self._disassemble_esponse_command(cmd,response,status_string)

                    # Ensure the response matches the expected command                    
                    if not bool:
                        # The instructions are different, re-fetch the reply                        
                        retries -= 1
                        continue
                    
                    logging.debug(f"Current status: {current_status}")

                    # Check if the command was successful (status 0x00)
                    return current_status == 0x00
                else:
                    return False

        except Exception as e:
            logging.error(f"Continuous mode start Error: {e}")
            return False
        
    def cmd_trigger_mode_start(self,bandWidth: int,length: int,unit: int,trigger: int)->bool:
        """
        Starts the data capture in trigger mode, where the capture occurs based on a trigger value.
        
        Args:
            bandWidth (U8): Bandwidth for the capture (1: 1K, 2: 1.25K, 3: 1.667K, 4: 2.5K, 5: 5K, 6: 10K)
            length (U8): Duration of the capture in seconds.
            unit (int): Unit for the measurement (0x00 for acceleration, 0x01 for velocity).
            trigger (int): The trigger value for capture start (scaled by 0.001).
        
        Returns:
            bool: True if the command was successful, False otherwise.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return False
        
        try:
            logging.info("Run cmd_trigger_mode_start")

            # Command identifier for starting trigger mode
            cmd = bytes([0xA9, 0x56])            
            
            # Construct the payload for the trigger mode start command            
            payload = bytearray(cmd) + bytes([bandWidth, length, unit]) + trigger.to_bytes(2, byteorder='big')            
            
            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(bytes(payload)) # Send the command over serial         
            
            retries = 2  # Set a retry limit for response handling            
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    # Analyze the response command
                    status_string = ["OK", "Wrong band width or length.", "Fail"]
                    bool,current_status,_ = self._disassemble_esponse_command(cmd,response,status_string)

                    # Extract the command from the response and compare                    
                    if not bool:
                        # The instructions are different, re-fetch the reply                        
                        retries -= 1
                        continue
                                    
                    logging.debug(f"Current status: {current_status}")

                    # Check if the command was successful (status 0x00)
                    return current_status == 0x00
                else:
                    return False # If no response, return failure

        except Exception as e:
            logging.error(f"Trigger mode start Error: {e}")
            return False
        
    def cmd_interrupt_mode(self)->bool:
        """
        Interrupts the current data capture mode and stops the operation.
        
        Returns:
            bool: True if the command was successful, False otherwise.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return False
        
        try:
            logging.info("Interrupt capture")
            cmd = bytes([0x98, 0x67])

            logging.debug(f"Send message: {cmd.hex().upper()}")
            self.serial_comm.write(cmd)

            retries = 2  # Set a retry limit for response handling            
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    status_string = ["OK", "Fail"]
                    bool,current_status,_ = self._disassemble_esponse_command(cmd,response,status_string)

                    # Extract the command from the response and compare                    
                    if not bool:
                        # The instructions are different, re-fetch the reply                        
                        retries -= 1
                        continue
                                    
                    logging.debug(f"Current status: {current_status}")

                    return current_status == 0x00
                else:
                    return False            
                
        except Exception as e:
            logging.error(f"Interrupt Mode Error: {e}")
            return False
        
    def cmd_read_operation_status(self)->int:
        """
        Reads the operation status of the device (e.g., idle, in progress, completed).
        
        Returns:
            int: The current status code (e.g., 0x02 for completed).
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None
                
        try:
            logging.info("Run cmd_read_operation_status")
            cmd = bytes([0x87, 0x78])

            logging.debug(f"Send message: {cmd.hex().upper()}")
            self.serial_comm.write(cmd) # Send the command over serial

            retries = 2  # Set a retry limit for response handling            
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    status_string = ["Idle", "In Progress", "Completed","Wait for Trigger", "Fail"]
                    bool,current_status,_ = self._disassemble_esponse_command(cmd,response,status_string)

                    # Extract the command from the response and compare                    
                    if not bool:
                        # The instructions are different, re-fetch the reply                        
                        retries -= 1
                        continue
                                    
                    logging.debug(f"Current status: {current_status}")

                    return current_status
                else:
                    return None              
                
        except Exception as e:
            logging.error(f"Read operation status Error: {e}")
            return None
        
    def cmd_read_operation_details(self) -> tuple:
        """
        Reads the details of the current operation (e.g., bandwidth, length, data size array).
        
        Returns:
            tuple: A tuple containing status (bool), part (data part number), and data_size_array (size of captured data).
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return False, None, None
                
        try:
            logging.info("Run cmd_read_operation_details")
            cmd = bytes([0x78, 0x89])

            logging.debug(f"Send message: {cmd.hex().upper()}")
            self.serial_comm.write(cmd) # Send the command over serial
            
            retries = 2  # Set a retry limit for response handling            
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    status_string = ["OK", "Fail"]
                    bool,current_status,payload = self._disassemble_esponse_command(cmd,response,status_string)

                    # Extract the command from the response and compare                    
                    if not bool:
                        # The instructions are different, re-fetch the reply                        
                        retries -= 1
                        continue
                                    
                    logging.debug(f"Current status: {current_status}")
                    
                    if current_status == 0x00:
                        # Extract operation details
                        bandWidth = payload[0]
                        length = payload[1]
                        unit = payload[4]
                        
                        bandWidth_string = ["NAN", "1K", "1.25K", "1.667K", "2.5K", "5K", "10K"]
                        unit_string = ["Acceleration", "Velocity"]

                        logging.info(f"BandWidth: {self._status_display_string(bandWidth, bandWidth_string)} "
                            f"Length: {length}sec "
                            f"Unit: {self._status_display_string(unit, unit_string)}")

                        # Extract part as a u16 (16-bit unsigned integer)
                        part = int.from_bytes(payload[2:4], byteorder='big')
                        logging.debug(f"Part (u16): {part}")

                        # Extract sizeArray and convert to u16 (16-bit unsigned integers)
                        sizeArray = payload[5:]  # Ensure this slice is correct based on the response format
                        data_size = []
                        
                        # Iterate through the sizeArray in chunks of 2 bytes (for u16)
                        for i in range(0, len(sizeArray), 2):
                            if i + 2 <= len(sizeArray):
                                u16_value = int.from_bytes(sizeArray[i:i + 2], byteorder='big')
                                data_size.append(u16_value)
                        
                        logging.info(f"Data Size Array (u16): {data_size}")

                        # Return current status (0x00 for success)
                        return True, part, data_size
                    else:
                        return False, None, None

                    # return current_status
                else:
                    return False, None, None             
            
        except Exception as e:
            logging.error(f"Read operation status Error: {e}")
            return False, None, None
    
    def cmd_read_rawdata(self,part: int,retry: int)->tuple:
        """
        Reads raw data from the device, retries if data read fails.
    
        Returns:
            tuple: A tuple containing data (list of float32 values),
                or None values on failure.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None
        
        try:
            logging.info("Start reading raw data")

            data_bytes_all = []
            for j in range(3): # Loop over three axes (X, Y, Z)
                data_bytes = bytearray()
                
                for i in range(part):# Loop over the data parts                    
                    
                    for r in range(retry):# Retry the data read if it fails
                        data = self.cmd_read_rawdata_fragment(i+1,j+1)
                        
                        if data:
                            data_bytes.extend(data)
                            break
                        else:
                            logging.debug(f"Retry: {r+1}")
                            if r + 1 == retry:                            
                                return None
            
                data_bytes_all.append(data_bytes)

            allData = []
            for _, data_bytes in enumerate(data_bytes_all): 
                float_values = []
                
                # Convert byte data to float values (4 bytes per float32)
                for i in range(0, len(data_bytes), 4):  
                    if i + 4 <= len(data_bytes):
                        float_value = struct.unpack('>f', data_bytes[i:i + 4])[0]  # Big-endian
                        float_values.append(float_value)
                
                allData.append(float_values)
            
            return allData # Return the data as a list of float32 values
        
        except Exception as e:
            print(f"Read Raw Data Error: {e}")
            return None
    
    def cmd_read_rawdata_fragment(self,part_index: int,axial: int) -> tuple:
        """
        Reads a fragment of raw data for a specific part and axis.
        
        Returns:
            tuple: Data fragment or None if read fails.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None
        
        try:
            # Ensure part_index is within valid range for u16 (0 to 65535)
            if not (0 <= part_index <= 0xFFFF):
                raise ValueError("part must be a 16-bit unsigned integer (0-65535)")
            
            logging.debug("Run cmd_read_rawdata_fragment")
            cmd = bytes([0x67, 0x9A])

            # Construct the payload for the command
            payload = bytearray(cmd) + part_index.to_bytes(2, byteorder='big') + bytes([axial])
            
            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(bytes(payload)) # Send the constructed command
            
            retries = 2  # Set a retry limit for response handling            
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until(2)
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    # status_string = ["OK", "Fail"]
                    bool,current_status,payload = self._disassemble_esponse_command(cmd,response)

                    # Extract the command from the response and compare                    
                    if not bool:
                        # The instructions are different, re-fetch the reply                        
                        retries -= 1
                        continue
                                    
                    logging.debug(f"Current status: {current_status}")

                    if current_status == 0x00:
                        data = payload  # Ensure this slice is correct based on the response format
                        return data
                    
                    # Return status indicating failure and None for data
                    return None
                else:
                   return None      
        
        except Exception as e:
            logging.error(f"Read Raw Data Fragment Error: {e}")
            return None
        
    def cmd_read_temperature(self)->tuple:
        """
        Reads the temperature of the device.
        
        Returns:
            float: The temperature in degrees Celsius.
            If there's an error or mismatch, returns None.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None
                
        try:
            logging.info("Run cmd_read_temperature")
            cmd = bytes([0x34, 0xCD])

            logging.debug(f"Send message: {cmd.hex().upper()}")
            self.serial_comm.write(cmd) # Send the command over serial

            retries = 2  # Set a retry limit for response handling
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    cmd_got = response[:2] # Get the first two bytes of the response as the command
                    if cmd_got != cmd:
                        # If the command doesn't match, log the error and return failure
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        return None
                    
                    temperature = int.from_bytes(response[2:2 + 4], byteorder='big')*0.001 

                    return temperature                   
                    
                else:
                    return None              
                
        except Exception as e:
            logging.error(f"Read Temperature Error: {e}")
            return None   

    def cmd_read_sensor_id(self)->tuple:
        """
        Reads the sensor ID of the device.
    
        Returns:
            string: The sensor ID as a string.
            If there's an error or mismatch, returns None.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None
                
        try:
            logging.info("Run cmd_read_sensor_id")
            cmd = bytes([0x61, 0x9A])

            logging.debug(f"Send message: {cmd.hex().upper()}")
            self.serial_comm.write(cmd) # Send the command over serial

            retries = 2  # Set a retry limit for response handling
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    cmd_got = response[:2] # Get the first two bytes of the response as the command
                    if cmd_got != cmd:
                        # If the command doesn't match, log the error and return failure
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        return None
                    
                    sensor_id = response[2:2 + 6].decode('ascii')
                    return sensor_id
                    
                else:
                    return None              
                
        except Exception as e:
            logging.error(f"Read SensorID Error: {e}")
            return None

    def cmd_read_firmware_version(self)->tuple:
        """
        Reads the firmware version of the device.
    
        Returns:
            string: The firmware version as a string in hexadecimal format.
            If there's an error or mismatch, returns None.
        """
        # Check if serial communication is initialized
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None
                
        try:
            logging.info("Run cmd_read_firmware_version")
            cmd = bytes([0x45, 0xBC])

            logging.debug(f"Send message: {cmd.hex().upper()}")
            self.serial_comm.write(cmd) # Send the command over serial

            retries = 2  # Set a retry limit for response handling
            while retries > 0:
                # Wait for response from MCU
                response = self.serial_comm.read_until()
                if response:
                    logging.debug(f"Response Payload: {response.hex().upper()}")

                    cmd_got = response[:2] # Get the first two bytes of the response as the command
                    if cmd_got != cmd:
                        # If the command doesn't match, log the error and return failure
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        return None
                    
                    version = f"{response[2:2+4].hex().upper()}"
                    return version
                    
                else:
                    return None              
                
        except Exception as e:
            logging.error(f"Read Firmware Version Error: {e}")
            return None       
    
    # Returns a string representation of a status based on the index and display list.
    def _status_display_string(self,index: int, display: List[str]) -> str:        
        if index >= len(display):
            return f"Value: {index}"
        return display[index]
    
    def _disassemble_esponse_command(self,cmd: bytes,response: bytes,status_string: list[str]=[]) -> tuple: 
        """
        Disassembles the response from the device, verifying the command and status.
        
        Args:
            cmd: The expected command bytes.
            response: The response received from the device.
            status_string: A list of possible status messages to interpret the response.
        
        Returns:
            tuple: A tuple containing:
                - bool: Whether the response matches the expected command.
                - status: The status code from the response.
                - payload: The data portion of the response.
        """        
        cmd_got = response[:2] # Get the first two bytes of the response as the command
        if cmd_got != cmd:
            # If the command doesn't match, log the error and return failure
            logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
            return False, 0, b''
        
        status = response[2] # Extract the status byte from the response        
        if status_string != []:
            logging.info(f"cmd status: {self._status_display_string(status, status_string)}")
        
        payload = response[3:] # Extract the payload (data) from the response        

        return True, status, payload



# Example usage
if __name__ == "__main__":
    # If you're working on a Windows system, the serial port would likely be COM1 (or another COM port such as COM2, COM3, etc., depending on the configuration of your system).
    serial_comm = SerialCommunication(port='/dev/ttyUSB0', baudrate=115200)
    serial_comm.connect()     
    
    instruction_set = InstructionSet(serial_comm)
    
    try:  

        mode = "continuous" # or "trigger"
        bandWidth = 6
        length = 2
        unit = 0x00
        # trigger = 0.5
        
        data_size, captured_data=instruction_set.start_data_capture(mode,bandWidth,length,unit)
        # data_size, captured_data=instruction_set.start_data_capture(mode,bandWidth,length,unit,int(trigger*1000))
        
        if data_size:
            logging.info("Data capture successful")
            with open('output.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(captured_data)
        else: 
            logging.info("Data capture unsuccessful")    
        
        '''# Read Temperature
        temperature = instruction_set.cmd_read_temperature()
        logging.info(f"Temperature: {temperature}")
        
        # Read sensor_id
        sensor_id = instruction_set.cmd_read_sensor_id()
        logging.info(sensor_id) 

        # Read firmware version
        version = instruction_set.cmd_read_firmware_version()
        logging.info(version)'''
        

    except KeyboardInterrupt:
        print("Stopped by user.")
    
    finally:
        serial_comm.close()
