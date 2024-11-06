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
                parity=serial.PARITY_EVEN,  # Use even parity for error checking
                stopbits=1, # 1 stop bit
                timeout=5 # Timeout in seconds for read operations
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
    def read_until(self,lengthBytes: int = False) -> bytes: 
        if self.ser is not None: 
            try:
                start_byte = b'\x5B' # Hexadecimal for '[' (start byte)
                stop_byte = b'\x5D'  # Hexadecimal for ']' (stop byte)
                
                response = bytearray()
                start_found = False
                length_found = False
                command_length = 0

                while True:
                    # Look for the start byte in the response
                    if not start_found:                        
                        if not self._find_start_byte(response,start_byte):   
                            logging.error("Read start byte timed out.")
                            return b''  # Return empty bytes on timeout                         
                            
                        logging.debug(f"Response message start_byte: {response.hex().upper()}")   

                    # Read the command length
                    if not length_found:
                        command_length = self._read_command_length(len_lengthByte,response)
                        if command_length is None:
                            logging.error("Read command length timed out.")
                            return b''  # Return empty bytes on timeout 
                        
                        logging.debug(f"Response message command_length: {response.hex().upper()}")                              

                    # Continue reading until the complete message is received
                    byte = self.ser.read(command_length-len(response))
                    if byte and len(byte) == command_length-len(response):
                       response.extend(byte)
                    else:
                        logging.error("Read command timed out.")
                        return b''  # Return empty bytes on timeout 
                    
                    logging.debug(f"Response message: {response.hex().upper()}")

                    # Check if the response ends with the stop byte and verify CRC
                    if  response[-1] == stop_byte[0]:
                        crc_expected = self._crc16(response[:-3])
                        crc_got = response[-3:-1]

                        if crc_expected == crc_got:
                            return bytes(response) # Return the valid response                            
                        else:
                            logging.error(f"checksum: expected {crc_expected.hex().upper()},got {crc_got.hex().upper()}")
                            return b''  # Return empty bytes if checksum mismatch
                    
                    # If stop byte is not found.
                    # If start byte is found, start reading the rest of the message
                    if start_byte in response:
                        start_found = True
                        start_index = response.index(start_byte)
                        response = response[start_index:]  # Keep from the start byte onward  

                        # Update command length
                        if not lengthBytes: 
                            # Command length (1 byte)                            
                            if len(response) >= 1+1: 
                                length_found = True
                                command_length = response[1]
                            else:
                                length_found = False 
                        else:
                            # Command length (2 bytes)
                            if len(response) >= 2+1: 
                                length_found = True
                                command_length = int.from_bytes(response[1:3], byteorder='big')
                            else:
                                length_found = False 
                    else:
                        start_found = False
                        length_found = False
            
            except Exception as e:
                logging.error(f"Error reading from serial port: {e}")
                return b''
        
        else:
            logging.warning("Serial port is not open.")
            return b''     

    def _find_start_byte(self, response: bytearray,start_byte: bytes) -> bool:        
        while True: 
            byte = self.ser.read(1)
            if byte:                
                if byte == start_byte:
                    response.extend(byte)
                    return True
            else:                
                return False # Return False if no byte is read
    
    def _read_command_length(self, len_lengthByte: int, response: bytearray) -> int:               
        byte = self.ser.read(len_lengthByte)
        if byte:
            response.extend(byte)
            if len_lengthByte == 1:
                return response[1]  # Command length (1 byte)
            else:
                return int.from_bytes(response[1:3], byteorder='big') # Command length (2 bytes)
            
        return None  # Not enough bytes read yet
    
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
    
    # Prepare the command to send, including headers, payload, and checksum
    def _combined_send_command(self,payload: bytes)-> bytearray:
        cmd = bytearray([0x53,0x53,0x00])
        stop_bytes = bytes([0x53, 0x54])

        cmd.extend(payload)
        cmd[2] = len(cmd)+2+2 #Checksum(2).End Bytes(2)
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
        if self.serial_comm is None:
            logging.error("Error: Serial communication not initialized.")
            return None, None

        try:
            if mode == "continuous":
                command_success = self.cmd_continuous_mode_start(bandWidth, length, unit)
            elif mode == "trigger":
                command_success = self.cmd_trigger_mode_start(bandWidth, length, unit, trigger)
            else:
                logging.error("Error: Invalid mode specified.")
                return None, None

            if not command_success:
                return None, None

            while True:
                status = self.cmd_read_operation_status()
                logging.debug(f"Current status: {status:#04x}")

                if status == 0x02:
                    logging.info("Data capture completed.")
                    status, part, data_size_array = self.cmd_read_operation_details()
                    if status:
                        allData = self.cmd_read_rawdata(part, 3)
                        if allData:
                            return data_size_array, np.array(allData).T.tolist()

                    return None, None

                time.sleep(0.5)

        except Exception as e:
            logging.error(f"Data capture Error: {e}")
            return None, None  # Return None values on exception
    
    def cmd_continuous_mode_start(self, bandWidth: int, length: int, unit: int) -> bool:
        """
        Starts the data capture in continuous mode.
        
        Args:
            bandWidth (int): The bandwidth for the capture.
            length (int): Duration of the capture in seconds.
            unit (int): The unit of measurement (acceleration or velocity).
        
        Returns:
            bool: True if the command was successful, False otherwise.
        """
        try:
            logging.debug("Run cmd_continuous_mode_start")

            # Command identifier for starting continuous mode
            cmd = bytes([0xBA, 0x45])            
            
            # Construct the payload for the continuous mode start command            
            payload = bytearray(cmd )+ bytes([bandWidth, length, unit])         
            
            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(payload)

            # Define a length size for response, which could be adjusted based on expected response format
            len_lengthByte = 1
            retries = 5  # Set a retry limit for response handling
            # Wait for response from MCU
            while True:
                response = self.serial_comm.read_until(length_size)
                if response:
                    logging.debug(f"Response message: {response.hex().upper()}")

                    # Extract the command from the response and compare
                    cmd_got = response[1+length_size:1+length_size+2]
                    if cmd_got != cmd:
                        # The instructions are different, re-fetch the reply
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        continue                    
                                    
                    # Get the current status from the response
                    # Check if the operation status is OK
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["OK", "Wrong band width or length.", "Fail"]
                    logging.info(f"Continuous mode start cmd status: {self._status_display_string(current_status, status_string)}")

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
            bandWidth (int): The bandwidth for the capture.
            length (int): Duration of the capture in seconds.
            unit (int): The unit of measurement (acceleration or velocity).
            trigger (int): The trigger value for capture start (scaled by 0.001).
        
        Returns:
            bool: True if the command was successful, False otherwise.
        """
        try:
            logging.debug("Run cmd_trigger_mode_start")
            cmd = bytes([0xA9, 0x56])
            length_size = 1
            
            # Construct the payload for the trigger mode start command            
            payload = bytearray(cmd) + bytes([bandWidth, length, unit])+trigger.to_bytes(2, byteorder='big')            
            
            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(bytes(payload))        
            
            # Wait for response from MCU
            while True:
                response = self.serial_comm.read_until(length_size)
                if response:
                    logging.debug(f"Response message: {response.hex().upper()}")

                    # Extract the command from the response and compare
                    cmd_got = response[1+length_size:1+length_size+2]
                    if cmd_got != cmd:
                        # The instructions are different, re-fetch the reply
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        continue                    
                                    
                    # Get the current status from the response
                    # Check if the operation status is OK
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["OK", "Wrong band width or length.", "Fail"]
                    logging.info(f"Trigger mode start cmd status: {self._status_display_string(current_status, status_string)}")

                    return current_status == 0x00
                else:
                    return False

        except Exception as e:
            logging.error(f"Trigger mode start Error: {e}")
            return False
        
    def cmd_interrupt_mode(self)->bool:
        """
        Interrupts the current data capture mode and stops the operation.
        
        Returns:
            bool: True if the command was successful, False otherwise.
        """
        try:
            logging.debug("Interrupt capture")
            cmd = bytes([0x98, 0x67])
            length_size = 1
            
            # Construct the interrupt mode command            
            payload = bytearray(cmd)            

            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(bytes(payload))

            # Wait for response from MCU
            while True:
                response = self.serial_comm.read_until(length_size)
                if response:
                    logging.debug(f"Response message: {response.hex().upper()}")

                    # Extract the command from the response and compare
                    cmd_got = response[1+length_size:1+length_size+2]
                    if cmd_got != cmd:
                        # The instructions are different, re-fetch the reply
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        continue                    
                                    
                    # Get the current status from the response
                    # Check if the operation status is OK
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["OK", "Fail"]
                    logging.info(f"Interrupt mode cmd status: {self._status_display_string(current_status, status_string)}")

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
        try:
            logging.debug("Run cmd_read_operation_status")
            cmd = bytes([0x87, 0x78])
            length_size = 1
            
            # Construct the read status command            
            payload = bytearray(cmd)

            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(bytes(payload))
            
            # Wait for response from MCU
            while True:
                response = self.serial_comm.read_until(length_size)
                if response:
                    logging.debug(f"Response message: {response.hex().upper()}")

                    # Extract the command from the response and compare
                    cmd_got = response[1+length_size:1+length_size+2]
                    if cmd_got != cmd:
                        # The instructions are different, re-fetch the reply
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        continue                    
                                    
                    # Get the current status from the response
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["Idle", "In Progress", "Completed","Wait for Trigger", "Fail"]
                    logging.info(f"Read operation status cmd status: {self._status_display_string(current_status, status_string)}")

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
            tuple: A tuple containing the status of the operation, part number, and data size array.
        """
        try:
            logging.debug("Run cmd_read_operation_details")
            cmd = bytes([0x78, 0x89])
            length_size = 1
            
            # Construct the read operation details command         
            payload = bytearray(cmd)

            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(bytes(payload))
            
            # Read until a specific stop byte
            response = self.serial_comm.read_until(length_size)
            if response:
                logging.debug(f"Response message: {response.hex().upper()}")
                
                # Get the current status from the response
                current_status = response[4]
                status_string = ["OK", "Fail"]
                logging.info(f"Read operation details cmd status: {self._status_display_string(current_status, status_string)}")

                
                if current_status == 0x00:
                    bandWidth = response[5]
                    length = response[6]
                    unit = response[9]
                    
                    bandWidth_string = ["NAN", "1K", "1.25K", "1.667K", "2.5K", "5K", "10K"]
                    unit_string = ["Acceleration", "Velocity"]

                    logging.info(f"BandWidth: {self._status_display_string(bandWidth, bandWidth_string)} "
                        f"Length: {length}sec "
                        f"Unit: {self._status_display_string(unit, unit_string)}")

                    # Extract part as a u16 (16-bit unsigned integer)
                    part = int.from_bytes(response[7:9], byteorder='big')
                    logging.debug(f"Part (u16): {part}")

                    # Extract sizeArray and convert to u16 (16-bit unsigned integers)
                    sizeArray = response[10:-3]  # Ensure this slice is correct based on the response format
                    data_size = []
                    
                    # Iterate through the sizeArray in chunks of 2 bytes (for u16)
                    for i in range(0, len(sizeArray), 2):
                        if i + 2 <= len(sizeArray):
                            u16_value = int.from_bytes(sizeArray[i:i + 2], byteorder='big')
                            data_size.append(u16_value)
                    
                    logging.info(f"Data Size Array (u16): {data_size}")

                    # Return current status (0x00 for success)
                    return current_status == 0x00, part, data_size

        except Exception as e:
            logging.error(f"Read operation status Error: {e}")
            return False, None, None
    
    def cmd_read_rawdata(self,part: int,retry: int)->tuple:
        """
        Returns:
            tuple: A tuple containing a data,
                or None values on failure.
        """
        try:
            logging.info("Start reading raw data")

            data_bytes_all = []
            for j in range(3): #three axes
                data_bytes = bytearray()
                for i in range(part):
                    for r in range(retry):
                        data = self.cmd_read_rawdata_fragment(i+1,j+1)
                        if data:
                            data_bytes.extend(data)
                            break
                        else:
                            logging.debug(f"Retry: {r+1}")
                            if r+1 == retry:                            
                                return None
            
                data_bytes_all.append(data_bytes)

            allData = []
            for index, data_bytes in enumerate(data_bytes_all):                
                
                float_values = []
                
                for i in range(0, len(data_bytes), 4):  # 4 bytes per float32
                    if i + 4 <= len(data_bytes):
                        float_value = struct.unpack('>f', data_bytes[i:i + 4])[0]  # Big-endian
                        float_values.append(float_value)
                
                allData.append(float_values)
            
            return allData
        
        except Exception as e:
            print(f"Read Raw Data Error: {e}")
            return None
    
    def cmd_read_rawdata_fragment(self,part_index: int,axial: int) -> tuple:
        try:
            # Ensure part is within u16 range (0 to 65535)
            if not (0 <= part_index <= 0xFFFF):
                raise ValueError("part must be a 16-bit unsigned integer (0-65535)")
            
            logging.debug("Run cmd_read_rawdata_fragment")
            cmd = bytes([0x67, 0x9A])
            length_size = 2

            # Construct the payload for the command
            payload = bytearray(cmd) + part_index.to_bytes(2, byteorder='big') + bytes([axial])
            
            
            logging.debug(f"Send message: {payload.hex().upper()}")
            self.serial_comm.write(bytes(payload))
            
            # MCU Response
            while True:
                response = self.serial_comm.read_until(length_size)
                if response:
                    logging.debug(f"Response message: {response.hex().upper()}")

                    cmd_got = response[1+length_size:1+length_size+2]
                    if cmd_got != cmd:
                        # The instructions are different, re-fetch the reply
                        logging.error(f"Cmd mismatch: expected {cmd.hex().upper()},got {cmd_got.hex().upper()}")
                        continue                    
                                    
                    # Get the current status from the response
                    current_status = response[5]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["OK", "Fail"]
                    logging.debug(f"Current status: {self._status_display_string(current_status, status_string)}")
                    
                    if current_status == 0x00:                                        
                        data = response[6:-3]  # Ensure this slice is correct based on the response format

                        return data
                    
                    # Return status indicating failure and None for data
                    return None
                else:
                   return None
        
        except Exception as e:
            logging.error(f"Read Raw Data Fragment Error: {e}")
            return None
    
    def _status_display_string(self,index: int, display: List[str]) -> str:
        #print(f"index: {index};display: {display} ")
        if index >= len(display):
            return f"Value: {index}"
        return display[index]



# Example usage
if __name__ == "__main__":
    serial_comm = SerialCommunication('/dev/ttyUSB0', 115200)
    serial_comm.connect()
    
    try: 
        mode = "continuous"
        bandWidth = 4
        length = 2
        unit = 0
        trigger = int(0.1*1000) 

        cmd = InstructionSet(serial_comm)
        size_array,allData=cmd.start_data_capture(mode,bandWidth,length,unit,trigger)
        
        if allData:
            logging.info("Data capture successful")
            with open('output.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(allData)
        else:
            logging.info("Data capture unsuccessful")
    
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        serial_comm.close()
