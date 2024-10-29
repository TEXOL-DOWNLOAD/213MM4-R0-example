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
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate        
        self.ser = None
        

    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port, 
                self.baudrate,
                bytesize=8,
                parity=serial.PARITY_EVEN,  # Use serial.PARITY_EVEN for 'E'
                stopbits=1,
                timeout=5
            )
            logging.info(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            logging.error(f"Error connecting to serial port: {e}")

    def close(self):
        if self.ser is not None:
            self.ser.close()
            logging.info("Serial port closed.")        

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

    def read_until(self,length_size: int) -> bytes: 
        if self.ser is not None: 
            try:
                start_byte = b'\x5B' # Hexadecimal for '['
                stop_byte = b'\x5D'  # Hexadecimal for ']' b'\x5D'
                
                response = bytearray()
                start_found = False
                length_found = False
                command_length = 0

                while True:
                    if not start_found:                        
                        if not self._find_start_byte(response,start_byte):   
                            logging.error("Read start byte timed out.")
                            return b''  # Return empty bytes on timeout                         
                            
                        logging.debug(f"Response message start_byte: {response.hex().upper()}")
                        '''
                        while True:
                            if self.ser.in_waiting > 0:
                                byte = self.ser.read(1)  # Read one byte at a time
                                if byte == start_byte:
                                    response.extend(byte)
                                    start_found = True
                                    break
                        '''
                        

                    if not length_found:
                        command_length = self._read_command_length(length_size,response)
                        if command_length is None:
                            logging.error("Read command length timed out.")
                            return b''  # Return empty bytes on timeout 
                        
                        logging.debug(f"Response message command_length: {response.hex().upper()}")
                        '''
                        while True:
                            if self.ser.in_waiting >= length_size:
                                byte = self.ser.read(length_size)  # Read one byte at a time
                                response.extend(byte)
                                length_found = True

                                if length_size == 1:                                                            
                                    command_length = response[1]  # Get command length
                                else:                            
                                    command_length = int.from_bytes(response[1:3], byteorder='big')  # Get command length
                                                                
                                break
                        '''          

                    byte = self.ser.read(command_length-len(response))
                    if byte and len(byte) == command_length-len(response):
                       response.extend(byte)
                    else:
                        logging.error("Read command timed out.")
                        return b''  # Return empty bytes on timeout 
                    
                    logging.debug(f"Response message: {response.hex().upper()}")
                    '''
                    while True:
                        if self.ser.in_waiting >= command_length-len(response):
                            byte = self.ser.read(command_length-len(response))
                            
                            break
                    '''
                    
                    

                    if  response[-1] == stop_byte[0]:
                        crc_expected = self._crc16(response[:-3])
                        crc_got = response[-3:-1]

                        if crc_expected == crc_got:
                            return bytes(response)
                            # break  # Found the stop byte, exit loop
                        else:
                            logging.error(f"checksum: expected {crc_expected.hex().upper()},got {crc_got.hex().upper()}")
                            return b''  # Return empty bytes on timeout
                    
                    if start_byte in response:
                        start_found = True
                        start_index = response.index(start_byte)
                        response = response[start_index:]  # Keep from the start byte onward  
                        
                        if len(response) >= length_size+1: 
                            length_found = True
                            if length_size == 1:
                                command_length = response[1]  # Update command length
                            else:
                                command_length = int.from_bytes(response[1:3], byteorder='big')
                        else:
                            length_found = False
                    
                    else:
                        start_found = False
                        length_found = False

                #logging.error("Read operation timed out.")
                #return b''  # Return empty bytes on timeout
            
            except Exception as e:
                logging.error(f"Error reading from serial port: {e}")
                return b''
        
        else:
            logging.warning("Serial port is not open.")
            return b''     

    def _find_start_byte(self, response: bytearray,start_byte: bytes) -> bool:
        #start_byte = b'\x5B' # Hexadecimal for '['
        while True: 
            byte = self.ser.read(1)
            if byte:                
                if byte == start_byte:
                    response.extend(byte)
                    return True
            else:                
                return False # Return False if no byte is read
    
    def _read_command_length(self, length_size: int, response: bytearray) -> int:        
        # if self.ser.in_waiting >= length_size:
        byte = self.ser.read(length_size)
        if byte:
            response.extend(byte)
            if length_size == 1:
                return response[1]  # Command length
            else:
                return int.from_bytes(response[1:3], byteorder='big')
            
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
    
    def _combined_send_command(self,payload: bytes)-> bytearray:
        cmd = bytearray([0x53,0x53,0x00])
        stop_bytes = bytes([0x53, 0x54])

        cmd.extend(payload)
        cmd[2] = len(cmd)+2+2 #Checksum(2).End Bytes(2)
        cmd.extend(self._crc16(cmd))
        cmd.extend(stop_bytes)

        return cmd
    
    

class InstructionSet:
    def __init__(self, SerialCommunication):
        self.serial_comm = SerialCommunication

    def start_data_capture(self, mode: str, bandWidth: int, length: int, unit: int, trigger: int = None) -> tuple:
        """
        Starts data capture mode (either continuous or trigger).

        Args:
            mode (str): The mode of capture ("continuous" or "trigger").
            bandWidth (U8): The bandwidth for data capture ("NAN", "1K", "1.25K", "1.667K", "2.5K", "5K", "10K")
            length (U8): The duration (Sec) of the capture
            unit (int): The unit of measurement for the capture (0x00 : acceleration, 0x01 : velocity).
            trigger (int, optional): Trigger value for trigger mode. (scale: 0.001)

        Returns:
            tuple: A tuple containing an size array, and data, 
                or None values on failure.
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
        try:
            logging.debug("Run cmd_continuous_mode_start")
            cmd = bytes([0xBA, 0x45])
            length_size = 1
            
            # Construct the payload            
            payload = bytearray(cmd )+ bytes([bandWidth, length, unit])         
            
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
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["OK", "Wrong band width or length.", "Fail"]
                    logging.info(f"Continuous mode start cmd status: {self._status_display_string(current_status, status_string)}")

                    # Check if the status is OK
                    return current_status == 0x00
                else:
                    return False

        except Exception as e:
            logging.error(f"Continuous mode start Error: {e}")
            return False
        
    def cmd_trigger_mode_start(self,bandWidth: int,length: int,unit: int,trigger: int)->bool:
        try:
            logging.debug("Run cmd_trigger_mode_start")
            cmd = bytes([0xA9, 0x56])
            length_size = 1
            
            # Construct the payload            
            payload = bytearray(cmd) + bytes([bandWidth, length, unit])+trigger.to_bytes(2, byteorder='big')            
            #bytes([bandWidth, length, unit])
            #payload.extend(trigger.to_bytes(2, byteorder='big'))
            

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
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["OK", "Wrong band width or length.", "Fail"]
                    logging.info(f"Trigger mode start cmd status: {self._status_display_string(current_status, status_string)}")

                    # Check if the status is OK
                    return current_status == 0x00
                else:
                    return False

        except Exception as e:
            logging.error(f"Trigger mode start Error: {e}")
            return False
        
    def cmd_interrupt_mode(self)->bool:
        try:

            logging.debug("Interrupt capture")
            cmd = bytes([0x98, 0x67])
            length_size = 1
            
            # Construct the payload            
            payload = bytearray(cmd)            

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
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["OK", "Fail"]
                    logging.info(f"Interrupt mode cmd status: {self._status_display_string(current_status, status_string)}")

                    # Check if the status is OK
                    return current_status == 0x00
                else:
                    return False
                
        except Exception as e:
            logging.error(f"Interrupt Mode Error: {e}")
            return False
        
    def cmd_read_operation_status(self)->int:
        try:
            logging.debug("Run cmd_read_operation_status")
            cmd = bytes([0x87, 0x78])
            length_size = 1
            
            # Construct the payload            
            payload = bytearray(cmd)

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
                    current_status = response[4]
                    logging.debug(f"Current status: {current_status}")
                    status_string = ["Idle", "In Progress", "Completed","Wait for Trigger", "Fail"]
                    logging.info(f"Read operation status cmd status: {self._status_display_string(current_status, status_string)}")

                    # Check if the status is OK
                    return current_status
                else:
                    return None
                
        except Exception as e:
            logging.error(f"Read operation status Error: {e}")
            return None
        
    def cmd_read_operation_details(self) -> tuple:
        """
        Returns:
            tuple: A tuple containing a success flag, part number, data size array,
                or None values on failure.
        """
        try:
            logging.debug("Run cmd_read_operation_details")
            cmd = bytes([0x78, 0x89])
            length_size = 1
            
            # Construct the payload            
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
