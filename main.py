import serial
import time
import struct
import csv
from typing import List
import numpy as np

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
                timeout=1
            )
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")

    def close(self):
        if self.ser is not None:
            self.ser.close()
            print("Serial port closed.")        

    def write(self, data: bytes):
        if self.ser is not None:
            try:
                self.ser.write(data)
                print(f"Sent: {data.hex().upper()}")
            except Exception as e:
                print(f"Error writing to serial port: {e}")
        else:
            print("Serial port is not open.")    

    def read_until(self,length_size: int) -> bytes:        
        start_byte = b'\x5B'  # Hexadecimal for '['
        stop_byte = b'\x5D'   # Hexadecimal for ']'
        
        if self.ser is not None:            
            try:                
                response = bytearray()
                start_found = False
                length_found = False

                while True:
                    if not start_found:    
                        while True:
                            if self.ser.in_waiting > 0:
                                byte = self.ser.read(1)  # Read one byte at a time                        
                                if byte == start_byte:
                                    response.extend(byte)
                                    start_found = True
                                    break

                    if not length_found:            
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

                    while True:
                        if self.ser.in_waiting >= command_length-len(response):
                            byte = self.ser.read(command_length-len(response))
                            response.extend(byte)
                            break

                    if response[-1] == stop_byte[0]:
                        break  # Found the stop byte, exit loop

                    else:
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


                    '''
                    if self.ser.in_waiting > 0:
                        print(f"Buffer: {self.ser.in_waiting}")
                        byte = self.ser.read(1)  # Read one byte at a time
                        
                        if not start_found:
                            if byte == start_byte:
                                response.extend(byte)
                                start_found = True  # Mark that we found the start byte
                        else:
                            response.extend(byte)
                            
                            if length_size == 1:
                                # Check if we have at least two bytes to determine command length
                                if len(response) == 2:                                    
                                    command_length = response[1]  # Get command length
                            else:
                                if len(response) == 3:
                                    command_length = int.from_bytes(response[1:3], byteorder='big')  # Get command length

                            # Check if we have collected enough bytes (length includes start byte)
                            if len(response) == command_length:  # Length includes start byte
                                # Ensure the last byte is the stop byte
                                if response[-1] == stop_byte[0]:
                                    break  # Found the stop byte, exit loop
                                else:
                                    # If stop byte is incorrect, reset to search for start byte again
                                    if start_byte in response:
                                        start_index = response.index(start_byte)
                                        response = response[start_index:]  # Keep from the start byte onward  
                                        
                                        if len(response) >= length_size+1: 
                                            if length_size == 1:
                                                command_length = response[1]  # Update command length
                                            else:
                                               command_length = int.from_bytes(response[1:3], byteorder='big')
                                    
                                    else:                                        
                                        response.clear()  # Clear response to start over
                                        start_found = False
                    '''
                         
                                    
                return bytes(response)
            except Exception as e:
                print(f"Error reading from serial port: {e}")
                return b''
        else:
            print("Serial port is not open.")
            return b''    

class InstructionSet:
    def __init__(self, SerialCommunication):
        self.serial_comm = SerialCommunication

    def continuous_mode(self, bandWidth: int, length: int, unit: int)-> tuple:
        try:
            # Start continuous mode and check if it was successful
            if self.continuous_mode_start(bandWidth, length, unit):
                while True:
                    # Read the operation status
                    status = self.read_operation_status() 
                    #print(f"Current status: {status:#04x}")  # Print status in hexadecimal format

                    # Break the loop if the status is 0x02
                    if status == 0x02:
                        print("Data capture completed.")
                        status, part, size_array = self.read_operation_details()

                        if status:
                            allData = []
                            for j in range(3):
                                axial_data = []
                                for i in range(part):
                                    status, float_values = self.read_rawdata(i+1,j+1)
                                    if status:
                                        axial_data.extend(float_values)

                                    # print(f"axial Data (f32 values): {axial_data}")
                            
                                allData.append(axial_data)

                            return True,part, size_array,np.array(allData).T.tolist()
                        
                        else:
                            return False,None, None,None
                    
                    # Introduce a delay of 0.5 seconds before the next status check
                    time.sleep(0.5)
            else:
                return False,None, None,None

        except Exception as e:
            print(f"Continuous mode Error: {e}")
    
    def continuous_mode_start(self, bandWidth: int, length: int, unit: int) -> bool:
        try:
            # Construct the payload
            payload = bytearray([0xBA, 0x45, bandWidth, length, unit])
            message = self.combined_send_command(payload)

            print(f"Send message: {message.hex().upper()}")
            self.serial_comm.write(bytes(message))        
            
            # Read until a specific stop byte
            response = self.serial_comm.read_until(1)
            if response:
                print(f"Response message: {response.hex().upper()}")
                
                # Get the current status from the response
                current_status = response[4]
                print(f"Current status: {current_status}")
                status_string = ["OK", "Wrong band width or length.", "Fail"]
                print(f"Current status: {self.status_display_string(current_status, status_string)}")

                # Check if the status is OK
                return current_status == 0x00

        except Exception as e:
            print(f"Continuous mode start Error: {e}")
            return False
        
    def trigger_mode_start(self,bandWidth: int,length: int,unit: int,trigger: int)->bool:
        try:
            # Construct the payload
            payload = bytearray([0xA9, 0x56, bandWidth, length, unit])
            payload.extend(trigger.to_bytes(2, byteorder='big'))
            message = self.combined_send_command(payload)

            print(f"Send message: {message.hex().upper()}")
            self.serial_comm.write(bytes(message))        
            
            # Read until a specific stop byte
            response = self.serial_comm.read_until(1)
            if response:
                print(f"Response message: {response.hex().upper()}")
                
                # Get the current status from the response
                current_status = response[4]
                status_string = ["OK", "Wrong band width or length.", "Fail"]
                print(f"Current status: {self.status_display_string(current_status, status_string)}")

                # Check if the status is OK
                return current_status == 0x00

        except Exception as e:
            print(f"Trigger mode start Error: {e}")
            return False
        
    def interrupt_mode(self)->bool:
        try:
            # Construct the payload
            payload = bytearray([0x98, 0x67])
            message = self.combined_send_command(payload)

            print(f"Send message: {message.hex().upper()}")
            self.serial_comm.write(bytes(message))        
            
            # Read until a specific stop byte
            response = self.serial_comm.read_until(1)
            if response:
                print(f"Response message: {response.hex().upper()}")
                
                # Get the current status from the response
                current_status = response[4]
                status_string = ["OK", "Fail"]
                print(f"Current status: {self.status_display_string(current_status, status_string)}")

                # Check if the status is OK
                return current_status == 0x00

        except Exception as e:
            print(f"Interrupt Mode Error: {e}")
            return False
        
    def read_operation_status(self)->int:
        try:
            # Construct the payload
            payload = bytearray([0x87, 0x78])
            message = self.combined_send_command(payload)

            print(f"Send message: {message.hex().upper()}")
            self.serial_comm.write(bytes(message))        
            
            # Read until a specific stop byte
            response = self.serial_comm.read_until(1)
            if response:
                print(f"Response message: {response.hex().upper()}")
                
                # Get the current status from the response
                current_status = response[4]
                status_string = ["Idle", "In Progress", "Completed","Wait for Trigger"]
                print(f"Current status: {self.status_display_string(current_status, status_string)}")

                # Check if the status is OK
                return current_status

        except Exception as e:
            print(f"Read operation status Error: {e}")
            return -1
        
    def read_operation_details(self) -> tuple:
        try:
            # Construct the payload for the command
            payload = bytearray([0x78, 0x89])
            message = self.combined_send_command(payload)

            print(f"Send message: {message.hex().upper()}")
            self.serial_comm.write(bytes(message))        
            
            # Read until a specific stop byte
            response = self.serial_comm.read_until(1)
            if response:
                print(f"Response message: {response.hex().upper()}")
                
                # Get the current status from the response
                current_status = response[4]
                status_string = ["OK", "Fail"]
                print(f"Current status: {self.status_display_string(current_status, status_string)}")

                
                if current_status == 0x00:
                    bandWidth = response[5]
                    length = response[6]
                    unit = response[9]
                    
                    bandWidth_string = ["NAN", "1K", "1.25K", "1.667K", "2.5K", "5K", "10K"]
                    unit_string = ["Acceleration", "Velocity"]

                    print(f"BandWidth: {self.status_display_string(bandWidth, bandWidth_string)} "
                        f"Length: {length} "
                        f"Unit: {self.status_display_string(unit, unit_string)}")

                    # Extract part as a u16 (16-bit unsigned integer)
                    part = int.from_bytes(response[7:9], byteorder='big')
                    print(f"Part (u16): {part}")

                    # Extract sizeArray and convert to u16 (16-bit unsigned integers)
                    sizeArray = response[10:-3]  # Ensure this slice is correct based on the response format
                    u16_values = []
                    
                    # Iterate through the sizeArray in chunks of 2 bytes (for u16)
                    for i in range(0, len(sizeArray), 2):
                        if i + 2 <= len(sizeArray):
                            u16_value = int.from_bytes(sizeArray[i:i + 2], byteorder='big')
                            u16_values.append(u16_value)
                    
                    print(f"Size Array (u16 values): {u16_values}")

                    # Return current status (0x00 for success)
                    return current_status == 0x00, part, u16_values

        except Exception as e:
            print(f"Read operation status Error: {e}")
            return False, None, None
    
    def read_rawdata(self,part: int,axial: int) -> tuple:
        try:
            # Ensure part is within u16 range (0 to 65535)
            if not (0 <= part <= 0xFFFF):
                raise ValueError("part must be a 16-bit unsigned integer (0-65535)")

            # Construct the payload for the command
            payload = bytearray([0x67, 0x9A]) + part.to_bytes(2, byteorder='big') + bytes([axial])
            message = self.combined_send_command(payload)
            
            print(f"Send message: {message.hex().upper()}")
            self.serial_comm.write(bytes(message))
            
            # Read until a specific stop byte
            response = self.serial_comm.read_until(2)
            if response:
                # print(f"Response message: {response.hex().upper()}")
                
                # Get the current status from the response
                current_status = response[5]
                status_string = ["OK", "Fail"]
                print(f"Current status: {self.status_display_string(current_status, status_string)}")
                
                if current_status == 0x00:                                        
                    sizeArray = response[6:-3]  # Ensure this slice is correct based on the response format

                    # Convert sizeArray bytes to float32 values
                    float_values = []
                    for i in range(0, len(sizeArray), 4):  # 4 bytes per float32
                        if i + 4 <= len(sizeArray):
                            float_value = struct.unpack('>f', sizeArray[i:i + 4])[0]  # Big-endian
                            float_values.append(float_value)                    

                    # print(f"Size Array (f32 values): {float_values}")
                    # Return current status (0x00 for success)
                    return True, float_values
                
                # Return status indicating failure and None for data
                return False, None
        
        except Exception as e:
            print(f"Read operation status Error: {e}")
            return False, None
    
    
    
    def crc16(self,data: bytes, poly=0xA001):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= poly
                else:
                    crc >>= 1
        return crc
    
    def combined_send_command(self,payload: bytes)-> bytearray:
        cmd = bytearray([0x53,0x53,0x00])
        stop_bytes = bytes([0x53, 0x54])

        cmd.extend(payload)
        cmd[2] = len(cmd)+2+2 #Checksum(2).End Bytes(2)
        cmd.extend(self.crc16(cmd).to_bytes(2, byteorder='little'))
        cmd.extend(stop_bytes)

        return cmd
    
    def status_display_string(self,index: int, display: List[str]) -> str:
        #print(f"index: {index};display: {display} ")
        if index >= len(display):
            return f"Value: {index}"
        return display[index]



# Example usage
if __name__ == "__main__":
    serial_comm = SerialCommunication('/dev/ttyUSB0', 115200)
    serial_comm.connect()
    
    try:
        
        cmd = InstructionSet(serial_comm)
        
        status,part, size_array,allData=cmd.continuous_mode(6,2,1)
        if status:
            with open('output.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(allData)
        
        print(f"Size Array (u16 values): {size_array}")
    
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        serial_comm.close()
