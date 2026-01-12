#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#





import time
import usb
import copy
import serial
from serial.tools import list_ports
import glob

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_drvs"
logger = Logger(log_name = log_name)



#######################
### Serial Port Utility Functions

STANDARD_BUAD_RATES = [110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

SERIAL_DEVICE_DICT = dict(    
                        port = "",
                        baudrate = 0,
                        addr_str = "",
                        manf_str = "None",
                        vendor_id = 0,
                        product_id = 0,
                        connected = False
                   )


def get_serial_ports_list():
    ports_list = []
    ports = list_ports.comports()
    for loc, desc, hwid in sorted(ports):
        ports_list.append(loc)
    #self.logger.log_warn("ports: " + str(ports))###
    add_ports = sorted(set(glob.glob('/dev/ttyTHS*')))
    add_ports = add_ports + sorted(set(glob.glob('/dev/ttyTCU*')))
    #self.logger.log_warn("add port: " + str(add_ports))###
    for add_port in add_ports:
        if add_port not in ports:
            ports_list.append(add_port)
    #self.logger.log_warn("ports list: " + str(self.ports_list))###

    return ports_list



# Function for getting info for all available (non-used) serial ports
def get_serial_ports_dict_list():
  port_dicts = dict()
  usb_ports = usb.core.find(find_all=True)
  ports = get_serial_ports_list() #get_serial_ports_list()
  for port in sorted(ports):
    device_dict = SERIAL_DEVICE_DICT
    device_dict = get_serial_device_dict(port)
    port_dicts[port.port] = device_dict
  return port_dicts
  

def get_serial_device_dict(serial_port):
    usb_ports = usb.core.find(find_all=True)
    device_dict = SERIAL_DEVICE_DICT
    try:
        device_dict['port'] = serial_port.port
        device_dict['baudrate'] = serial_port.baudrate
        device_dict['addr_str'] = addr_str
        device_dict["vender_id"] = serial_port.vid
        device_dict["manf_str"] = serial_port.manufacturer
        product_id = 0
        for usb_port in usb_ports:
            if usb_port.idVendor == serial_port.vid:
                product_id = usb_port.idProduct
                break
        device_dict["product_id"] = product_id
    except Exception as e:
        logger.log_debug("Failed to get serial device info: " + str(e))
    return device_dict


def read_witmotion_packet(serial_port):
    while True:
        byte = serial_port.read(1)
        if byte == b'\x55':  # Look for the start byte (0x55)
            packet = byte + serial_port.read(10)  # Read the remaining 10 bytes
            return packet
        time.sleep(0.01) # Small delay to avoid busy waiting

import struct

def decode_witmotion_message(data):
    print("Got data: " + str(data))
    if not data or len(data) < 11 or data[0] != 0x55: # Check for start byte and minimum length
        return None

    # Calculate checksum (example - adjust based on your specific sensor's method)
    calculated_checksum = sum(data[1:-1]) & 0xFF  # Example: sum of bytes excluding start and checksum
    received_checksum = data[-1]

    if calculated_checksum != received_checksum:
        print("Checksum mismatch!") # Handle checksum errors
        return None

    # Decode data based on the specific WitMotion sensor model and protocol
    # Example: Decode 3D acceleration, 3D angular velocity, 3D angle
    # Refer to your sensor's manual for specific data format and scaling factors
    try:
        (accel_x, accel_y, accel_z,
         gyro_x, gyro_y, gyro_z,
         angle_x, angle_y, angle_z,
         temp) = struct.unpack('<hhhhhhhhhB', data[1:-1]) # Example format string

        # Apply scaling factors if necessary (refer to the sensor's manual)

        decoded_data = {
            'acceleration': (accel_x, accel_y, accel_z),
            'angular_velocity': (gyro_x, gyro_y, gyro_z),
            'angle': (angle_x, angle_y, angle_z),
            'temperature': temp,
        }

        return decoded_data

    except struct.error as e:
        print(f"Error decoding data: {e}")
        return None


# Function for serial send/receive check
def listen_serial_message(serial_port, 
                        wait_time = 0.010):
    response = None
    try:
        #nepi_sdk.sleep(wait_time)
        bs = read_witmotion_packet(serial_port) #serial_port.readline()
        response = bs.decode() #decode_witmotion_message(bs) #bs.decode('utf-8').strip()
        msg = ("Got a serial binary response: " + str(response))
        print(str(msg))
        logger.log_debug(msg)
    except Exception as e:
        msg = "Got a serial read/write error: " + str(e)
        print(str(msg))
        logger.log_debug(msg)
    print('All Done with listen on port')
    return response

# Function for serial send/receive check
def listen_serial_port(port_str, 
                    baudrate = 9600, 
                    wait_time = 0.010,
                    verbose = False):
    response = None
    if verbose == True:
            msg = ("Starting serial device product id check " + str(port_str))
            print(msg)
            logger.log_info(msg)
    print('Got Here1')
    serial_port = None
    try:
        # Try and open serial port
        serial_port = serial.Serial(port_str,baudrate,timeout = wait_time)
        nepi_sdk.sleep(0.1)
        print('Got Here2')
    except Exception as e:
        msg = ("Unable to open serial port " + str(port_str) + " with baudrate: " + str(baudrate) + " " + str(e))
        print(msg)
        logger.log_info(msg)

    if serial_port is not None:
        print('Got Here3')
        response = listen_serial_message(serial_port = serial_port,wait_time = wait_time)
        msg = ("Listener got response " + str(port_str) + " with baudrate: " + str(baudrate) + " " + str(response))
        print(msg)
        # Clean up the serial port
        serial_port.close()
    return response
# Function for tserial send/receive check for list of ports, addrs, baud_rates 



def listen_serial_ports(
                    port_str_list = None,
                    buadrate_list = STANDARD_BUAD_RATES,
                    wait_time = 0.010,
                    verbose = False):

    if port_str_list is None:
        port_str_list = get_serial_ports_list()

    print(str(port_str_list))
    for port_str in port_str_list:
        print("a")
        for baudrate in buadrate_list:
                if verbose == True:
                    msg = ("Listeing for serial device " + str(port_str) + " baudrate: " + str(baudrate))
                    print(msg)
                    logger.log_info(msg)
                response = listen_serial_port(
                                        port_str, 
                                        baudrate = baudrate, 
                                        verbose = verbose)
        
                if verbose == True:
                    msg = ("Got response: " + str(response))
                    print(msg)
                    logger.log_info(msg)





# Function for checking if serial_port is available
def check_for_serial_port(port_str):
    success = False
    serial_port = list_ports.comports()
    for loc, desc, hwid in sorted(serial_port):
      if loc == port_str:
        success = True
    return success

# Function for serial send/receive check
def send_serial_message(serial_port, 
                        message_str = '', 
                        include_cr = True, 
                        include_lf = True, 
                        wait_time = 0.010):
    # Add carriage return and line feed if needed
    ser_str= message_str
    if include_cr == True:
        ser_str = (ser_str + '\r')
    if include_lf == True:
        ser_str = (ser_str + '\n')
    ################################################  
    # Send Serial String
    logger.log_debug("")
    logger.log_debug("Sending serial message: " + message_str)
    b=bytearray()
    b.extend(map(ord, ser_str))
    response = None
    try:
        serial_port.write(b)
        logger.log_debug("Waiting for response")
        nepi_sdk.sleep(wait_time)
        bs = serial_port.readline()
        response = bs.decode()
    except Exception as e:
        logger.log_debug("Got a serial read/write error: " + str(e))
    return response





# Function for serial send/receive check
def check_serial_port_by_product_id(port_str, 
                                    baudrate = 9600, 
                                    product_id = 0,
                                    wait_time = 0.010,
                                    verbose = False):
    valid = False
    device_dict = None
    if verbose == True:
            msg = ("Starting serial device product id check " + str(port_str))
            print(msg)
            logger.log_info(msg)
    try:
        # Try and open serial port
        serial_port = serial.Serial(port_str,baudrate,timeout = 1)
    except Exception as e:
        msg = ("Unable to open serial port " + str(port_str) + " with baudrate: " + str(baudrate) + " " + str(e))
        print(msg)
        logger.log_info(msg)
        return device_dict
    dd = get_serial_device_dict(serial_port)
    if device_dict is not None:
        if product_id == dd['product_id']:
            valid = True
    if valid:
        device_dict = dd
    # Clean up the serial port
    serial_port.close()
    return device_dict
# Function for tserial send/receive check for list of ports, addrs, baud_rates 

def check_serial_ports_by_product_id(
                    port_str_list = None,
                    buadrate_list = STANDARD_BUAD_RATES,
                    product_id = 0,
                    include_cr = True, 
                    include_lf = True,
                    wait_time = 0.010,
                    verbose = False):

    device_dict = None
    if port_str_list is None:
        port_str_list = get_serial_ports_list()

    for port_str in port_str_list:
        for baudrate in buadrate_list:
                if verbose == True:
                    msg = ("Checking for serial device " + str(port_str) + " product_id: " + str(product_id))
                    print(msg)
                    logger.log_info(msg)
                device_dict = check_serial_port_by_product_id(
                                                        port_str, 
                                                        baudrate = baudrate, 
                                                        product_id = product_id, 
                                                        include_cr = include_cr, 
                                                        include_lf = include_lf,
                                                        verbose = verbose)
                if device_dict is not None:
                    device_dict['addr_str'] = addr_str
                    if verbose == True:
                        msg = ("Found Device: " + str(device_dict))
                        print(msg)
                        logger.log_info(msg)
                    return device_dict
    return device_dict




    
def update_msg_to_length(message, length = None, length_prefix = '', length_suffix = ''):
    msg_str = message
    if length == None or lenght == 0 or len(message) >= length or (length_prefix == '' and \
                    length_suffix == '') or (length_prefix != '' and length_suffix != ''):
        return msg_str
    for i in range(length):
        data_str = length_prefix + data_str + length_suffix


def create_serial_port_addrs_list(start_str='1', stop_str='2', length = None, length_prefix = '', length_suffix = ''):
    addr_list = []

    # First Check if Int
    try:
      start_addr = int(start_str)
      stop_addr = int(stop_str)
      addr_range = stop_addr - start_addr
      if addr_range > 0:
        self.addr_list = list(range(start_addr,stop_addr+1))
      else:
        self.addr_list = [start_addr]    
    except:
        pass

    # Check if letters with matching cases
    if len(start_str) == 1 and len(stop_str) == 1:
        if start_str.isalpha() and stop_str.isalpha():
            if start_str.islower() and stop_str.islower():
                try:
                    letters = nepi_utils.get_lowercase_letters()
                    start_ind = letters.index(start_str)
                    stop_ind = letters.index(stop_str)
                    addr_list = list(letters[start_ind:stop_ind + 1])
                except:
                    pass
            if start_str.isupper() and stop_str.isupper():
                try:
                    letters = nepi_utils.get_uppercase_letters()
                    start_ind = letters.index(start_str)
                    stop_ind = letters.index(stop_str)
                    addr_list = list(letters[start_ind:stop_ind + 1])
                except:
                    pass

    # Adjust for length if needed
    addr_list_adj = []
    for addr in addr_list:
        adj_addr = update_msg_to_length(addr, length = length, length_prefix = length_prefix, length_suffix = length_suffix)
        addr_list_adj.append(adj_addr)
    return addr_list_adj

        



# Function for serial send/receive check
def check_serial_port_by_message(
                                port_str, 
                                baudrate = 9600, 
                                message_str = '', 
                                response_test_function = None, 
                                include_cr = True, 
                                include_lf = True,
                                wait_time = 0.010,
                                verbose = False
                                ):
    valid = False
    device_dict = None
    if verbose == True:
            msg = ("Starting serial device check " + str(port_str))
            print(msg)
            logger.log_info(msg)
    try:
        # Try and open serial port
        serial_port = serial.Serial(port_str,baudrate,timeout = 1)
    except Exception as e:
        msg = ("Unable to open serial port " + str(port_str) + " with baudrate: " + str(baudrate) + " " + str(e))
        print(msg)
        logger.log_info(msg)
        return device_dict
    # Add carriage return and line feed if needed
    if verbose == True:
        msg = ("Sending serial msg " + str(message_str))
        print(msg)
        logger.log_info(msg)
    response = send_serial_message(serial_port, 
                            message_str = message_str, 
                            include_cr = include_cr, 
                            include_lf = include_lf, 
                            wait_time = wait_time)
    if response is not None:
        if verbose == True:
                msg = ("Got serial response " + str(response))
                print(msg)
                logger.log_info(msg)
        valid = False
        if response_test_function is not None:
            try:
                valid = response_test_function(message_str,response)
            except:
                pass
        else:
            valid = (response == message_str)
        if valid:
            device_dict = get_serial_device_dict(serial_port)
    # Clean up the serial port
    serial_port.close()
    return device_dict



# Function for tserial send/receive check for list of ports, addrs, baud_rates 
def check_serial_ports_by_message(
                    port_str_list = None,
                    message_start_str = '', 
                    message_addr_str_list = [''], 
                    message_stop_str = '',
                    response_test_function = None, # Will use echo of command if None
                    buadrate_list = STANDARD_BUAD_RATES,
                    include_cr = True, 
                    include_lf = True,
                    wait_time = 0.010,
                    verbose = False):

    device_dict = None
    print("Got ports: " + str(port_str_list))
    if port_str_list is None:
        port_str_list = get_serial_ports_list()
        print("Found ports: " + str(port_str_list))
    print("Searching ports: " + str(port_str_list))

    for port_str in port_str_list:
        for baudrate in buadrate_list:
            for addr_str in message_addr_str_list:
                message_str = message_start_str + addr_str + message_stop_str

                if verbose == True:
                    msg = ("Checking for serial device " + str(port_str) + " baudrate: " + str(baudrate) + " message: " + str(message_str))
                    print(msg)
                    logger.log_info(msg)
                device_dict = check_serial_port_by_message(
                                                        port_str, 
                                                        baudrate = baudrate, 
                                                        message_str = message_str,
                                                        response_test_function = response_test_function, 
                                                        include_cr = include_cr, 
                                                        include_lf = include_lf,
                                                        verbose = verbose)
                if device_dict is not None:
                    device_dict['addr_str'] = addr_str
                    if verbose == True:
                        msg = ("Found Device: " + str(device_dict))
                        print(msg)
                        logger.log_info(msg)
                    return device_dict
    return device_dict


def create_serial_port_addrs_list(start_str='1', stop_str='2', length = None, length_prefix = '', length_suffix = ''):
    addr_list = []

    # First Check if Int
    try:
      start_addr = int(start_str)
      stop_addr = int(stop_str)
      addr_range = stop_addr - start_addr
      if addr_range > 0:
        self.addr_list = list(range(start_addr,stop_addr+1))
      else:
        self.addr_list = [start_addr]    
    except:
        pass

    # Check if letters with matching cases
    if len(start_str) == 1 and len(stop_str) == 1:
        if start_str.isalpha() and stop_str.isalpha():
            if start_str.islower() and stop_str.islower():
                try:
                    letters = nepi_utils.get_lowercase_letters()
                    start_ind = letters.index(start_str)
                    stop_ind = letters.index(stop_str)
                    addr_list = list(letters[start_ind:stop_ind + 1])
                except:
                    pass
            if start_str.isupper() and stop_str.isupper():
                try:
                    letters = nepi_utils.get_uppercase_letters()
                    start_ind = letters.index(start_str)
                    stop_ind = letters.index(stop_str)
                    addr_list = list(letters[start_ind:stop_ind + 1])
                except:
                    pass

    # Adjust for length if needed
    addr_list_adj = []
    for addr in addr_list:
        adj_addr = update_msg_to_length(addr, length = length, length_prefix = length_prefix, length_suffix = length_suffix)
        addr_list_adj.append(adj_addr)
    return addr_list_adj

        


    
def update_msg_to_length(message, length = None, length_prefix = '', length_suffix = ''):
    msg_str = message
    if length == None or lenght == 0 or len(message) >= length or (length_prefix == '' and \
                    length_suffix == '') or (length_prefix != '' and length_suffix != ''):
        return msg_str
    for i in range(length):
        data_str = length_prefix + data_str + length_suffix

  
