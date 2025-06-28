#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#





import time
import usb
import copy
import serial

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_drvs"
logger = Logger(log_name = log_name)



#######################
### Serial Port Utility Functions

STANDARD_BUAD_RATES = [110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

SERIAL_DEVICE_DICT = dict(    
                        serial_port = "",
                        baudrate = 0,
                        addr_str = "",
                        manf_str = "None",
                        vendor_id = 0,
                        product_id = 0,
                        connected = False
                   )
                  
# Function for getting info for all available (non-used) serial ports
def get_serial_ports_dict_list():
  port_dicts = dict()
  usb_ports = usb.core.find(find_all=True)
  ports = serial.tools.list_port.comports()
  for serial_port in sorted(ports):
    device_dict = SERIAL_DEVICE_DICT
    device_dict = get_serial_device_dict(serial_port)
    port_dicts[serial_port.port] = device_dict
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
  return device_dict



# Function for checking if serial_port is available
def check_for_serial_port(port_str):
    success = False
    serial_port = serial.tools.list_port.comports()
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
        ser_str = (ser_msg + '\r')
    if include_lf == True:
        ser_str = (ser_msg + '\n')
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
        logger.log_degub("Got a serial read/write error: " + str(e))
    return response


# Function for serial send/receive check
def check_for_device_on_serial_port(port_str, 
                                    baudrate = 9600, 
                                    message_str = '', 
                                    response_test_function = None, 
                                    include_cr = True, 
                                    include_lf = True,
                                    wait_time = 0.010):
    device_dict = None
    try:
        # Try and open serial port
        serial_port = serial.Serial(port_str,baudrate,timeout = 1)
    except Exception as e:
        logger.log_info("Unable to open serial port " + path_str + " with baudrate: " + str(baudrate) + " " + str(e))
        return device_dict
    # Add carriage return and line feed if needed
    response = send_serial_message(serial_port, 
                            message_str = message_str, 
                            include_cr = include_cr, 
                            include_lf = include_lf, 
                            wait_time = wait_time)
    if response is not None:

        
        if response_test_function is not None:
            try:
                valid = response_test_function(message_str,response)
            except:
                pass
        else:
            valid = (response == message_str)
        if response == response_str:
            device_dict = get_serial_device_dict(serial_port)
    # Clean up the serial port
    serial_port.close()
    return device_dict


# Function for tserial send/receive check for list of ports, addrs, baud_rates 
def check_for_device_on_serial_ports(
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
    port_str_list = serial.tools.list_port.comports()
    for port_str in port_str_list
        for baudrate in buadrate_list:
            for addr_str in message_addr_str_list:
                message_str = message_start_str + addr_str + message_stop_str

                if verbose == True:
                    msg = ("Checking for serial device " + port_str + " baudrate: " + str(baudrate) + \
                    " message: " + str(message_str))
                    print(msg)
                    logger.log_info(msg)
                device_dict = check_for_device_on_serial_port(
                                                        port_str, 
                                                        baudrate = baudrate, 
                                                        message_str = message_str,
                                                        response_test_function = response_test_function, 
                                                        include_cr = include_cr, 
                                                        include_lf = include_lf)
                if device_dict is not None:
                    device_dict['addr_str'] = addr_str
                    if verbose == True:
                        msg = ("Found Device: " + str(device_dict))
                        print(msg)
                        logger.log_info(msg)
                    break
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
                    start_ind = letters.index(start_addr)
                    stop_ind = letters.index(stop_addr)
                    addr_list = list(letters[start_ind:stop_ind + 1])
                except:
                    pass
            if start_str.isupper() and stop_str.isupper():
                try:
                    letters = nepi_utils.get_uppercase_letters()
                    start_ind = letters.index(start_addr)
                    stop_ind = letters.index(stop_addr)
                    addr_list = list(letters[start_ind:stop_ind + 1])
                except:
                    pass

    # Adjust for length if needed
    addr_list_adj = []
    for addr in addr_list:
        adj_addr = update_msg_to_length(addr, length = length, length_prefix = length_prefix, length_suffix = length_suffix)
        addr_list_adj.append()
    return addr_list_adj

        


    
def update_msg_to_length(message, length = None, length_prefix = '', length_suffix = ''):
    msg_str = message
    if length == None or lenght == 0 or len(message) >= length or (length_prefix == '' and \
                    length_suffix == '') or or (length_prefix != '' and length_suffix != ''):
        return msg_str
    for i in range(length):
        data_str = length_prefix + data_str + length_suffix

  
