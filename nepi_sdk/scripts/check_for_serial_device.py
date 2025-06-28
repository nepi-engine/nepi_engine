from nepi_sdk import nepi_serial

STANDARD_BUAD_RATES = [110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

def check_serial_ports_by_message(
                    port_str_list = None,
                    buadrate_list = nepi_serial.STANDARD_BUAD_RATES,
                    start_str = '', 
                    addr_str_start = '#', 
                    addr_str_list = [''],
                    stop_str = '',
                    response_test_function = None, # Will use echo of command if None
                    include_cr = True, 
                    include_lf = True,
                    wait_time = 0.010,
                    verbose = True):
    device_dict = None
    if response_test_function is not None:
        device_dict = nepi_serial.check_serial_ports_by_message(
                        port_str_list = port_str_list,
                        buadrate_list = buadrate_list,
                        message_start_str = start_str, 
                        message_addr_str_list = addr_str_list, 
                        message_stop_str = stop_str,
                        response_test_function = response_test_function, # Will use echo of command if None
                        include_cr = include_cr, 
                        include_lf = include_lf,
                        wait_time = wait_time,
                        verbose = verbose)
    if device_dict is None:
      print("Did not find Device by message")
    return device_dict

def check_serial_ports_by_product_id(
                    port_str_list = None,
                    buadrate_list = nepi_serial.STANDARD_BUAD_RATES,
                    product_id = 0,
                    include_cr = True, 
                    include_lf = True,
                    wait_time = 0.010,
                    verbose = True):
    device_dict = None
    if product_id is not None:
        device_dict = nepi_serial.check_serial_ports_by_product_id(
                        port_str_list = port_str_list,
                        buadrate_list = buadrate_list,
                        product_id = product_id,
                        include_cr = include_cr, 
                        include_lf = include_lf,
                        wait_time = wait_time,
                        verbose = verbose)
        if device_dict is None:
            print("Did not find Device by message")
    return device_dict
    


def response_test_function(message_str,response_str):
    valid = False
    #if response_str == message_str:
    #    valid = True
    if response_str[0:5] == message_str[0:5]:
        valid = True
    return valid

if __name__ == '__main__':
    port_str_list = None # If None, checks all available ports
    start_str = '#' 
    addr_str_start = 'A' 
    addr_str_stop = 'B' 
    addr_length = None
    addr_length_prefix = '' 
    addr_length_suffix = ''
    stop_str = 'MRA0000R'
    buadrate_list = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    test_function = response_test_function
    product_id = None
    include_cr = True 
    include_lf = True
    wait_time = 0.010
    verbose = True

    addr_str_list = nepi_serial.create_serial_port_addrs_list(
                    start_str=addr_str_start, 
                    stop_str=addr_str_stop, 
                    length = addr_length, 
                    length_prefix = addr_length_prefix, 
                    length_suffix = addr_length_suffix)

    if test_function is not None:
        check_serial_ports_by_message(
                                port_str_list = port_str_list,
                                start_str = start_str, 
                                addr_str_start = addr_str_start, 
                                addr_str_list = addr_str_list,
                                stop_str = stop_str,
                                response_test_function = response_test_function, # Will use echo of command if None
                                buadrate_list = buadrate_list,
                                include_cr = include_cr, 
                                include_lf = include_lf,
                                wait_time = wait_time,
                                verbose = verbose)
    if product_id is not None:
        check_serial_ports_by_product_id(
                                port_str_list = port_str_list,
                                product_id = product_id,
                                response_test_function = response_test_function, # Will use echo of command if None
                                buadrate_list = buadrate_list,
                                include_cr = include_cr, 
                                include_lf = include_lf,
                                wait_time = wait_time,
                                verbose = verbose)
