from nepi_sdk import nepi_test

def response_test_function(message_str,response_str):
    valid = False
    if response_str == message_str:
        valid = True

    return valid

if __name__ == '__main__':

start_str = '' 
addr_str_start = 'A' 
addr_str_stop = 'B' 
addr_length = None
addr_length_prefix = '' 
addr_length_suffix = ''
stop_str = ''
buadrate_list = STANDARD_BUAD_RATES
include_cr = True 
include_lf = True
wait_time = 0.010
verbose = True


nepi_test.test_serial_device(start_str = start_str, 
                    addr_str_start = addr_str_start, 
                    addr_str_stop = addr_str_stop, 
                    addr_length = addr_length,
                    addr_length_prefix = addr_length_prefix, 
                    addr_length_suffix = addr_length_suffix,
                    stop_str = stop_str,
                    response_test_function = response_test_function, # Will use echo of command if None
                    buadrate_list = buadrate_list,
                    include_cr = include_cr, 
                    include_lf = include_lf,
                    wait_time = wait_time,
                    verbose = verbose)
