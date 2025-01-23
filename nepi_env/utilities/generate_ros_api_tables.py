#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# This script is intended to be run from a system with the proper ROS setup.bash file sourced so that all message types are understood.
# Generally, run it from the NEPI device itself

import subprocess
import sys

NEPI_TOPIC_OUTFILE = "nepi_topics.csv"
NON_NEPI_TOPIC_OUTFILE = "non_nepi_topics.csv"
NEPI_SERVICE_OUTFILE = "nepi_services.csv"
NON_NEPI_SERVICE_OUTFILE = "non_nepi_services.csv"

def documentTopics(nepi_topic_out_filename = NEPI_TOPIC_OUTFILE, non_nepi_topic_out_filename = NON_NEPI_TOPIC_OUTFILE):
    # First, gather all topics
    try:
        topic_list = subprocess.check_output(["rostopic", "list"]).splitlines()
    except Exception as e:
        print("Failed to gather topic list... ", str(e))
        print("... is ROS setup.bash sourced? Is ROS running?")
        sys.exit(1)

    topic_count = len(topic_list)
    print("Identified " + str(topic_count) + " topics")
    print("Parsing and writing output to " + str(nepi_topic_out_filename) + " [NEPI TOPICS]" + "and " + str(non_nepi_topic_out_filename + " [NON-NEPI TOPICS]"))
    
    TOPIC_CSV_HEADER_STRING = "TOPIC,TYPE,PUBLISHERS,SUBSCRIBERS\n"

    # NEPI Topic File
    nepi_topic_file = open(nepi_topic_out_filename, 'w')
    nepi_topic_file.write(TOPIC_CSV_HEADER_STRING)

    # Non-NEPI Topic FIle
    non_nepi_topic_file = open(non_nepi_topic_out_filename, 'w')
    non_nepi_topic_file.write(TOPIC_CSV_HEADER_STRING)

    # Capture the NEPI base namespace -- assumes this is a NEPI topic, so a bit fragile
    longest_topic = max(topic_list, key=len)
    topic_parsed_list = longest_topic.split('/')
    nepi_id = topic_parsed_list[1]
    device_id = topic_parsed_list[2]
    if (nepi_id != "nepi"):
        print("Warning: Unexpected nepi_id: " + nepi_id)

    nepi_namespace_base = '/' + nepi_id + '/' + device_id + '/'
    print("NEPI Namespace Base: " + nepi_namespace_base)

    for n, topic in enumerate(topic_list):
        print("Processing topic " + str(n+1) + '/' + str(topic_count) + ': ' + topic)
                
        try:
            topic_info = subprocess.check_output(["rostopic", "info", topic])
        except Exception as e:
            print("Failed to gather topic_info for ", topic, " ", str(e), "... skipping this entry")
            continue

        topic_type = None
        topic_publishers = []
        topic_subscribers = []

        topic_info_lines = topic_info.splitlines()
        #print("Debug: Lines = " + str(topic_info_lines))
        for i, line in enumerate(topic_info_lines):
            #print("Debug: Line = ", line)
            if line.startswith("Type:"):
                topic_type = line.split(":")[1]
            elif line.startswith("Publishers:"):
                for publisher_line in topic_info_lines[i+1:]:
                    if publisher_line.startswith(" *"):
                        #print("Debug: Publisher line = " + publisher_line)
                        publisher = publisher_line.split()[1]
                        topic_publishers.append(publisher)
                    else:
                        break
            elif line.startswith("Subscribers:"):
                for subscriber_line in topic_info_lines[i+1:]:
                    if subscriber_line.startswith(" *"):
                        subscriber = subscriber_line.split()[1]
                        topic_subscribers.append(subscriber)
                    else:
                        break
        
        if topic.startswith(nepi_namespace_base):
            topic_shortname = topic[len(nepi_namespace_base):]
            topic_csv_string = topic_shortname.strip()
            f = nepi_topic_file
        else:
            topic_csv_string = topic.strip()
            f = non_nepi_topic_file
        
        topic_csv_string += ',' + topic_type.strip() + ','
        for i, pub in enumerate(topic_publishers):
            if pub.startswith(nepi_namespace_base):
                pub = pub[len(nepi_namespace_base):]
            if (i > 0):
                topic_csv_string += ' | '
            topic_csv_string += pub.strip()
            
        topic_csv_string += ','
        for i, sub in enumerate(topic_subscribers):
            if sub.startswith(nepi_namespace_base):
                sub = sub[len(nepi_namespace_base):]
            if (i > 0):
                topic_csv_string += ' | ' 
            topic_csv_string += sub.strip()

        f.write(topic_csv_string + "\n")
        #print(topic,topic_type,topic_publishers,topic_subscribers)

    nepi_topic_file.close()
    non_nepi_topic_file.close()

def documentServices(nepi_service_out_filename = NEPI_SERVICE_OUTFILE, non_nepi_service_out_filename = NON_NEPI_SERVICE_OUTFILE, skip_loggers=True): 
    # First, gather all services
    try:
        service_list = subprocess.check_output(["rosservice", "list"]).splitlines()
    except Exception as e:
        print("Failed to gather service list... ", str(e))
        print("... is ROS setup.bash sourced? Is ROS running?")
        sys.exit(1)

    service_count = len(service_list)
    print("Identified " + str(service_count) + " services")
    print("Parsing and writing output to " + str(nepi_service_out_filename) + " [NEPI TOPICS]" + "and " + str(non_nepi_service_out_filename + " [NON-NEPI TOPICS]"))
    
    SERVICE_CSV_HEADER_STRING = "SERVICE,TYPE,NODE\n"

    # NEPI Service File
    nepi_service_file = open(nepi_service_out_filename, 'w')
    nepi_service_file.write(SERVICE_CSV_HEADER_STRING)

    # Non-NEPI Service FIle
    non_nepi_service_file = open(non_nepi_service_out_filename, 'w')
    non_nepi_service_file.write(SERVICE_CSV_HEADER_STRING)

    # Capture the NEPI base namespace -- assumes this is a NEPI service, so a bit fragile
    longest_service = max(service_list, key=len)
    service_parsed_list = longest_service.split('/')
    nepi_id = service_parsed_list[1]
    device_id = service_parsed_list[2]
    if (nepi_id != "nepi"):
        print("Warning: Unexpected nepi_id: " + nepi_id)

    nepi_namespace_base = '/' + nepi_id + '/' + device_id + '/'
    print("NEPI Namespace Base: " + nepi_namespace_base)

    for n, service in enumerate(service_list):
        print("Processing service " + str(n+1) + '/' + str(service_count) + ': ' + service)

        if (skip_loggers is True) and (service.endswith('get_loggers') or service.endswith('set_logger_level')):
            continue
               
        try:
            service_info = subprocess.check_output(["rosservice", "info", service])
        except Exception as e:
            print("Failed to gather service_info for ", service, " ", str(e), "... skipping this entry")
            continue

        service_type = None
        service_node = None

        service_info_lines = service_info.splitlines()
        for line in service_info_lines:
            #print("Debug: Line = ", line)
            if line.startswith("Type:"):
                service_type = line.split(":")[1]
            elif line.startswith("Node:"):
                service_node = line.split(":")[1]

        if service.startswith(nepi_namespace_base):
            service_shortname = service[len(nepi_namespace_base):]
            service_csv_string = service_shortname.strip()
            f = nepi_service_file
        else:
            service_csv_string = service.strip()
            f = non_nepi_service_file
        
        service_csv_string += ',' + service_type.strip() + ','

        if service_node.startswith(nepi_namespace_base):
            service_node = service_node[len(nepi_namespace_base):]
        service_csv_string += service_node.strip()

        f.write(service_csv_string + "\n")
        #print(service,service_type,service_publishers,service_subscribers)

    nepi_service_file.close()
    non_nepi_service_file.close()

if __name__ == '__main__':
    documentTopics()
    documentServices()            