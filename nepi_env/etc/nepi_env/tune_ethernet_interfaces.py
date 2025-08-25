#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# This script is called as part of NEPI roslaunch at start-up to ensure that ethernet interfaces
# have tuning parameters set to optimize throughput for a wide collection of sensors. In testing
# discovered that these settings should be applied prior to roslaunch -- otherwise high-throughput
# topics (imagery, etc.) appear to be totally broken.

import os
import subprocess
import time

# Ethernet interface tuning - Primarily based on Genicam needs as described in
# FLIR Spinnaker SDK README_ARM
# TODO: Maybe these should be config. file params?
NEPI_ETH_MTU = '9216'
NEPI_UDP_RMEM_MIN = '12288'
NEPI_NETDEV_MAX_BACKLOG = '4096'
NEPI_MAX_DGRAM_QLEN = '1024'
NEPI_RMEM_MAX = '10485760'
NEPI_RMEM_DEFAULT = '10485760'
NEPI_RP_FILTER = '0' # Disabled -- improves Genicam discovery enumeration?

def tuneEthernetInterfaces():
    print(f'Tuning all ethernet interfaces for faster sensor throughput')

    # First, gather all ethernet interfaces and set the MTU
    eth_ifaces = [x for x in os.listdir('/sys/class/net') if x.startswith('eth')]
    for iface in eth_ifaces:
        # Better to skip any interfaces that already have the proper MTU because that avoids bringing that interface
        # down and then back up.
        current_settings = subprocess.run(['ip', 'link', 'show', 'dev', iface], capture_output=True).stdout.decode('utf-8')
        current_mtu = current_settings.split('mtu ')[1].split()[0]
        if current_mtu == NEPI_ETH_MTU:
            print(f'Skipping MTU for {iface} because it is already as desired')
            continue

        print(f'Updating MTU from {current_mtu} to {NEPI_ETH_MTU} for {iface}')
        try:
            # Must bring the interface down before adjusting MTU
            subprocess.run(['ip', 'link', 'set', 'dev', iface, 'down'])
            subprocess.run(['ip', 'link', 'set', 'dev', iface, 'mtu', str(NEPI_ETH_MTU)])
            subprocess.run(['ip', 'link', 'set', 'dev', iface, 'up'])
        except Exception as e:
            print(f'Failed to set MTU for interface {iface} ({e})')

    try:
        # Now set the various global params via sysctl
        subprocess.run(['sysctl', '-w', 'net.ipv4.udp_rmem_min=' + NEPI_UDP_RMEM_MIN])
        subprocess.run(['sysctl', '-w', 'net.core.netdev_max_backlog=' + NEPI_NETDEV_MAX_BACKLOG])
        subprocess.run(['sysctl', '-w', 'net.unix.max_dgram_qlen=' + NEPI_MAX_DGRAM_QLEN])
        subprocess.run(['sysctl', '-w', 'net.core.rmem_max=' + NEPI_RMEM_MAX])
        subprocess.run(['sysctl', '-w', 'net.core.rmem_default=' + NEPI_RMEM_DEFAULT])
        subprocess.run(['sysctl', '-w', 'net.ipv4.conf.all.rp_filter=' + NEPI_RP_FILTER])
        subprocess.run(['sysctl', '-w', 'net.ipv4.conf.default.rp_filter=' + NEPI_RP_FILTER])
    except Exception as e:
        print(f'Failed to set at least one network tuning parameter ({e})')

    #print(f'Sleeping long enough for changes to take effect')
    #time.sleep(5)

if __name__ == '__main__':
    tuneEthernetInterfaces() 