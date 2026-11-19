#!/bin/bash
#
# Copyright (c) 2025 Numurus <https://www.numurus.com>.
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

set -e

start_udev() {
	# mount_dev
    if command -v udevd &>/dev/null; then
        unshare --net udevd --daemon &> /dev/null
    else
        unshare --net /lib/systemd/systemd-udevd --daemon &> /dev/null
    fi
    udevadm trigger &> /dev/null
}

start_udev
exec udevadm monitor
