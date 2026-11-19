#!/bin/bash
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


# This script deletes all nepi folders/files in the nepi system
cd /opt/nepi/nepi_engine
sudo find . -type d -name 'nepi_*' -exec rm -rf {} +
sudo chown -R nepi:nepi ./*
# Run Nepi deploy and build complete scripts to rebuild system

