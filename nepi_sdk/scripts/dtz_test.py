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

import time
import datetime
import pytz

tzd = 'America/New_York'

tzo = pytz.timezone(tzd)
print(tzo)

dt_now = datetime.datetime.now(tzo).strftime("%m/%d/%Y,%H:%M:%S")
print(dt_now)

time_sec = time.time()
dt = datetime.datetime.fromtimestamp(time_sec,tzo)
print(dt)
