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

import tkinter as tk
from tkinter import filedialog
import os
import glob
import fileinput
import statistics
import math

# Get file path
root = tk.Tk()
root.withdraw()
file = filedialog.askopenfilename()
pathname=os.path.dirname(file)
print('Folder Name')
print( pathname)

# Get bolog filename list
filelist=os.listdir(pathname)
for i, val in enumerate(filelist): 
    oldname=filelist[i]
    indx = oldname.rindex('.')
    ext=oldname[(indx+1):]
    name=oldname[:(indx)]
    if ext == "avi":
        newname= name + '_SHORT' + '.' + ext    
        os.rename(pathname + '/' + oldname,pathname + '/' + newname)
        

