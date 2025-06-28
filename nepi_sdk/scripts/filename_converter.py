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
        

