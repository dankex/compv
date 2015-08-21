#!/usr/bin/python

from __future__ import print_function
import sys
from PIL import Image

infile = sys.argv[1]

name_map = \
    ["", "", "top", "",
     "back", "left", "front", "right",
     "", "", "bottom", ""]

print(infile)

try:
    im = Image.open(infile)
    print(infile, im.format, "%dx%d" % im.size, im.mode)
    
    print(name_map);

except IOError:
    pass


