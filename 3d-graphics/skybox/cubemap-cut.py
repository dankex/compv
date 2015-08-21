#!/usr/bin/python

from __future__ import print_function
import sys
import os.path
from PIL import Image

if len(sys.argv) < 2:
    print("Usage: cubemap-cut.py <filename.jpg|png>")
    sys.exit(-1)

infile = sys.argv[1]
filename, file_extension = os.path.splitext(infile)

name_map = [ \
     ["", "", "posy", ""],
     ["posz", "negx", "negz", "posx"],
     ["", "", "negy", ""]]

try:
    im = Image.open(infile)
    print(infile, im.format, "%dx%d" % im.size, im.mode)

    width, height = im.size

    cube_size = width / 6

    print("%dx%d cube %d" % (width, height, cube_size))
    for row in range(3):
        for col in range(4):
            if name_map[row][col] != "":
                sx = cube_size * col
                sy = cube_size * row
                fn = name_map[row][col] + file_extension
                print("%s --> %s" % (str((sx, sy, sx + cube_size, sy + cube_size)), fn))
                im.crop((sx, sy, sx + cube_size, sy + cube_size)).save(fn) 

except IOError:
    pass


