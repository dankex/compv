#!/bin/bash
mkdir -p data
rm -f data/*.jpg
files=`ls data/*.p*m`
for f in ${files}
do
    newname=`echo $f|sed 's/\.ppm$//g'`
    convert ${f} ${newname}.jpg
    rm -f ${f}
done
