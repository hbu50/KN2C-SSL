#!/bin/bash

while true; do

git pull
cd src
/opt/Qt5.2.1/5.2.1/gcc_64/bin/qmake kn2cssl.pro
make -j 8
sleep 15

done
