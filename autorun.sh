#!/bin/bash

pkill -f "kn2cssl"
pause

cd bin

while true; do

pkill -f "kn2cssl"
./kn2cssl nogui &
sleep 30
#sleep 1

done
