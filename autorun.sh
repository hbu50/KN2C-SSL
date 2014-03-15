#!/bin/bash

while true; do

./autocompile.sh &
sleep 30
pkill -f "kn2cssl"
sleep 1

done


