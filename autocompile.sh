#!/bin/bash

git pull
cd src
qmake kn2cssl.pro
make -j 8

