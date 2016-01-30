#!/bin/bash

cp hplanning_debug.pro hplanning.pro
./clear.sh
./run.sh -F

cat breakpoints > .gdbinit
echo "r" "`head -n 1 log-*.txt | sed -e 's|./hplanning||g'`" >> .gdbinit
cgdb hplanning
cp hplanning_release.pro hplanning.pro
reset
