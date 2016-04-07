#!/bin/bash

cp hplanning_debug.pro hplanning.pro
./clear.sh
./run.sh -F

rm -f .gdbinit
while read LINE; do
    if [[ ${LINE:0:1} != "#" ]]; then
        echo "b $LINE" >> .gdbinit
    fi
done <breakpoints

echo "r" "`head -n 1 log-*.txt | sed -e 's|./hplanning||g'`" >> .gdbinit
cgdb hplanning
cp hplanning_release.pro hplanning.pro
reset
