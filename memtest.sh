#!/bin/bash

cp hplanning_debug.pro hplanning.pro
./clear.sh
./run.sh -F
valgrind -v --leak-check=full `head -n 1 log-*.txt` 2>&1 | tee valgrind.txt
cp hplanning_release.pro hplanning.pro
