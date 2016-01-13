#!/bin/bash

cp d2ng-pomcp_debug.pro d2ng-pomcp.pro
./clear.sh
./run.sh -F
valgrind -v --leak-check=full `head -n 1 log-*.txt` 2>&1 | tee valgrind.txt
cp d2ng-pomcp_release.pro d2ng-pomcp.pro
