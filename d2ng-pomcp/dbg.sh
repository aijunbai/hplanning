#!/bin/bash

cp d2ng-pomcp_debug.pro d2ng-pomcp.pro
./clear.sh
./run.sh -F

cat breakpoints > .gdbinit
echo "r" "`head -n 1 log-*.txt | sed -e 's|./d2ng-pomcp||g'`" >> .gdbinit
cgdb d2ng-pomcp
cp d2ng-pomcp_release.pro d2ng-pomcp.pro
