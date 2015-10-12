#!/bin/bash
# (C) Joydeep Biswas, 2010
# A simple Bash script to profile a command and 
# display the results on termination.
# You'll need to install the following packages: valgrind, kcachegrind

if [ ! -d "profile_results" ]; then
    mkdir profile_results
fi
echo Profiling \"$@\"
valgrind --tool=callgrind --dump-instr=yes --trace-jump=yes --callgrind-out-file="profile_results/callgrind.out.%p" $@
kcachegrind `ls -t1 profile_results/callgrind.out.*|head -n 1`

