#!/bin/bash
# (C) Joydeep Biswas, 2010
# A simple Bash script to profile a command and 
# display the results on termination.
# You'll need to install the following packages: valgrind, kcachegrind

./clear.sh
./run.sh -F

if [ ! -d "profile_results" ]; then
    mkdir profile_results
fi

echo Profiling \"$@\"
valgrind --tool=callgrind --dump-instr=yes --trace-jump=yes --callgrind-out-file="profile_results/callgrind.out.%p" `head -n 1 log-*.txt` 2>&1 | tee callgrind.txt
kcachegrind `ls -t1 profile_results/callgrind.out.*|head -n 1`

