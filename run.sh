#!/bin/bash

set -o nounset

PROBLEM="rocksample"
PROBLEM="redundant_object_0"
PROBLEM="redundant_object_1"
PROBLEM="continousrooms_1"
PROBLEM="continousrooms_0"
PROBLEM="rooms_0"
PROBLEM="rooms_1"

MAP="data/10_rooms.map"
MAP="data/4_rooms.map"
MAP="data/16_rooms.map"
MAP="data/8_rooms.map"
MAP="data/32_rooms.map"
SIZE=7
NUM=8
RUNS=1
VERBOSE=2
MINPOWER2=11
MAXPOWER2=11
SEEDING=0
TIMEOUT=3600
TIMEOUTPERACTION=-1

HPLANNING=1
ACTIONABSTRACTION=1
POLLING=1
SMARTROLLOUT=1
STACK=0
LOCALREWARD=0
BRANCHINGFACTOR=1

FAKE="false"

OUTPUT="output-$$.txt"
LOG="log-$$.txt"

make -j4

while getopts "p:s:n:R:v:L:H:S:N:t:a:m:z:P:l:b:h:A:k:F" OPTION; do
    case $OPTION in
        p) PROBLEM=$OPTARG ;;
        s) SIZE=$OPTARG ;;
        n) NUM=$OPTARG ;;
        m) MAP=$OPTARG ;;
        R) RUNS=$OPTARG ;;
        v) VERBOSE=$OPTARG ;;
        L) MINPOWER2=$OPTARG ;;
        H) MAXPOWER2=$OPTARG ;;
        S) SEEDING=$OPTARG ;;
        N) MINPOWER2=$OPTARG; MAXPOWER2=$OPTARG ;;
        t) TIMEOUT=`expr $OPTARG \* 3600` ;;
        a) TIMEOUTPERACTION=$OPTARG ;;
        P) POLLING=$OPTARG ;;
        z) STACK=$OPTARG ;;
        l) LOCALREWARD=$OPTARG ;;
        b) BRANCHINGFACTOR=$OPTARG ;;
        h) HPLANNING=$OPTARG ;;
        A) ACTIONABSTRACTION=$OPTARG ;;
        k) SMARTROLLOUT=$OPTARG ;;
        F) FAKE="true" ;;
        *) exit ;;
    esac
done

run() {
    echo "$*" > $OUTPUT
    echo "$*" | tee $LOG
    if [ $FAKE == "false" ]; then
        exec $* 2>&1 | tee -a $LOG
    fi
}

run ./hplanning --outputfile $OUTPUT \
            --problem $PROBLEM \
            --size $SIZE \
            --number $NUM \
            --map $MAP \
            --verbose $VERBOSE \
            --mindoubles $MINPOWER2 \
            --maxdoubles $MAXPOWER2 \
            --runs $RUNS \
            --seeding $SEEDING \
            --timeout $TIMEOUT \
            --timeoutperaction $TIMEOUTPERACTION \
            --polling $POLLING \
            --stack $STACK \
            --localreward $LOCALREWARD \
            --branchingfactor $BRANCHINGFACTOR \
            --hplanning $HPLANNING \
            --actionabstraction $ACTIONABSTRACTION \
            --rolloutknowledge $SMARTROLLOUT \
