#!/bin/bash

set -o nounset

PROBLEM="rocksample"
PROBLEM="continousrooms_1_0"
PROBLEM="continousrooms_0"
PROBLEM="continousrooms_1_1"
PROBLEM="redundant_object_0"
PROBLEM="redundant_object_1"
PROBLEM="rooms_0"
PROBLEM="rooms_1_0"
PROBLEM="rooms_1_1"
MAP="data/8_rooms.map"
SIZE=7
NUM=8
RUNS=1
VERBOSE=2
MINPOWER2=10
MAXPOWER2=10
SEEDING=0
TIMEOUT=3600
TIMEOUTPERACTION=-1
POLLING=1
STACK=1
LOCALREWARD=0
#CONVERGED=1.0
#CACHERATE=0.9
FAKE="false"

OUTPUT="output-$$.txt"
LOG="log-$$.txt"

make -j4

while getopts "p:s:n:R:v:L:H:S:P:N:h:a:m:@:l:F" OPTION; do
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
        h) TIMEOUT=`expr $OPTARG \* 3600` ;;
        a) TIMEOUTPERACTION=$OPTARG ;;
#       C) CONVERGED=$OPTARG ;;
#       c) CACHERATE=$OPTARG ;;
        P) POLLING=$OPTARG ;;
        @) STACK=$OPTARG ;;
        l) LOCALREWARD=$OPTARG ;;
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

run ./d2ng-pomcp --outputfile $OUTPUT \
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
            --localreward $LOCALREWARD
#           --converged $CONVERGED \
#           --cacherate $CACHERATE \

