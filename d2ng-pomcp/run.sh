#!/bin/bash

PROBLEM="rocksample"
PROBLEM="continousrooms_1_0"
PROBLEM="continousrooms_0"
PROBLEM="continousrooms_1_1"
PROBLEM="redundant_object_0"
PROBLEM="redundant_object_1"
PROBLEM="rooms_1_0"
PROBLEM="rooms_0"
PROBLEM="rooms_1_1"
SIZE=7
NUM=8
LEVELT=1
LEVELR=2
RUNS=1
VERBOSE=2
USEPFILTER=0
REUSETREE=0
MINPOWER2=10
MAXPOWER2=10
SEEDING=0
TIMEOUT=3600
THOMPSONSAMPLING=0
TIMEOUTPERACTION=-1
CONVERGED=0.1
CACHERATE=0.9

OUTPUT="output-$$.txt"
LOG="log-$$.txt"

while getopts "p:s:n:t:r:R:v:u:L:H:S:P:N:h:a:T:C:c:" OPTION; do
    case $OPTION in
        p) PROBLEM=$OPTARG ;;
        s) SIZE=$OPTARG ;;
        n) NUM=$OPTARG ;;
        t) LEVELT=$OPTARG ;;
        r) LEVELR=$OPTARG ;;
        R) RUNS=$OPTARG ;;
        v) VERBOSE=$OPTARG ;;
        u) REUSETREE=$OPTARG ;;
        L) MINPOWER2=$OPTARG ;;
        H) MAXPOWER2=$OPTARG ;;
        S) SEEDING=$OPTARG ;;
        P) USEPFILTER=$OPTARG ;;
        N) MINPOWER2=$OPTARG; MAXPOWER2=$OPTARG ;;
        h) TIMEOUT=`expr $OPTARG \* 3600` ;;
        T) THOMPSONSAMPLING=$OPTARG ;;
        a) TIMEOUTPERACTION=$OPTARG ;;
        C) CONVERGED=$OPTARG ;;
        c) CACHERATE=$OPTARG ;;
    esac
done

run() {
    echo "$*" > $OUTPUT
    echo "$*" | tee $LOG
    exec $* 2>&1 | tee -a $LOG
}

run ./d2ng-pomcp --outputfile $OUTPUT \
            --problem $PROBLEM \
            --size $SIZE \
            --number $NUM \
            --verbose $VERBOSE \
            --reusetree $REUSETREE \
            --useparticlefilter $USEPFILTER \
            --treeknowledge $LEVELT \
            --rolloutknowledge $LEVELR \
            --mindoubles $MINPOWER2 \
            --maxdoubles $MAXPOWER2 \
            --runs $RUNS \
            --seeding $SEEDING \
            --timeout $TIMEOUT \
            --thompsonsampling $THOMPSONSAMPLING \
            --timeoutperaction $TIMEOUTPERACTION \
            --converged $CONVERGED \
            --cacherate $CACHERATE

