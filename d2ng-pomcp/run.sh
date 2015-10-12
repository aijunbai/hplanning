#!/bin/bash

PROBLEM="redundant_object_1"
SIZE=10
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
TIMEOUTPERACTION=-1
MEMORYSIZE=-1

OUTPUT="output-$$.txt"
LOG="log-$$.txt"

while getopts "p:s:n:t:r:R:v:u:L:H:S:P:N:h:a:m:" OPTION; do
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
        h) TIMEOUT=`expr $OPTARG \* 3600`;;
        a) TIMEOUTPERACTION=$OPTARG ;;
        m) MEMORYSIZE=$OPTARG ;;
    esac
done

run() {
    echo "#$*" > $OUTPUT
    echo "#$*" | tee $LOG
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
            --timeoutperaction $TIMEOUTPERACTION \
            --memorysize $MEMORYSIZE

