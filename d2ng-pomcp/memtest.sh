#!/bin/bash

valgrind -v --leak-check=full ./d2ng-pomcp --outputfile output-valgrind.txt --problem rocksample --size 7 --number 8 --verbose 2 --reusetree 0 --useparticlefilter 0 --treeknowledge 1 --rolloutknowledge 2 --mindoubles 10 --maxdoubles 10 --runs 1 --seeding 0 --timeout 3600 --thompsonsampling 0 --timeoutperaction -1 --memorysize -1 2>&1 | tee log-valgrind.txt

