#!/bin/bash

valgrind -v --leak-check=full ./d2ng-pomcp --outputfile output-valgrind.txt --problem redundant_object_1 --size 10 --number 8 --verbose 2 --reusetree 0 --useparticlefilter 0 --treeknowledge  1 --rolloutknowledge 2 --mindoubles 10 --maxdoubles 10 --runs 1 --seeding 0 --timeout 3600 --timeoutperaction -1 --memorysize 1

