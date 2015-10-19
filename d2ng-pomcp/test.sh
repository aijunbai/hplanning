#!/bin/bash

sh ./clear.sh
make

./run.sh -p redundant_object_0 -m -1 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 
./run.sh -p redundant_object_0 -m 1 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 
./run.sh -p redundant_object_1 -m -1 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 &
./run.sh -p redundant_object_1 -m 1 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 
./run.sh -p redundant_object_1 -m 2 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 
./run.sh -p redundant_object_1 -m 4 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 
./run.sh -p redundant_object_1 -m 8 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 
./run.sh -p redundant_object_1 -m 16 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 
./run.sh -p redundant_object_1 -m 32 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 1000 -L 0 -H 15 & 

