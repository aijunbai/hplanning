#!/bin/bash

sh ./clear.sh
make

#./run.sh -p rooms_0 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 100 -L 0 -H 10 &
#./run.sh -p rooms_1 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 100 -L 0 -H 10 &

./run.sh -p redundant_object_0 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 100 -L 0 -H 10 &
./run.sh -p redundant_object_1 -h 12 -v 0 -S 1 -P 0 -u 0 -t 1 -r 2 -R 100 -L 0 -H 10 &

wait
