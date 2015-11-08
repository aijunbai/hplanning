#!/bin/bash

./run.sh -p rooms_0 -m data/8_rooms.map -h 3 -v 0 -S 1 -R 300 -L 0 -H 18 & 
./run.sh -p rooms_1_0 -m data/8_rooms.map -h 3 -v 0 -S 1 -R 300 -L 0 -H 18 &
./run.sh -p rooms_1_1 -m data/8_rooms.map -P 1 -@ 0 -h 3 -v 0 -S 1 -R 300 -L 0 -H 18 &
./run.sh -p rooms_1_1 -m data/8_rooms.map -P 1 -@ 1 -h 3 -v 0 -S 1 -R 300 -L 0 -H 18 &

