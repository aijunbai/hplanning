#!/bin/bash

for i in 32; do
    ./run.sh -p continousrooms_0 -m data/${i}_rooms.map -S 1 -t 1 -v 0 -R 50 -L 0 -H 18 -h 1 -A 0 -z 0 -k 0 -P 1 & # UCT
    ./run.sh -p continousrooms_1 -m data/${i}_rooms.map -S 1 -t 1 -v 0 -R 50 -L 0 -H 18 -h 1 -A 0 -z 0 -k 0 -P 1 & # POMCP
    ./run.sh -p continousrooms_1 -m data/${i}_rooms.map -S 1 -t 1 -v 0 -R 50 -L 0 -H 18 -h 1 -A 1 -z 0 -k 0 -P 1 & # H-POMCTS
    ./run.sh -p continousrooms_1 -m data/${i}_rooms.map -S 1 -t 1 -v 0 -R 50 -L 0 -H 18 -h 1 -A 1 -z 0 -k 1 -P 1 & # smart H-POMCTS
done

