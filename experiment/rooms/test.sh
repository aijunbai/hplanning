#!/bin/bash

for i in 4 8; do
    ./run.sh -p rooms_0 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 0 -z 0 -k 0 -P 1 -M 0 & # UCT
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 0 -z 0 -k 0 -P 1 -M 1 & # abstract UCT
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 0 -z 0 -k 0 -P 1 -M 0 & # POMCP
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 1 -z 0 -k 0 -P 1 -M 0 & # H-POMCTS
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 1 -z 0 -k 1 -P 1 -M 0 & # smart H-POMCTS
done

wait 

for i in 16 32; do
    ./run.sh -p rooms_0 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 0 -z 0 -k 0 -P 1 -M 0 & # UCT
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 0 -z 0 -k 0 -P 1 -M 1 & # abstract UCT
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 0 -z 0 -k 0 -P 1 -M 0 & # POMCP
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 1 -z 0 -k 0 -P 1 -M 0 & # H-POMCTS
    ./run.sh -p rooms_1 -m data/${i}_rooms.map -S 1 -t 12 -v 0 -R 300 -L 0 -H 20 -h 1 -A 1 -z 0 -k 1 -P 1 -M 0 & # smart H-POMCTS
done

