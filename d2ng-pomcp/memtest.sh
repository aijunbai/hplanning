#!/bin/bash

./clear.sh
make
./run.sh
valgrind -v --leak-check=full `head -n 1 log-*.txt` 2>&1 | tee valgrind.txt

