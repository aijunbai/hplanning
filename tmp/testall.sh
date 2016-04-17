#!/bin/bash - 
#===============================================================================
#
#          FILE: plot-all.sh
# 
#         USAGE: ./plot-all.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 10/23/2015 11:21
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

ulimit -c unlimited

for dir in `ls -d */`; do
    cd $dir
    sh ../clear.sh
    sh ../build.sh
    ./test.sh &
    sleep 10
    cd ..
done

wait
