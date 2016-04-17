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

for dir in `ls -d */`; do
    for i in 4 8 10 16 32; do
        echo $dir
        ./plot.sh $dir $i &
    done
done
