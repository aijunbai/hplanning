#!/bin/bash - 
#===============================================================================
#
#          FILE: build.sh
# 
#         USAGE: ./build.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 10/26/2015 16:16
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

DIR=`pwd`
cd ../../
make clean
make
cp hplanning $DIR
cp run.sh $DIR
cp -r data $DIR
cd $DIR

