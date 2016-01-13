#!/bin/bash - 
#===============================================================================
#
#          FILE: debug.sh
# 
#         USAGE: ./debug.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 01/13/2016 13:29
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

cp d2ng-pomcp_release.pro d2ng-pomcp.pro
./clear.sh
./run.sh $*
