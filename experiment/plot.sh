#!/bin/bash

set -e
set -o nounset                              # Treat unset variables as an error

#declare -a X_ARRAY=(1 8)
#declare -a Y_ARRAY=(3 5 9 11)
declare -a X_ARRAY=(1 8)
declare -a Y_ARRAY=(5)

DIR=`basename $1`
ROOMS=$2
cd $DIR

TITLES=`head -2 output-*.txt | tail -n 1 | sed -e 's/#//g' -e 's/ /-/g' -e 's/_/+/g'`
IFS=$'\t' read -a TITLE_ARRAY <<< "$TITLES"
PDF_DIR="pdf"

plot() {
    local x=$1
    local y=$2

    echo $x $y $DIR $ROOMS $TITLES
    local XLABEL="${TITLE_ARRAY[`expr $x - 1`]}"
    local YLABEL="${TITLE_ARRAY[`expr $y - 1`]}"
    echo $XLABEL $YLABEL

    local TEX="${DIR}_${ROOMS}_${XLABEL}_${YLABEL}.tex"
    local PLT="plot_${TEX//tex/sh}"
    cp ../terminal_epslatex.gnu.txt "$PLT"
    echo "set output '${TEX}'" >>"$PLT"

    if [ $XLABEL = "TimePerAction" ]; then
        XLABEL="Time Per Action (s)"
    fi

    echo "set xlabel '$XLABEL' font 'DejaVuSansCondensed,18'" >>"$PLT"
    echo "set ylabel '$YLABEL' font 'DejaVuSansCondensed,18'" >>"$PLT"
    echo -n "plot" >>"$PLT"

    for i in output-*.txt; do
        echo $i
        local PROBLEM=`head $i | grep -- --problem | awk '{print $5}' | sed -e 's/_//g'`
        local MAP=`head $i | grep -- --problem | awk '{print $11}' | sed -e 's/_/ /g' -e 's|/| |g'| awk '{print $2}'`
        local HPLANNING=`head $i | grep -- --problem | awk '{print $35}'`
        local ACTIONABSTRACTION=`head $i | grep -- --problem | awk '{print $37}'`
        local error=`expr ${y} + 1`

        if [ $ACTIONABSTRACTION -eq 1 ]; then
            ACTIONABSTRACTION="Hierarchical-"
        else
            ACTIONABSTRACTION=""
        fi

        if [ $HPLANNING -eq 1 ]; then
            HPLANNING="H-"
        else
            HPLANNING=""
        fi

        if grep -q "rooms1" <<<$PROBLEM; then
            local POLLING=`head $i | grep -- --problem | awk '{print $27}'`
            local STACK=`head $i | grep -- --problem | awk '{print $29}'`
            local LOCALREWARD=`head $i | grep -- --problem | awk '{print $31}'`
            local KNOWLEDGE=`head $i | grep -- --problem | awk '{print $39}'`
            local MEMORYLESS=`head $i | grep -- --problem | awk '{print $41}'`

            if [ $LOCALREWARD -eq 1 ]; then
                LOCALREWARD=":localreward"
            else
                LOCALREWARD=""
            fi

            if [ $POLLING -eq 1 ]; then
                POLLING=":polling"
            else
                POLLING=""
            fi

            if [ $STACK -eq 1 ]; then
                STACK=":stack"
            else
                STACK=""
            fi

            if [ $KNOWLEDGE -eq 1 ]; then
                KNOWLEDGE=":smart"
            else
                KNOWLEDGE=""
            fi

            if [ $MEMORYLESS -eq 1 ]; then
                MEMORYLESS=":memoryless"
            else
                MEMORYLESS=""
            fi
        fi

        local TITLE=""
        if grep -q "rooms0" <<<$PROBLEM; then
            if [ -z $HPLANNING ]; then
                TITLE="UCT"
            else
                TITLE="MCTS"
            fi
        elif grep -q "rooms1" <<<$PROBLEM; then
            if [ -z $HPLANNING ]; then
                TITLE="POMCP"
            else
                if [ ! -z $ACTIONABSTRACTION ]; then
                    TITLE="MCTS\$|_{\phi,\mathcal{O}}\$${STACK}${LOCALREWARD}${KNOWLEDGE}${POLLING}"
                else
                    TITLE="MCTS\$|_\phi\$${MEMORYLESS}"
                fi
            fi
        fi

        if [ $MAP -eq $ROOMS ]; then
            echo $TITLE
            echo -n "'$i' u ${x}:${y}:${error} w yerrorlines t '$TITLE', " >>"$PLT"
        fi
    done

    chmod +x "$PLT"
    ./"$PLT"
    pdflatex "${TEX}"
    mv "${TEX//tex/pdf}" "$PDF_DIR"
    evince $PDF_DIR/"${TEX//tex/pdf}" &
}


mkdir -p $PDF_DIR

for x in "${X_ARRAY[@]}"; do
    for y in "${Y_ARRAY[@]}"; do
        plot $x $y &
    done
done

wait

