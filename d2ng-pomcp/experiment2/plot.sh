#!/bin/bash

set -e
set -o nounset                              # Treat unset variables as an error

#declare -a X_ARRAY=(1 8)
#declare -a Y_ARRAY=(3 5 9 11)
declare -a X_ARRAY=(8)
declare -a Y_ARRAY=(3)


cd $1 

TITLES=`cat output-*.txt | head -2 | tail -n 1 | sed -e 's/#//g' -e 's/ /-/g' -e 's/_/+/g'`
IFS=$'\t' read -a TITLE_ARRAY <<< "$TITLES"
PDF_DIR="pdf"

plot() {
    local x=$1
    local y=$2

    local XLABEL="${TITLE_ARRAY[`expr $x - 1`]}"
    local YLABEL="${TITLE_ARRAY[`expr $y - 1`]}"

    local TEX="${XLABEL}_${YLABEL}.tex"
    local PLT="plot_${XLABEL}_${YLABEL}.sh"
    cp ../terminal_epslatex.gnu.txt "$PLT"
    echo "set output '${TEX}'" >>"$PLT"
    echo "set xlabel '$XLABEL' font 'DejaVuSansCondensed,18'" >>"$PLT"
    echo "set ylabel '$YLABEL' font 'DejaVuSansCondensed,18'" >>"$PLT"
    echo -n "plot" >>"$PLT"
    for i in output-*.txt; do
        local PROBLEM=`cat $i | grep -- --problem | awk '{print $5}' | sed -e 's/_//g'`
        local SIZE=`cat $i | grep -- --problem | awk '{print $7}'`
        local MAP=`cat $i | grep -- --problem | awk '{print $11}' | sed -e 's/_/ /g' -e 's|/| |g'| awk '{print $2}'`
        local POLLING=`cat $i | grep -- --problem | awk '{print $31}'`
        local STACK=`cat $i | grep -- --problem | awk '{print $33}'`
        local error=`expr ${y} + 1`

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

        if [ $MAP -eq 10 ]; then
            if grep -q "rooms" <<<$PROBLEM; then
                echo -n "'$i' u ${x}:${y}:${error} w yerrorlines t '${PROBLEM}:${MAP}${POLLING}${STACK}', " >>"$PLT"
            else
                echo -n "'$i' u ${x}:${y}:${error} w yerrorlines t '${PROBLEM}:${SIZE}${POLLING}${STACK}', " >>"$PLT"
            fi
        fi
    done

    chmod +x "$PLT"
    ./"$PLT"
    pdflatex "${TEX}"
    mv "${TEX//tex/pdf}" $PDF_DIR
    evince $PDF_DIR/"${TEX//tex/pdf}" &
}


rm -fr $PDF_DIR
mkdir -p $PDF_DIR

for x in "${X_ARRAY[@]}"; do
    for y in "${Y_ARRAY[@]}"; do
        plot $x $y &
    done
done

wait

