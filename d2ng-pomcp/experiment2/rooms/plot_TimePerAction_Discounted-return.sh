#!/usr/bin/gnuplot
#
# Demonstrates a simple usage of gnuplot.
#
# AUTHOR: Hagen Wierstorf 

reset

# epslatex
# NOTE: thes ize is given in inches. It can also be given in cm.
set terminal epslatex size 6.5,3.62 standalone color colortext 10

# Line styles
set border linewidth 2
set style line 1 lc rgb 'green' lt 1 lw 2 pt 9 ps 1.5
set style line 2 lc rgb 'cyan' lt 1 lw 2 pt 11 ps 1.5
set style line 3 lc rgb 'blue' lt 1 lw 2 pt 13 ps 1.5
set style line 4 lc rgb 'purple' lt 1 lw 2 pt 5 ps 1.5
set style line 5 lc rgb 'red' lt 1 lw 2 pt 7 ps 1.5

# Legend
set key left top Right
#set key right bottom Right
# Axes label
#set xlabel '$x$'
#set ylabel '$y$'
# Axis ranges
#set xrange[1:100000]
#set yrange[-1.5:1.5]
# Tics in LaTeX format
# Axis labels
set log x
set grid

#set output 'terminal_epslatex.tex'
#set xlabel "Number of Iterations" font "DejaVuSansCondensed,18"
#set ylabel "Avg. Discounted Return" font "DejaVuSansCondensed,18"
#plot './uct/output-28476.txt' u 1:5:6 w yerrorlines ls 3 t 'POMCP', './dng/output-2054.txt' u 1:5:6 w yerrorlines ls 5 t 'D$^2$NG-POMCP'

set output 'TimePerAction_Discounted-return.tex'
set xlabel 'TimePerAction' font 'DejaVuSansCondensed,18'
set ylabel 'Discounted-return' font 'DejaVuSansCondensed,18'
plot'output-7623.txt' u 8:5:6 w yerrorlines t 'P:rooms0;M:1;AS:0', 'output-7624.txt' u 8:5:6 w yerrorlines t 'P:rooms10;M:-1;AS:0', 'output-7625.txt' u 8:5:6 w yerrorlines t 'P:rooms10;M:1;AS:0', 'output-7626.txt' u 8:5:6 w yerrorlines t 'P:rooms11;M:-1;AS:0', 'output-7627.txt' u 8:5:6 w yerrorlines t 'P:rooms11;M:1;AS:0', 'output-7628.txt' u 8:5:6 w yerrorlines t 'P:rooms11;M:-1;AS:1', 'output-7629.txt' u 8:5:6 w yerrorlines t 'P:rooms11;M:1;AS:1', 