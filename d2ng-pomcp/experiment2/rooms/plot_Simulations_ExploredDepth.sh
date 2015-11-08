#!/usr/bin/env gnuplot
#
# Demonstrates a simple usage of gnuplot.
#
# AUTHOR: Hagen Wierstorf 

reset

# epslatex
# NOTE: thes ize is given in inches. It can also be given in cm.
set terminal epslatex size 6.5,3.35 standalone color colortext 10
set colors podo

# Line styles
# set border linewidth 2
# set style line 1 lc rgb 'green' lt 1 lw 2 pt 9 ps 1.5
# set style line 2 lc rgb 'cyan' lt 1 lw 2 pt 11 ps 1.5
# set style line 3 lc rgb 'blue' lt 1 lw 2 pt 13 ps 1.5
# set style line 4 lc rgb 'purple' lt 1 lw 2 pt 5 ps 1.5
# set style line 5 lc rgb 'red' lt 1 lw 2 pt 7 ps 1.5

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

set output 'Simulations_ExploredDepth.tex'
set xlabel 'Simulations' font 'DejaVuSansCondensed,18'
set ylabel 'ExploredDepth' font 'DejaVuSansCondensed,18'
plot'output-28785.txt' u 1:11:12 w yerrorlines t 'rooms0:8', 'output-28786.txt' u 1:11:12 w yerrorlines t 'rooms10:8', 'output-28787.txt' u 1:11:12 w yerrorlines t 'rooms11:8', 'output-28788.txt' u 1:11:12 w yerrorlines t 'rooms11:8:polling', 'output-28789.txt' u 1:11:12 w yerrorlines t 'rooms11:8:stack', 'output-28790.txt' u 1:11:12 w yerrorlines t 'rooms11:8:polling:stack', 