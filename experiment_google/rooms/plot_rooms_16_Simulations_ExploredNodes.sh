#!/usr/bin/env gnuplot
#
# Demonstrates a simple usage of gnuplot.
#
# AUTHOR: Hagen Wierstorf 

reset

# epslatex
# NOTE: thes ize is given in inches. It can also be given in cm.
set terminal epslatex size 3.5,2.62 standalone color colortext 10
#set colors podo

#set output 'terminal_epslatex.tex'

# Line styles
set border linewidth 1
set style line 1 lc rgb 'red' lt 1 lw 1 pt 7 ps 1.
set style line 2 lc rgb 'green' lt 1 lw 1 pt 9 ps 1.
set style line 3 lc rgb 'blue' lt 1 lw 1 pt 13 ps 1.
set style line 4 lc rgb 'cyan' lt 1 lw 1 pt 11 ps 1.
set style line 5 lc rgb 'purple' lt 1 lw 1 pt 5 ps 1.

# Legend
#set key left top Left
set key font ",14"
set key right bottom Right
# Axes label
#set xlabel '$x$'
#set ylabel '$y$'
# Axis ranges
#set xrange[1:100000]
#set yrange[-1.5:1.5]
# Tics in LaTeX format
# Axis labels
set format x "%g"
set log x
set grid
#set xlabel "Number of Iterations" font "DejaVuSansCondensed,18"
#set ylabel "Avg. Discounted Return" font "DejaVuSansCondensed,18"
#plot './output-RoundRobin.txt' u 1:2 w lp ls 1 t 'RoundRobin', './output-Randomized.txt' u 1:2 w lp ls 2 t 'Randomized', './output-Greedy.txt' u 1:2 w lp ls 3 t '0.5-Greedy', './output-UCB.txt' u 1:2 w lp ls 4 t 'UCB1', './output-ThompsonSampling.txt' u 1:2 w lp ls 5 t 'ThompsonSampling'
#plot './uct/rocksample-7,8.txt' u 1:5:6 w yerrorlines ls 3 t 'POMCP', './dng/rocksample-7,8.txt' u 1:5:6 w yerrorlines ls 5 t 'D$^2$NG-POMCP',
#plot './uct/output-28472.txt' u 1:5 w lp ls 3 t 'POMCP', './dng/output-2050.txt' u 1:5 w lp ls 5 t 'D$^2$NG-POMCP'

set output 'rooms_16_Simulations_ExploredNodes.tex'
set xlabel 'Simulations' font 'DejaVuSansCondensed,18'
set ylabel 'ExploredNodes' font 'DejaVuSansCondensed,18'
plot