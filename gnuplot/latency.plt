set term png
set output output
set datafile missing "NA"
set title title
set offset graph 0, 0, 0.1, 0
set yr [0:]
set ylabel "Latency [ms]"
set xlabel xlabel
set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5
plot input using 1:($2) with linespoints ls 1 title ""
