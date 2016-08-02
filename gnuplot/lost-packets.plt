set term png
set output output
set datafile missing "NA"
set title title
set yr [0:100.1]
set ylabel "Reception rate [%]"
set xlabel xlabel
set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5
plot input using 1:(100*$2/$3) with linespoints ls 1 title ""
