set term png
set output output
set datafile missing "NA"
set title title
set yr [0:105]
set ylabel "Reception rate [%]"
set xlabel xlabel
set grid ytics lc rgb "#bbbbbb" lw 1 lt 0
set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5
plot input using 1:2:3 with yerrorbars ls 1 title "", "" using 1:2 w lines ls 1 title ""
