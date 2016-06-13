set term png
set output output
set title title
set offset 0, 0, 1, 0
set yr [0:]
set ylabel "Number of packets"
set xlabel "Impairment value"
set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5
plot input using 1:(100*$2/$3) with linespoints ls 1 title "Packets received"
#, input using 1:3 with lines title "Packets published"
