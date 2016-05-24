set term png
set output output
set title title
set offset 0, 0, 1, 0
set yr [0:]
set ylabel "Number of packets"
set xlabel "Impairment value"
plot input using 1:2 with lines title "Packets received", input using 1:3 with lines title "Packets published"
