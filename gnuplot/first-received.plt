set term png
set output output
set title title
set offset 0, 0, 1, 0
set yr [0:]
set ylabel "Delay [s]"
set xlabel "Impairment value"
plot input using 1:2 with lines title "First packet received"
