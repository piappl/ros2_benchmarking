set term png
set output output
set datafile missing "NA"
set title title
set xrange [0:3]
set ylabel "Time from start [s]"
set xlabel "Robot/Console"
plot input1 using (1):1 title "Packets published by console", input2 using (2):1 title "Packets received by received"
