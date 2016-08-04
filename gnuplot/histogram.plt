set terminal postscript eps size 3.5,2.62 enhanced color
set output output
set datafile missing "NA"
set title title
binwidth=1
bin(x,width)=width*floor(x/width)
set offset 0, 0, 10, 0
set boxwidth binwidth*0.9
set xrange [0:]
set xlabel "Time from start [s]"
set ylabel label
set style fill solid 1.0 noborder
plot input using (bin($1,binwidth)):(1.0) linecolor rgb "#0060ad" smooth freq with boxes title legend
