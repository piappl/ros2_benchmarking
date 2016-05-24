set term png
set output output
set title title
binwidth=1
bin(x,width)=width*floor(x/width)
set offset 0, 0, 10, 0
set boxwidth binwidth*0.9
set xrange [0:]
set xlabel "Time from start [s]"
set ylabel label
plot input using (bin($1,binwidth)):(1.0) smooth freq with boxes title legend
