set terminal x11 size 1800,1000 enhanced font 'Verdana,9' persist

set multiplot layout 4,1 #title "Pronto vs VICON" font ",14"
set tmargin 2
set grid

# Setting up a desired maximum range in x
if (exists("max_x")) set xrange [0:max_x]

# Setting up the filename
if (!exists("filename")) filename='~/.ros/preview_opt.dat'

set ylabel "Command cost"
plot filename using 1 title '' with lines

set ylabel "Energy cost"
plot filename using 2 title '' with lines

set ylabel "Stability cost"
plot filename using 3 title '' with lines

set ylabel "Model cost"
set xlabel "Number of iterations"
plot filename using 4 title '' with lines

unset multiplot
