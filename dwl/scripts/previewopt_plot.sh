#!/bin/bash


# Display help
programname=$0
function usage {
    echo "usage: $programname [--file:infile] [--max_iter:] [--min_y:] [--max_y:]"
    echo "	--help, -h			print this message"
    echo "	--file,				specify input file infile"
    echo "	--max_iter:			specify a maximum number of iteration to plot"
    echo "	--min_y:			specify a minimum y to plot"
    echo "	--max_y:			specify a maximum y to plot"
    exit 1
}


# Check arguments
filename=$HOME/.ros/preview_opt.dat
for ARG in "$@"; do
	if [ $ARG == "--help" ] || [ $ARG == "-h" ]; then  # Display help
		usage
		exit
	elif [ ${ARG:0:7} == "--file:" ]; then
		filename=${ARG:6}
	elif [ ${ARG:0:11} == "--max_iter:" ]; then
		echo ${ARG:11}
		max_iter=${ARG:11}
	elif [ ${ARG:0:8} == "--min_y:" ]; then
		min_y=${ARG:8}
		echo ${ARG:8}
	elif [ ${ARG:0:8} == "--max_y:" ]; then
		max_y=${ARG:8}
	fi
done


################################################################################
## run gnuplot
gnuplot -persist << EOF
# Setting up a terminal output for the cost evolutions
set terminal qt size 1800,1000 enhanced font 'Verdana,9' persist

# Setting up the plotting range
set xrange [:$max_iter]
set yrange [$min_y:$max_y]

# Declaring the multiplot layout
set multiplot layout 4,1
set tmargin 2
set grid

# Command cost plot
set ylabel "Command cost"
plot "$filename" using 1 title '' with lines

# Energy cost plot
set ylabel "Energy cost"
plot "$filename" using 2 title '' with lines

# Stability cost plot
set ylabel "Stability cost"
plot "$filename" using 3 title '' with lines

# Model cost plot
set ylabel "Model cost"
set xlabel "Number of iterations"
plot "$filename" using 4 title '' with lines

unset multiplot
EOF




###############################################################################
# BORDER EXPERIMENTAL CODE
#rows=$(cat $filename | wc -l)
#columns=$(head $filename -n1 | wc -w)
#echo $rows
#echo $columns

#string=$(head -n 1 $filename)
#IFS=$'\t' read -r -a array <<< "$string"

#for index in "${!array[@]}"
#do
#	if [[ ${array[index]} == x* ]]; then
#		echo "plot index"
#		echo "$index ${array[index]}"
#	fi
#done

#	set yrange [*:*]
#	set terminal X11 1
#	set multiplot layout 3,1


#	set ylabel "Time duration"
#	plot "$filename" using 5 title columnheader with lines,\
#		 "$filename" using 6 title columnheader with lines,\
#		 "$filename" using 7 title columnheader with lines,\
#		 "$filename" using 8 title columnheader with lines,\
#		 "$filename" using 9 title columnheader with lines,\
#		 "$filename" using 10 title columnheader with lines,\
#		 "$filename" using 11 title columnheader with lines,\
#		 "$filename" using 12 title columnheader with lines,\
#		 "$filename" using 13 title columnheader with lines,\

#	set ylabel "CoP x-shift"
#	plot "$filename" using 6 title '' with lines

#	set ylabel "CoP y-shift"
#	plot "$filename" using 7 title '' with lines
	#splot "$filename" using 6:7:0 title '' with lines
#	unset multiplot
