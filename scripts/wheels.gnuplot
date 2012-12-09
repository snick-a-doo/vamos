#-*-gnuplot-*-
#
# Plot single-column data from each of four wheels.  This is handy if
# you stick printing statements in a wheel (tire, suspension, etc.)
# class and you want to be able to tell which wheel the data came
# from.
#
# Execute from within Gnuplot with
# 	load 'wheels.gnuplot'
#
plot [x=0:][-5000:5000]\
     'out' every 4::0 title 'Right Front',\
     'out' every 4::1 title 'Left Front',\
     'out' every 4::2 title 'Right Rear',\
     'out' every 4::3 title 'Left Rear'