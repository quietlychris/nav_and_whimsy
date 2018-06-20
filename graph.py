import scipy
import numpy
import csv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# imports waypoint data
wp_file = open("waypoints.csv")
row_count = sum(1 for row in wp_file)  # fileObject is your csv.reader
waypoints = numpy.loadtxt(open("waypoints.csv","rb"),delimiter=",")
#print("number of waypoints are", row_count)
if row_count == 1:
   a = waypoints[0]
   b = waypoints[1]
   c = waypoints[2]
else:
   a = waypoints[:,0]
   b = waypoints[:,1]
   c = waypoints[:,2]

# imports data from the run log
with open("run_log.csv", "rb") as f:
    reader = csv.reader(f)
    i = next(reader)

# Parses the header line of data file to find the right columns to plot
for num in range(0,len(i)):
    if i[num] == 'xaxis.var':
        xcolumn = num
    elif i[num] == 'yaxis.var':
        ycolumn = num
    elif i[num] == 'zaxis.var':
        zcolumn = num

data = numpy.loadtxt(open("run_log.csv", "rb"), delimiter=",",skiprows=1)

# samples the total data set to lower computational load of viewing 3D plot
# sample_size defines sample rate of data (i.e. sample_size = 50 - 1/50 points)
sample_size = 10
x = data[::sample_size,xcolumn]
y = data[::sample_size,ycolumn]
z = data[::sample_size,zcolumn]

print("Generating graph: ")
fig = plt.figure()  # Generates figure
ax = Axes3D(fig)    # Uses 3D module to create graph
ax.scatter(x,y,z)   # plots path data
ax.scatter(a,b,c)   # plots waypoints
ax.set_xlabel('x-axis [m]')
ax.set_ylabel('y-axis [m]')
ax.set_zlabel('z-axis [m]')

plt.show()
fig.savefig("graph.png")

quit()
