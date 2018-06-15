import scipy
import numpy
import csv
import matplotlib.pyplot as plt

with open("run_log.csv", "rb") as f:
    reader = csv.reader(f)
    i = next(reader)

# Parses the header line of data file to find the right columns to plot
for num in range(0,len(i)):
    if i[num] == 'xaxis.var':
        xcolumn = num
    elif i[num] == 'yaxis.var':
        ycolumn = num

print("Generating graph: ")
data = numpy.loadtxt(open("run_log.csv", "rb"), delimiter=",",skiprows=1)
waypoint = numpy.loadtxt(open("waypoints.csv","rb"),delimiter=",")

a = data[:,xcolumn]
b = data[:,ycolumn]

f = waypoint[:,0]
g = waypoint[:,1]

fig = plt.figure()
plt.plot(a,b)
plt.scatter(f,g)
plt.xlabel('x-axis [m]')
plt.ylabel('y-axis [m]')
plt.show()
fig.savefig("graph.png")
quit()
