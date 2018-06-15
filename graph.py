import scipy
import numpy
import matplotlib.pyplot as plt

print("Generating graph: ")
data = numpy.loadtxt(open("run_log.csv", "rb"), delimiter=",",skiprows=1)
waypoint = numpy.loadtxt(open("waypoints.csv","rb"),delimiter=",")

x = data[:,0]
y = data[:,1]

f = waypoint[:,0]
g = waypoint[:,1]

fig = plt.figure()
plt.plot(x,y)
plt.scatter(f,g)
#plt.xlim(-100,100)
#plt.ylim(-100,100)
plt.show()
fig.savefig("graph.png")
plt.xlabel('x-axis [m]')
plt.ylabel('y-axis [m]')
quit()
