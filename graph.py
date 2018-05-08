import scipy
import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


print("Generating graph: ")
data = numpy.loadtxt(open("run_log.csv", "rb"), delimiter=",",skiprows=1)
waypoints = numpy.loadtxt(open("waypoints.csv", "rb"), delimiter=",",skiprows=0)

# imports x,y,z position data
x = data[:,0]
y = data[:,1]
z = data[:,2]

# imports waypoint data
a = waypoints[:,0]
b = waypoints[:,1]
c = waypoints[:,2]

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
