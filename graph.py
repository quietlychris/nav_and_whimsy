import scipy
import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


print("Generating graph: ")
data = numpy.loadtxt(open("run_log.csv", "rb"), delimiter=",",skiprows=1)
#control = numpy.loadtxt(open("./secondaries/graph-sample.csv","rb"),delimiter=",")

# imports x,y,z position data
x = data[:,1]
y = data[:,0]
z = data[:,2]


fig = plt.figure()  # Generates figure
ax = Axes3D(fig)    # Uses 3D module to create graph
ax.scatter(x,y,z)   # plots data
plt.show()
fig.savefig("graph.png")

#plt.xlabel('x-axis [m]')   # labels axes
#plt.ylabel('y-axis [m]')
#plt.zlabel('z-axis [m]')

quit()
