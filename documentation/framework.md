###Functionality Framework

This is an overview of the functionality and implementation of nav_and_whimsy.

__This part's definitely not done__

This project is being built as a simulation for the efficient movement of a object
in three-dimensional space, such that the the position of this object can be planned
and mapped such that it can hit various arbitrarily placed waypoints.

First, it's important to note the degrees of freedom. Since drag dynamics have not
yet been implemented, the standard roll vector has been eliminated, leaving only
the x,y, and z cartesian axes and the pitch and yaw angles. The leaves a control
problem with 5 degrees of freedom, with the following scheme.

  - Travel due to accelerations in the x-y plane is assumed to be due to a
    single force like a motor, in a direction based on the yaw angle. This velocity
