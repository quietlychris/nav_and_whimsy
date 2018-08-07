###Functionality Framework

This is an overview of the functionality and implementation of nav_and_whimsy.

_Note: This section is unfinished_

This project is being built as a simulation for the efficient movement of a object
in three-dimensional space, such that the the position of this object can be planned
and mapped such that it can hit various arbitrarily placed waypoints.

#### Kinematics
First, it's important to note the degrees of freedom. The particular control scheme
allows for indepdent control of up/down (z-axis), forward/backwards (xy plane position),
and yaw (xy plane direction).

  - Travel due to accelerations in the x-y plane is assumed to be due to a
    single force like a motor, in a direction based on the yaw angle. This force would
    result in an acceleration (based on the inertial mass of the system), along with
    any external forces. Based on the yaw angle, this value is decomposed in the respective
    x and y direction accelerations, and then is numerically integrated to provide
    velocity and position values along their respective axes based on a given time-step.

  - Yaw is treated similarly, assumed to act as a perfect couple about the center of mass.

  - Travel in the z-axis operated in the same manner, and is current independent of all
    other axes, although a pitch degree of freedom may later be added which would make travel
    in the z-axis related to pitch (as well as the pitch contributions to the decomposed parts
    travel in the x-y plane).

#### Control System
The control system is obviously subject to change, and it likely going to be one of the greatest
areas of development moving forward. As of this writing (2018-06-22), control are implemented in
the following way:

  1. From the current position and desired position, get the required yaw values

  2. Calculate the difference between the current yaw and the required yaw value, as found in the
     previous step

  3. If the difference between the current yaw and the desired yaw is low enough, and the rate
     of change for yaw is below a certain threshold (to avoid drift), increase forward acceleration
     based on the position-SMD (a spring-mass-damper).

     If the yaw difference is too high, but the current total velocity in the x-y plane is below
     a certain threshold (again to prevent drift), change the yaw by producing a moment acceleration
     based on the yaw_SMD.

     If neither of the two states above are met, change direction and produce an acceleration to reduce
     the speed such that one of the conditions above are met.

     The z-axis is always undergoing corrective accelerations.

  4. Continue the above loop until the position is close enough the desired point. Mark that point as
     searched, and progress to the next waypoint.
