# nav_and_whimsy

The goal of nav_and_whimsy is to be a 3D kinematic movement and localization simulation.
It is in development, and not ready for use. To use, clone and use ./run.sh to run.
Key dependencies include an up-to-date version of the Rust package manager 'cargo',
as well as 'numpy' and 'scipy'.

The following is a graph of the 2D implementation, with full kinematic implementation.
The control system is implemented using virtual spring-mass-damper systems based on the
difference between existing and desired states. These produce two acceleration values
(like those that could be generated by a motor or thruster being activated) for both yaw
and forward/backward thrust, which are numerically integrated to provide velocity and
position values in the x-y plane.

<img src="/documentation/images/20180614_2D_graph.png" width="450" height="350" />

For more documentation, and updates on the development, please refer to
the documentation, which is written in Markdown using GitBook.

Important note: Less whimsical than desired.
