## status
- we have a nonlinear system
- we linearized the system about 'n' (where n = 2 in the runner) equilibrium points
    - so now we have 2 linear systems
- we designed a controller per system
    - namely, we designed a stabilizing proportional controller for the upper-position and 
    a destabilizing controller for the lower-position.
    - one could easily then interpolate the proportional gains for any intermediate regions
    that we have (so we increase 'n' to be 6 and keep the same bottom and top proportional 
    gains, but interpolate gains between the top and bottom regions for smoother transition)
    - one could easily design more complex controllers as well
- now we have a closed-loop system which we can feed reference signals to
- next, we must design a system to generate reference waypoints, which, if moved to, will
not violate temporal logic specs
- circling back to the feedback controller in the closed-loop system, we must design the 
system to meet certain specs regarding overshoot, settling-time etc, which we then exploit
from the exterior path planner
  - if we know the overshoot will be less than 25%, then we need to make sure that any 
  reference waypoint we give is at least 25% in front of a obstacle
- first assume we have complete knowledge of the environment.
- we have a goal waypoint and we have a starting waypoint, create a list of waypoints from 
one to the other so it will reach the goal in finite time without entering into a forbidden
zone
- next include a specification that we won't enter a certain area
- next include a specification that we won't go above a certain speed
- next include a specification that we won't go above a certain input torque
- can we demonstrate a priori that we won't meet a spec even if we demand it? (like input torque spec?)


## longer term goal:
- for each variable we linearize about the main points (at least 2 points: one stable, one unstable), 
- there are N linearization combinations (2x2x2x2x....=N)
- we search over the linearizations for a path from unstable to stable