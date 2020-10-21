# FA20 Computational Robotics: Robot Localization
*Anusha Datar and Siddharth Garimella*

Given a map, an initial pose estimate, and sensor data, narrow down areas of the map a robot is most likely to be by using the particle filter algorithm. 

### The Particle Filter

Particle filtering is a common approach to localizing robots that begins with a Gaussian distribution of estimates of potential robot poses on the map. As the robot navigates its surroundings, it accumulates data on its environment, calculates constraints from the consistency of the original estimates with this data, and draws new distributions of estimates to fit these constraints. Eventually, pose estimates converge to specific points within the map, and the robot's position becomes known.

![Particle Filter GIF](https://github.com/anushadatar/robot_localization/blob/master/report_visuals/ac109_1.gif "Initial Demo GIF")

Our implementation of this algorithm contains the following steps:

-   Seed random points `point_cloud` that represent the set of potential locations for the robot
-   While we haven’t found the robot’s position (i.e. we have not converged to a single point):
    -  Store the robot’s laser scan data `laser_scan` to determine the position of obstacles around it. If the robot is moving, update this variable to align with changes in odometry.
    -   For each point in the `point_cloud`:
        -   Measure how close the particle is to some position on the map (given the likelihood field)
        -   Weight particles to reward closeness to `laser_scan`
        -   Resample points into `point_cloud` using measurements, weight sample based on previously weighted points
        -   Update robot’s pose estimate, converting from mapped points to odometry values
        -   If some threshold of points in `point_cloud` meet some threshold of compatibility with `laser_scan`, return the robot’s position

### Design Decisions

As continuously running the above calculations would present a large computational burden, particularly so with large particle counts, we decided to determine the frequency of particle updates by setting distance thresholds on the robot's movement. The precise number of particles, and the size of this threshold, can be determined further depending on the resolution of localization required for a specific use case.

Since particles describe the robot's position on the map, they must move with the robot, and have to be updated with odometry information for every step. This odometry "delta" is crucial for identifying not only how these particles should be translated/oriented, but if particle updates should happen in the first place. As the procedure for extracting this information is not particularly straightforward, we maintain a `pose_delta` variable within our particle filter class that we continuously update and reference.

We spent some time understanding how and why our particles converge so quickly. One reason we were able to deduce was the uniformity of our particle weighting scheme, as we simply calculated the inverse of the sum of the distances returned by the occupancy field for each angle. To make our implementation more robust, we chose to cube each of the distances returned by the occupancy field, and then make the weight of each particle the inverse of the sum of these cubed distances. This converged to a closer pose estimate, and looked more like the expected behavior.

### Reflection

#### Challenges

A surprising amount of time went into getting visuals to display in rviz, which preceded any real testing we could do to understand if our algorithm was functional. It would not be a stretch to say the majority of our work on this project was spent solving system configuration and implementation errors that messed with our ability to run code and observe its output on-screen, rather than the particle filtering logic itself.

Distinguishing between the work being done by our odometry update and our actual localization algorithm was also somewhat tricky, and this was amplified by arrows converging to points and “stacking”, which led us to think our particles were disappearing. Adding in noise and debugging changes step-by-step in rvz allowed us to find false flags here and keep moving forward.

#### Further Extensions

We could probably be better about how we weighted particles, as our current function is quite rudimentary and still may not produce sufficiently differentiated values. Being more systematic about this could likely yield a more accurate pose estimation.

There are also interesting “corner” cases where our particle filtering algorithm is tricked to converge in the wrong spots on the map, at least initially. We’re curious how we might approach making our particle filter more robust so as not to be tripped up on these.

There are a variety of larger features we could also pursue, such as enhanced visualization techniques and implementing the ray tracing function proposed as an extension in the scaffolded code.

#### Key Takeaways

1. Working visualizations are a priority for meaningful progress on sufficiently complex robot algorithms, especially so when there is no physical robot nearby to deploy on. 
2. While simultaneous feature work on larger software projects might work incredibly well with branch-and-merge approaches, there's often an order of operations that needs to be followed for work on algorithms. We realized this early when understanding particles need to be instantiated and visualized before we would be able to write any of the filtering logic, and decided on a back-and-forth codebase handoff approach instead. This worked well.

