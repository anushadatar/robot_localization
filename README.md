# FA20 Computational Robotics: Robot Localization
*Anusha Datar and Siddharth Garimella*

Given a map, an initial pose estimate, and sensor data, narrow down areas of the map a robot is most likely to be using the particle filter algorithm. 

### The Particle Filter

Particle filtering is a common approach to localizing robots that begins with a Gaussian distribution of estimates of robot poses on the map. As the robot navigates its surroundings, data on its environment is accumulated, constraints are calculated from the consistency of our original estimates with this data, and new distributions of estimates are drawn to fit these constraints. Eventually, pose estimates converge to specific points within the map, and the robot's position becomes known.

Our implementation of this algorithm contained the following steps:

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

As continuously running the above calculations would present a large computational burden, particularly so with large particle counts, in implementation frequency of particle updates is determined by setting distance thresholds on the robot's movement. The precise number of particles, and the size of this threshold, can be determined further depending on the precision of localization required for a specific use-case.

Since particles describe the robot's position on the map, they must move with the robot, and have to be updated with odometry information for every step. This odometry "delta" is crucial for identifying not only how these particles should be translated/oriented, but if particle updates should happen in the first place. As the procedure for extracting this information is not particularly straightforward, we maintain a `pose_delta` variable within our particle filter class that gets updated continuously and makes calculating updates easier.

TODO Something something, how we calculate particle weights

### Reflection

#### Challenges

A surprising amount of time went into getting visuals to display in rviz, which preceded any real testing we could do to understand if our algorithm was functional. It would not be a stretch to say the majority of our work on this project was spent solving system configuration and implementation errors that messed with our ability to run code and observe its output on-screen, rather than the particle filtering logic itself.

#### Further Extensions

TODO What would you do to improve your project if you had more time?

#### Key Takeaways

1. Working visualizations are a priority for meaningful progress on sufficiently complex robot algorithms, especially so when there is no physical robot nearby to deploy on. 
2. While simultaneous feature work on larger software projects might work incredibly well with branch-and-merge approaches, there's often an order of operations that needs to be followed for work on algorithms. We realized this early when understanding particles need to be instantiated and visualized before we would be able to write any of the filtering logic, and decided on a back-and-forth codebase handoff approach instead. This worked well. 
