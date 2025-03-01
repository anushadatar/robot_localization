#!/usr/bin/env python3

"""
Adapted started code for Olin College FA20 Computational Robotics robot
localization project. Project description and original starter code available
at https://comprobo20.github.io/assignments/robot_localization.
"""
import math
import numpy as np
import rospy
import time
import tf

from copy import deepcopy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Header, String

from helper_functions import TFHelper
from occupancy_field import OccupancyField
from particle import Particle


class ParticleFilter(object):
    """
    Particle Filter ROS Node.
    """

    def __init__(self):
        """
        Initialize node and necessary helper functions and p
        """
        rospy.init_node('pf')

        # Helper functions and debugging.
        # Occupancy field used to get closest obstacle distance.
        self.occupancy_field = OccupancyField()
        # Helper functions for coordinate transformations and operations.
        self.transform_helper = TFHelper()
        # Set debug to true to print robot state information to the terminal.
        self.debug = True

        # Particle filter attributes.
        # List of each particle in the filter.
        self.particle_cloud = []
        # Config attributes:
            # n = Number of particles in the particle cloud.
            # xy_spread_size: Scale factor for the spread of the x and y
            #                 coordinates of the initial particle cloud.
            # theta_spread_size: Scale factor for the spread of the angles
            #                    in the initial particle cloud.
            # xy_update_thresh: Change in x and y coordinates of the robot
            #                   position (as determined by odometry data) at
            #                   which to re-estimate robot position and
            #                   resample the particle cloud.
            # theta_update_thresh: Change (in degrees) of the robot's
            #                   orientation (as determined by odometry data) at
            #                   which to re-estimate robot position and
            #                   resample the particle cloud.
        self.particle_cloud_config = {
            "n": 100,
            "xy_spread_size": 1,
            "theta_spread_size": 30,
            "xy_update_thresh": 0.005,
            "theta_update_thresh": 0.001
        }
        # The mininum weight of a particle, used to ensure non weights are NaN.
        self.minimum_weight = 0.0000001

        # Robot location attributes.
        # Initial pose estimate, stored as a triple (x, y, theta).
        # Used to create particle cloud.
        self.xy_theta = None
        # Pose estimate, stored as a pose message type.
        # Used to track changes in pose and update pose markers.
        self.current_pose_estimate = Pose()
        # The overall change in the pose of the robot.
        self.pose_delta = [0, 0, 0]
        # Whether or not there is an initial pose value.
        self.pose_set = False
        # The frame of the robot base.
        self.base_frame = "base_link"
        # The name of the map coordinate frame.
        self.map_frame = "map"
        # The name of the odom coordinate frame.
        self.odom_frame = "odom"
        # The number of the most highly-weighted particles to incorporate
        # in the mean value used to update the robot position estimate.
        self.particles_to_incoporate_in_mean = 100
        # Adjustment factor for the magnitude of noise added to the cloud
        # during the resampling step.
        self.noise_adjustment_factor = 0.001

        # ROS Publishers/Subscribers
        # Listen for new approximate initial robot location.
        # Selected in rviz through the "2D Pose Estimate" button.
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.initialize_pose_estimate)
        # Get input data from laser scan.
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        # Publish particle cloud for rviz.
        self.particle_pub = rospy.Publisher("/particlecloud",
                                            PoseArray,
                                            queue_size=10)

    def initialize_pose_estimate(self, msg):
        """
        Initialize new pose estimate and particle cloud. Store the pose estimate as it as a
        triple with the format (x, y, theta).

        msg: PoseWithCovarianceStamped message received on the initialpose topic
             after selection of the "2D Pose Estimate" button in rviz.
        """
        if self.debug:
            print("Got initial pose.")
        self.xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.create_particle_cloud(msg.header.stamp)
        self.pose_set = True

    def update_pose_estimate(self, timestamp):
        """
        Update robot's pose estimate given the current particle cloud.
        Calculate the pose estimate using the mean of the most highly weighted
        particles, and then convert the mean coordinates and angle to a pose
        stored in self.current_pose_estimate. Call fix_map_to_odom transform
        to update the displayed current pose estimate.

        timestamp: The timestamp of the current particle cloud in type time.
        """
        self.normalize_particles()
        mean_x = 0
        mean_y = 0
        mean_theta = 0
        # Calculate the mean of the top  se
        particle_cloud_majority = sorted(self.particle_cloud, key=lambda x: x.w, reverse=True)
        for particle in particle_cloud_majority[self.particles_to_incoporate_in_mean:]:
            mean_x += particle.x * particle.w
            mean_y += particle.y * particle.w
            mean_theta = particle.theta * particle.w
        mean_x /= self.particles_to_incoporate_in_mean
        mean_y /= self.particles_to_incoporate_in_mean
        mean_theta /= self.particles_to_incoporate_in_mean

        # Use particle methods to convert particle to pose.
        current_pose_particle = Particle(mean_x, mean_y, mean_theta)
        self.current_pose_estimate = current_pose_particle.as_pose()

        # Send out next map to odom transform with updated pose estimate.
        self.transform_helper.fix_map_to_odom_transform(self.current_pose_estimate, timestamp)

    def normalize_particles(self):
        """
        Ensures particle weights sum to 1 by finding the sum of the particle
        weights and then dividing each value by this sum.

        Modifies self.particle_cloud directly.
        """
        # Ensure that none of the particle weights are NaN.
        self.set_minimum_weight()
        total_w = sum(p.w for p in self.particle_cloud)
        if total_w != 1.0:
            for i in range(len(self.particle_cloud)):
                self.particle_cloud[i].w /= total_w

    def set_minimum_weight(self):
        """
        Change any NaN weights in self.particle_cloud to the minimum weight
        value instead.

        Modifies self.particle_cloud directly.
        """
        for i in range(len(self.particle_cloud)):
            if math.isnan(self.particle_cloud[i].w):
                self.particle_cloud[i].w = self.minimum_weight

    def create_particle_cloud(self, timestamp):
        """
        Generate a new particle cloud using the parameters stored in
        self.particle_cloud_config, and then normalize the particles and
        update the current pose estimate based on the state of the created
        particle cloud.

        timestamp: The timestamp of the current particle cloud in type time.
        """
        if (self.debug):
            print("Creating particle cloud.")
        if self.xy_theta is None:
            self.xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
                self.odom_pose.pose)

        # Sample particle values from normal distributions
        x_spread = np.random.normal(
            self.xy_theta[0],
            self.particle_cloud_config["xy_spread_size"],
            self.particle_cloud_config["n"])
        y_spread = np.random.normal(
            self.xy_theta[1],
            self.particle_cloud_config["xy_spread_size"],
            self.particle_cloud_config["n"])
        theta_spread = np.random.normal(
            self.xy_theta[2], np.deg2rad(self.particle_cloud_config["theta_spread_size"]), self.particle_cloud_config["n"])

        self.particle_cloud = []
        for (x, y, theta) in zip(x_spread, y_spread, theta_spread):
            self.particle_cloud.append(Particle(x, y, theta, 1))

        self.normalize_particles()
        self.update_pose_estimate(timestamp)

    def resample(self):
        """
        Select new distribution of particles, weighted by each particle's
        weight w. Then add some noise to the system to aid in visualization and
        increase system robustness.

        Modifies self.particle_cloud instance directly.
        """
        self.set_minimum_weight()
        if len(self.particle_cloud):
            self.normalize_particles()
            weights = [particle.w  if not math.isnan(particle.w) else self.minimum_weight for particle in self.particle_cloud]

            # Resample points based on their weights.
            self.particle_cloud = [deepcopy(particle) for particle in list(np.random.choice(
                    self.particle_cloud,
                    size=len(self.particle_cloud),
                    replace=True,
                    p=weights,
                ))]

            # Add noise to each particle.
            for p in self.particle_cloud:
               particle_noise = np.random.randn(3)
               p.x += particle_noise[0] * self.noise_adjustment_factor
               p.y += particle_noise[1] * self.noise_adjustment_factor
               p.theta += particle_noise[2] * self.noise_adjustment_factor

        if self.debug:
            print("Resampling executed.")

    def publish_particle_viz(self):
        """
        Publish a visualization of self.particle_cloud for use in rviz.
        """
        self.particle_pub.publish(
            PoseArray(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id=self.map_frame),
                poses=[
                    p.as_pose() for p in self.particle_cloud]))

        if self.debug:
            print("Publishing new visualization.")

    def update_pose_delta(self, pose1, pose2):
        """
        Calculate floating point distance between pose triples.

        pose1: The first pose, stored as a triple (x, y, theta).
        pose2: The second pose, stored as a triple (x, y, theta).
        Returns a new triple with the difference between the poses in the
        form of (x, y, theta) and also updates self.pose_delta.
        """
        self.pose_delta[0] = pose2[0] - pose1[0]
        self.pose_delta[1] = pose2[1] - pose1[1]
        self.pose_delta[2] = self.transform_helper.angle_diff(pose2[2], pose1[2])
        return self.pose_delta

    def update_thresholds_met(self, msg):
        """
        Calculate the estimated laser scan and robot pose, and then update the
        current pose. Return if the difference between the previous and current
        pose exceeds a given threshold.

        msg: Incoming laser scan data of message type LaserScan.

        Returns a boolean indicating whether the change in the robot's position
        exceeds the given movement threshold.
        """
        # Calculate pose of laser relative to the robot base.
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.transform_helper.tf_listener.transformPose(self.base_frame, p)

        # Find out where the robot thinks it is based on its odometry.
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.transform_helper.tf_listener.transformPose(self.odom_frame, p)

        # Store the the odometry pose into (x,y,theta).
        current_pose = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose.pose)

        # If the old pose has not definition, the robot needs to move more.
        if not hasattr(self, "old_pose"):
            self.old_pose = current_pose
            return False

        # Otherwise, update self.pose_delta and return whether its magnitude
        # exceeds the threshold.
        x_d, y_d, theta_d = self.update_pose_delta(self.old_pose, current_pose)
        self.old_pose = current_pose
        return math.fabs(x_d) > self.particle_cloud_config["xy_update_thresh"] or \
            math.fabs(y_d) > self.particle_cloud_config["xy_update_thresh"] or \
            math.fabs(theta_d) > self.particle_cloud_config["theta_update_thresh"]

    def odom_update(self):
        """
        Use self.pose_delta to update particle locations to reflect the change
        in the robot's position.

        Modifies self.particle_cloud directly.
        """
        x_d, y_d, theta_d = self.pose_delta
        for i in range(len(self.particle_cloud)):
            self.particle_cloud[i].x -= x_d
            self.particle_cloud[i].y -= y_d
            self.particle_cloud[i].theta += theta_d

    def laser_update(self, msg):
        """
        Use scan data in msg to update particle weights by using the occupancy
        field to determine the distance from the closest obstacle and
        adjusting the particle weights accordingly.

        msg: Incoming laser scan data of message type LaserScan.

        Modifies self.particle_cloud in place.
        """
        for particle in self.particle_cloud:
            # Get total distances for each angle for each particle.
            total_distance = 0
            x_values = msg.ranges * np.cos(np.degrees(np.arange(0, 361, dtype=float) + particle.theta))
            y_values = msg.ranges * np.sin(np.degrees(np.arange(0, 361, dtype=float) + particle.theta))

            for x, y in zip(x_values, y_values):

                # Disregard any invalid distances.
                if x == 0 and y == 0:
                    continue
                o_d = self.occupancy_field.get_closest_obstacle_distance(particle.x + x, particle.y + y)
                if not(math.isnan(o_d)) and o_d != 0:
                    # Cube the distance to weight closer objects more highly.
                    total_distance += o_d**3

            # Set particle weight to the inverse of the total distance.
            if total_distance > 0:
                particle.w = 1.0/total_distance

        # Ensure that particle weights sum to 1.
        self.normalize_particles()

    def laser_scan_callback(self, msg):
        """
        Process incoming laser scan data. If the change in position exceeds
        the thresholds, update the pose estimate and resample.

        msg: Incoming laser scan data of message type LaserScan.
        """
        if not self.pose_set:
            return

        self.transform_helper.tf_listener.waitForTransform(self.base_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
        if not(self.transform_helper.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)) or \
                not(self.transform_helper.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            return

        # Regardless of update, publish particle cloud visualization.
        self.publish_particle_viz()

        if self.update_thresholds_met(msg):
            self.odom_update()
            self.laser_update(msg)

            # Update the self.current_pose_estimate with the mean particle and resample.
            self.update_pose_estimate(msg.header.stamp)
            self.resample()

        else:
            if self.debug:
                print("Update thresholds not met!")


    def run(self):
        """
        As most of the processing happens through callback functions for laser
        scan and odometry data, simply publish the most recent transform for as
        long as the node runs.
        """
        r = rospy.Rate(2)
        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
