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

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap

from helper_functions import TFHelper
from occupancy_field import OccupancyField
from particle import Particle


class ParticleFilter(object):
    """
    Particle filter node.
    TODO Describe what this is and what its attributes are, once we know what
    this is and what its attributes are.
    """

    def __init__(self):
        rospy.init_node('pf')

        # Helper functions.
        self.transform_helper = TFHelper()
        self.occupancy_field = OccupancyField()

        # Particle filter attributes.
        self.particle_cloud = []
        # Config attributes: TODO Document
            # n = Number of particles
            # xy_spread_size:
            # theta_spread_size:
            # xy_update_thresh:
            # theta_update_thresh:
        self.particle_cloud_config = {
            "n": 300,
            "xy_spread_size": 0.2,
            "theta_spread_size": 20,
            "xy_update_thresh": 0.2,
            "theta_update_thresh": 20
        }
        self.minimum_weight = 0.00001

        # Robot location attributes.
        # Pose estimate, stored as a triple (x, y, theta). Used to create particle cloud.
        self.xy_theta = None
        # Pose estimate, stored as a pose message type.
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
    
        # ROS Publishers/Subscribers
        # Listen for new approximate initial robot location.
        # Selected in rviz through the "2D Pose Estimate" button
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.initialize_pose_estimate)
        # Get input data from laser scan.
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)

        # Publish particle cloud for rviz.
        self.particle_pub = rospy.Publisher("/particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # TODO Add some debugging prints, actually debug...

    def initialize_pose_estimate(self, msg):
        """
        Initialize new pose estimate and particle filter. Store it as a
        triple as (x, y, theta).
        """
        print("Got initial pose.")
        self.xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.create_particle_cloud(msg.header.stamp)
        self.pose_set = True

    def update_pose_estimate(self):
        """
        Update robot's pose estimate given particles.
        TODO Improve docstring, add params etc.
        
        TODO This is still kind of an untested WIP
        """
        self.normalize_particles()
        
        # Calculate the mean particle, offset by the normalized weights.
        mean_x = 0
        mean_y = 0
        mean_x_angle = 0
        mean_y_angle = 0
        for particle in self.particle_cloud:
            mean_x += particle.x * particle.w
            mean_y += particle.y * particle.w
            total_dist = math.sqrt((particle.x)**2 + (particle.y)**2)
            mean_x_angle += total_dist * math.cos(particle.theta) * particle.w
            mean_y_angle += total_dist * math.sin(particle.theta) * particle.w      
        mean_theta = np.arctan2(float(mean_y_angle), (mean_x_angle))
        mean_x /= self.particle_cloud_config["n"]
        mean_y /= self.particle_cloud_config["n"]
        
        # TODO Make sure this actually returns the /correct/ pose.
       
        # Use particle methods to convert particle to pose.
        # TODO if this is the correct strat, do something more elegant.
        particle_mean = Particle(mean_x, mean_y, mean_theta)
        self.current_pose_estimate = particle_mean.as_pose()

    def normalize_particles(self):
        """
        Ensures particle weights sum to 1
        """
        self.set_minimum_weight()
        total_w = sum(p.w for p in self.particle_cloud)
        if total_w > 1.0:
            for i in range(len(self.particle_cloud)):
                self.particle_cloud[i].w /= total_w

    def set_minimum_weight(self):
        """
        Change any nan weights in self.particle_cloud to the minimum weight
        value instead. Modifies self.particle_cloud directly.
        """
        for p in self.particle_cloud:
            if math.isnan(p.w):
                p.w = self.minimum_weight

    def create_particle_cloud(self, timestamp):
        """
        TODO Improve docstring, add params etc.
        """
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
        self.update_pose_estimate()

    def resample(self):
        """
        Select new distribution of particles, weighted by each particle's
        weight w. Modify object's particle_cloud instance directly.
        """
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
        else:
            print ("No particle cloud, did not resample successfully.")

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

    def update_pose_delta(self, pose1, pose2):
        """
        # TODO Should this be static or in helper functions?
        Calculate floating point distance between pose triples.
        """
        self.pose_delta = [(v1 - v2) for (v1, v2) in zip(pose1, pose2)]
        return self.pose_delta

    def update_thresholds_met(self, msg):
        """
        Return whether update thresholds are met # TODO make this docstring more specifc
        Guarantee self.laser_pose and self.odom_pose for updates
        """
        # calculate pose of laser relative to the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_helper.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_helper.tf_listener.transformPose(self.odom_frame, p)

        # store the the odometry pose into (x,y,theta)
        current_pose = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose.pose)

        # if not defined yet, robot needs to move more
        if not hasattr(self, "old_pose"):
            self.old_pose = current_pose
            return False

        # return if update thresholds are exceeded
        x_d, y_d, theta_d = self.update_pose_delta(self.old_pose, current_pose)
        return math.fabs(x_d) > self.particle_cloud_config["xy_update_thresh"] or \
            math.fabs(y_d) > self.particle_cloud_config["xy_update_thresh"] or \
            math.fabs(theta_d) > self.particle_cloud_config["theta_update_thresh"]

    def odom_update(self, msg):
        """
        Use self.pose_delta to update particle locations
        """
        x_d, y_d, theta_d = self.pose_delta
        # TODO Adjust particles by the delta!

    def laser_update(self, msg):
        """
        Use scan data in msg to update particle weights
        TODO Confirm this weighting scheme works
        """
        for i in range(len(self.particle_cloud)):
            o_d = self.occupancy_field.get_closest_obstacle_distance(
                self.particle_cloud[i].x, self.particle_cloud[i].y)

            if not(math.isnan(o_d)) and o_d != 0:
                self.particle_cloud[i].w = 1.0/o_d

    def laser_scan_callback(self, msg):
        """
        Process incoming laser scan data.
        """
        if not self.pose_set:
            return

        self.tf_helper.tf_listener.waitForTransform(self.base_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
        if not(self.tf_helper.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)) or \
                not(self.tf_helper.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            return

        if self.update_thresholds_met(msg):
            # TODO Determine what else needs updating here, and make those updates!
            self.odom_update(msg)
            self.laser_update(msg)

            # Update the pose with the mean particle and resample.
            self.update_pose_estimate()
            self.resample()
            
            # Send out next map to odom transform with updated pose estimate.
            self.transform_helper.fix_map_to_odom_transform(self.current_pose_estimate, msg.header.stamp)

        # Regardless of update, publish particle cloud visualization.
        self.publish_particle_viz()

    def run(self):
        """
        TODO Improve docstring, add params etc.
        """
        r = rospy.Rate(5)
        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
