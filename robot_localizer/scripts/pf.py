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

from tf import TransformListener
from tf import TransformBroadcaster

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

        # Transform helpers
        self.transform_helper = TFHelper()
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # Particle filter attributes.
        self.particle_cloud = []
        self.particle_cloud_config = {
            "n": 300,
            "xy_spread_size": 0.2,
            "theta_spread_size": 20
        }

        # Pose estimates, stored as a triple (x, y, theta)
        self.xy_theta = None
        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame

        # Listen for new approximate initial robot location.
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.initialize_pose_estimate)

        # Publish particle cloud for rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)
        # Get input data from laser scan.
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)

        # Initialize occupancy field
        self.occupancy_field = OccupancyField()

    def initialize_pose_estimate(self, msg):
        """
        Initialize new pose estimate and particle filter. Store it as a
        triple as (x, y, theta).
        """
        self.xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        # TODO Implement any other initialization, maybe have a boolean about
        # if this happens
        self.create_particle_cloud(msg.header.stamp)

    def update_pose_estimate(self, timestamp):
        """
        Update robot's pose estimate given particles.
        TODO decide what to use to estimate (mean? mode?)
        TODO Improve docstring, add params etc.
        """
        # TODO Update this to be a real pose, computed based on particle
        # likelihood.
        # TODO Stop using the origin as a placeholder.
        self.robot_pose = Pose()

        self.transform_helper.fix_map_to_odom_transform(
            self.robot_pose, timestamp)

    def normalize_particles(self):
        """
        Ensures particle weights sum to 1
        """
        total_w = sum(p.w for p in self.particle_cloud)
        for i in range(len(self.particle_cloud)):
            self.particle_cloud[i].w /= total_w

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
        self.update_pose_estimate(timestamp)

    def resample(self):
        """
        TODO Use probability, resample points.
        TODO Improve docstring, add params etc.
        """
        pass

    def publish_particle_viz(self, msg):
        """
        Publish a visualization of the particles for use in rviz.
        TODO Improve docstring, add params etc.
        """
        self.particle_pub.publish(
            PoseArray(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id=self.map_frame),
                poses=[
                    p.as_pose() for p in self.particle_cloud]))

    def laser_scan_callback(self, msg):
        """
        Process incommming laser scan data.
        TODO Improve docstring, add params etc.
        """
        # TODO Make sure we have set the initial pose

        if not(self.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)) or \
                not(self.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            return

        # TODO Find laser pose relative to the robot
        # TODO Find robot position by odometry
        # TODO If we don't have a particle cloud, initialize one. If we do, update it as needed.
        # TODO Update robot pose
        self.update_pose_estimate(msg.header.stamp)
        self.resample()
        self.publish_particle_viz(msg)

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
