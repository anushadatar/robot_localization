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
            "theta_spread_size": 20,
            "xy_update_thresh": 0.2,
            "theta_update_thresh": 20
        }
        self.minimum_weight = 0.00001

        # Pose estimates, stored as a triple (x, y, theta)
        self.xy_theta = None
        self.pose_delta = [0, 0, 0]
        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame

        # Listen for new approximate initial robot location.
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.initialize_pose_estimate)
        self.pose_set = False

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
        self.create_particle_cloud(msg.header.stamp)
        self.pose_set = True

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
        if total_w > 1.0:
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
        Select new distribution of particles, weighted by each particle's
        weight w. Modify object's particle_cloud instance directly.
        """
        if len(self.particle_cloud):
            self.normalize_particles()
            weights = [particle.w  if not math.isnan(particle.w) else self.minimum_weight for particle in self.particle_cloud]
            # Resample points based on their weights.
            self.particles = [particle.deep_copy() for particle in list(np.random.choice(
                self.particles,
                size=len(self.particles),
                replace=True,
                p=weights,
            ))]
        else:
            print ("No particle cloud, did not resample successfully.")

    def publish_particle_viz(self, msg):
        """
        Publish a visualization of the particles for use in rviz.
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
        Floating point distance between pose triples
        """
        self.pose_delta = [(v1 - v2) for (v1, v2) in zip(pose1, pose2)]
        return self.pose_delta

    def update_thresholds_met(self, msg):
        """
        Return whether update thresholds are met
        Guarantee self.laser_pose and self.odom_pose for updates
        """
        # calculate pose of laser relative to the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

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
        pass

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

        self.tf_listener.waitForTransform(self.base_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, msg.header.frame_id, msg.header.stamp)) or \
                not(self.tf_listener.canTransform(self.base_frame, self.odom_frame, msg.header.stamp)):
            return

        if self.update_thresholds_met(msg):
            # TODO quick math
            self.odom_update(msg)
            self.laser_update(msg)

            # TODO ensure particles stay normalized through pose update
            self.normalize_particles()
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
