#!/usr/bin/env python3

"""
Particle class for robot localization particle filter. Adapted from particle
class given in assignment's pf_scaffold.py:
https://github.com/comprobo20/robot_localization/blob/master/robot_localizer/scripts/pf_scaffold.py
"""
import math
import tf

from geometry_msgs.msg import Pose, Point, Quaternion


class Particle(object):
    """ 
    Represents a hypothesis (particle) of the robot's pose consisting of x,y
    and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights
        are normalized)
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ 
        Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle
            weights are normalized)
        """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ 
        A helper function to convert a particle to a geometry_msgs/Pose message.
        """
        orientation_tuple = tf.transformations.quaternion_from_euler(
            0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0), orientation=Quaternion(
            x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))
