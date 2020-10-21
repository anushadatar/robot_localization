This directory contains a few .bag files with data that we collected by running our particle filter with a given map, setting an initial position, and then allowing the particle filter algorithm to determine the robot's position. 

The files included are:
- [ac109_1_demo](https://github.com/anushadatar/robot_localization/blob/master/robot_localizer/bags/ac109_1_demo.bag). In this file, the robot handles tracking the location of a moving robot as it changes its position and orientation. This test helped us ensure that our algorithm was sensitive to changes in both position and orientation on their own and together.
- [ac109_3_demo](https://github.com/anushadatar/robot_localization/blob/master/robot_localizer/bags/ac109_3_demo.bag). This is a more complicated course where there is an opportunity for the algorithm to center on an incorrect position. This test helped us consider how quickly our particles ought to converge.
- [ac109_4_demo](https://github.com/anushadatar/robot_localization/blob/master/robot_localizer/bags/ac109_4_demo.bag). This features a fairly irregular map, and it helped us identify and grapple with some corner cases.
