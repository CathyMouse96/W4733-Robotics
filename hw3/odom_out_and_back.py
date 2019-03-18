#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import sys
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # Subscriber to obtain the distance to nearest obstacle
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # How fast will we update the robot's movement?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        linear_speed = 0.15
        
        # Set the travel distance in meters
        goal_distance = 10.0

        # Set the rotation speed in radians per second
        angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)
        
        # Set the rotation angle to Pi radians (180 degrees)
        goal_angle = pi

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        # Initialize the position variable as a Point type
        position = Point()
            
        # Initialize the movement command
        move_cmd = Twist()
        
        # Set the movement command to forward motion
        move_cmd.linear.x = linear_speed
        
        # Get the starting position values     
        (position, rotation) = self.get_odom()
                    
        x_start = position.x
        y_start = position.y

        # Goal position values
        x_goal = x_start + goal_distance
        y_goal = y_start
        
        # Keep track of the distance traveled
        distance = 0

        # Hit point
        hit_p = None

        # Threshold distance to obstacle before turning
        threshold_dist = 0.05
        
        while not rospy.is_shutdown():
            if self.goaltest(position, x_goal, y_goal): # Success!
                # Stop the robot
                self.cmd_vel.publish(Twist())
                break
            elif self.ahead_range < threshold_dist: # Obstacle encountered
                # Turn left
                self.cmd_vel.publish(Twist())
                break
            else:
		        print(self.ahead_range)
                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmd)
                r.sleep()
    
                # Get the current position
                (position, rotation) = self.get_odom()
                # Compute the Euclidean distance from the start
                distance = sqrt(pow((position.x - x_start), 2) + 
                                pow((position.y - y_start), 2))
        
        # Set the movement command to a rotation
        # move_cmd.angular.z = angular_speed
            
        # Track the last angle measured
        # last_angle = rotation
        
        # Track how far we have turned
        # turn_angle = 0
        
        # while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            # self.cmd_vel.publish(move_cmd)
            # r.sleep()
            
            # Get the current rotation
            # (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last loop
            # delta_angle = normalize_angle(rotation - last_angle)
            
            # Add to the running total
            # turn_angle += delta_angle
            # last_angle = rotation
            
        # Stop the robot before the next leg
        # move_cmd = Twist()
        # self.cmd_vel.publish(move_cmd)
        # rospy.sleep(1)
            
        # Stop the robot for good
        # self.cmd_vel.publish(Twist())

    def goaltest(self, position, x_goal, y_goal):
        return position.x == x_goal and position.y == y_goal

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
    
    def scan_callback(self, msg):
	laser_min = msg.range_max
	for i in msg.ranges:
	    if not isnan(i):
		laser_min = min(laser_min, i)
	self.ahead_range = laser_min
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

