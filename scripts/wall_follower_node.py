#!/usr/bin/env python
"""
    wall_follower_node.py

    A ROS node that commands a Turtlebot3 to follow a wall to its right
    according to the latest laser scan.

    Subscribed: scan/, cmd/
    Publishes: cmd_vel/

    Created: 2020/02/04
    Author: Brendan Halloran
"""

import rospy
import numpy as np 
import tf
import math
import transformations as trans
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from enum import Enum
from commands import RobotState

# Constants
BASE_FRAME =        "base_link"         # Param from the SLAM module
MAX_SIDE_LIMIT =    0.50                # This furthest distance we 'see' the wall to the side
MIN_APPROACH_DIST = 0.30                # The closest we want to get to a wall from the front
MAX_APPROACH_DIST = 0.50                # The distance we want to start slowing to approach a front wall
ROBOT_RADIUS =      0.20                # The bounding circle around the robot
MAX_TRANS_SPEED =   0.25                # Forward movement
MAX_TURN_SPEED =    1.4                 # Rotation

class FollowSide(Enum):
    LEFT = 1
    RIGHT = 2

class wall_follower_node:
    def __init__(self):
        self.stopped = False    # Assume robot is moving and needs to be stopped first
        self.explore = False
        self.follow_side = FollowSide.LEFT

        self.transform_listener = tf.TransformListener()

        self.subscriber_laser_scan = rospy.Subscriber('scan/', LaserScan, self.callback_laser_scan)
        self.subscriber_state = rospy.Subscriber('state/', String, self.callback_state)
        self.publisher_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Send a final stop command before shutting off the node
    def shutdown(self):
        if not self.stopped:
            self.stopped = True
            self.explore = False
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.publisher_twist.publish(vel_msg)


    def callback_laser_scan(self, data):
        # If we are no longer exloring don't process the scan
        if not self.explore:
            # If we aren't exploring, but we haven't told the robot to stop, do so now.
            # This is located here rather than in the other callback because the stop command
            # might be received halfway through processing a laser scan, then the laser scan
            # callback's move command will come after the stop and the robot will start moving
            # again.
            if not self.stopped:
                self.stopped = True
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.publisher_twist.publish(vel_msg)
            return
        # Otherwise, process the scan.

        # Find the transform between the laser scanner and the robots base frame
        # The transformations are all populated at startup
        try:
            self.transform_listener.waitForTransform(BASE_FRAME, data.header.frame_id, data.header.stamp, rospy.Duration(2.0))
            position, quaternion = self.transform_listener.lookupTransform(BASE_FRAME, data.header.frame_id, data.header.stamp)
            transformation = trans.pq_to_se3(position, quaternion);
        except tf.Exception:
            print "Unable to get transformation!"


        # We want to find how far in front of the robot the wall to the left extends
        # and how far away the nearest point in front of the robot is.
        x_side_max = -np.inf
        x_front_min = np.inf
        angle = data.angle_min

        # Transform scans into 3D points relative to robots base frame
        for scan in data.ranges: 
            # 3D homogeneous point [x; y; z; 1]
            point = np.array([[math.cos(angle) * scan], [math.sin(angle) * scan], [0], [1]])
            point = np.matmul(transformation, point);

            # Update angle
            angle += data.angle_increment

            # Unwrap
            point_x = point[0][0]
            point_y = point[1][0]

            # Find if point is forward-most wall point on correct side 
            if (math.fabs(point_x) <= ROBOT_RADIUS) and (math.fabs(point_y) <= MAX_SIDE_LIMIT):
                if (self.follow_side == FollowSide.LEFT and point_y > 0) or (self.follow_side == FollowSide.RIGHT and point_y < 0):
                    # Point is between the front and back of robot and within range on correct side
                    if point_x > x_side_max:
                        x_side_max = point_x

            # Find if point is nearest wall point in front of the robot
            if point_x > 0 and point_x <= MAX_APPROACH_DIST and math.fabs(point_y) < ROBOT_RADIUS:
                # Point is between left and right sides of robot and within range from the front
                if point_x < x_front_min:
                    x_front_min = point_x

        # print "Detected walls {0} left, {1} front".format(x_side_max, x_front_min)

        turn = 0.0
        drive = 0.0
        if x_side_max == -np.inf:
            # No wall to side, so turn that way
            turn = 1.0
            drive = 0.0
        elif x_front_min <= MIN_APPROACH_DIST:
            # Wall to side and blocked at front, so turn opposite way
            turn = -1.0
            drive = 0.0
        else:
            # Wall to side but front is clear, so drive forward and start turning if 
            # an opening is starting to appear to side

            # If side wall doesn't extend past front of robot start turning and reduce speed
            turn_side = np.clip((ROBOT_RADIUS - x_side_max) / (2 * ROBOT_RADIUS), 0.0, 1.0)
            drive_side = np.clip((ROBOT_RADIUS + x_side_max) / (2 * ROBOT_RADIUS), 0.0, 1.0)

            # If front wall is between MAX_APPROACH_DIST and MIN_APPROACH_DIST then slow down and reduce turn
            turn_front = np.clip((MAX_APPROACH_DIST - x_front_min) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST), 0.0, 1.0)
            drive_front = np.clip((x_front_min - MIN_APPROACH_DIST) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST), 0.0, 1.0)

            # Combine two turn and drive decisions
            turn = turn_side - turn_front
            drive = drive_side * drive_front

        # Flip turn if we are following to the right
        if self.follow_side == FollowSide.RIGHT:
            turn *= -1.0

        # Convert to a Twist message for 'cmd_vel/'
        vel_msg = Twist()
        vel_msg.linear.x = drive * MAX_TRANS_SPEED
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = turn * MAX_TURN_SPEED
        # print "Publishing velocities {0} m/s, {1} r/s".format(vel_msg.linear.x, vel_msg.angular.z)
        self.publisher_twist.publish(vel_msg)
        self.stopped = False




    def callback_state(self, data):
        state = RobotState(data.data)

        if state is RobotState.EXPLORING and not self.explore:
            self.explore = True
            print 'Starting Exploring'
        elif (state is RobotState.RETURNING or state is RobotState.PAUSED) and self.explore:
            print 'Stopping Exploring'
            self.explore = False
            self.stopped = False
            # setting explore to false and stopped to false will cause the next laser scan
            # callback to send a stop command then set stopped to true.
            

    
if __name__ == '__main__':
    print "Starting ROS Wall Following module"
    rospy.init_node('wall_follower_node', anonymous=True)
    wf = wall_follower_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Wall Following module"
    wf.shutdown()