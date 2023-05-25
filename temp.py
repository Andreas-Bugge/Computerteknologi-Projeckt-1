# If there less then 35 cm, we turn around 
#               if Front_cone < COLLISION_DISTANCE * 1.6 :
#                    # If turn_time counter is above 0, turtlebot will turn around in a circle
#                    if Turn_time > 0:
#                        if Front_Left_cone < Front_Right_cone:
#                           if Left_cone < Right_cone:
#                               Turn_factor = 1
#                               rospy.loginfo('---12---')
#                        else:
#                            if Left_cone > Right_cone:
#                               Turn_factor = -1
#                                rospy.loginfo('---13---')

#                    # Turn timer initiated
#                    Turn_time -= 1
#                    turtlebot_moving = False
#                    rospy.loginfo('Turn time counter: %f', Turn_time)
#                    # Turning around itself in given direction from scans
#                    Turning_around(Turn_factor)



#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #
# Authors: Andreas Bugge & Mathies Schou #

# Importing the necessary libraries
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import smbus2
import time

# Global Variables
LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
TURNING_DISTANCE = 0.45
COLLISION_DISTANCE = 0.2
BACK_VAL = 0.15

# Class for obstacle dection.
class Obstacle():

    # Setting up interactions with the ROS-system.
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()

    # Function for setting up a scan filter array that gets us information from the lidar.  
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        scan_filter = list(scan.ranges[0:359])
        # Sorts out all invalid, NaN and zeros from our scan_filter       
        for i in range(len(scan_filter)):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 20.0
            if math.isnan(scan_filter[i]):
                scan_filter[i] = 20.0
            elif scan_filter[i] == 0:
                scan_filter[i] = 20.0
        
        # Makes the front cones that the turtlebot uses detection indicators
        Left_front_cone = scan_filter[0:15]
        Right_front_cone = scan_filter[345:359]
        Front_cone = Left_front_cone + Right_front_cone
        Front_cone = min(Front_cone)

        Front_Left_cone = min(scan_filter[16:36])
        Front_Right_cone = min(scan_filter[324:344])
        Backing_cone = min(scan_filter[170:190])

        Left_cone = min(scan_filter[37:80])
        Right_cone= min(scan_filter[280:323])

        return Front_cone, Front_Left_cone, Front_Right_cone, Backing_cone, Left_cone, Right_cone

    def obstacle(self):

        # Initial stats for the program
        twist = Twist()
        turtlebot_moving = True
        Victims_found = 0
        Collision_counter = 0

        # Speed stats
        Accumulated_speed = 0
        Speed_updates = 0

        # Runtime settings
        Run_time = 60 * 2
        End_run = time.time() + Run_time
    
        # ALGORITMS FOR TURTLEBOT MOVEMENTS
        # Turtlebot is moving forward
        def Forward (speed):
            twist.linear.x = speed
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            rospy.loginfo('Moving forward! %f', Front_cone)

        # Turning turtlebot to the right
        def Turn_right (speed,turn):   
            twist.linear.x = speed
            twist.angular.z = -turn
            self._cmd_pub.publish(twist)
            rospy.loginfo('Turning to right! %f', Front_Right_cone)

        # Turning turtlebot to the left
        def Turn_left (speed, turn):
            twist.linear.x = speed
            twist.angular.z = turn
            self._cmd_pub.publish(twist)
            rospy.loginfo('Turning to left! %f', Front_Left_cone)

        # Turtlebot is backing up
        def Backing (speed):   
            twist.linear.x = speed
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            rospy.loginfo('Backing! %f', Backing_cone)

        # Turning around in case of encirclement
        def Turning_around (turn):
            twist.linear.x = 0.0
            twist.angular.z = turn
            self._cmd_pub.publish(twist)
            rospy.loginfo('Turning around!')  

        # Control loop for the program
        while not rospy.is_shutdown() and time.time() < End_run:
            Front_cone, Front_Left_cone, Front_Right_cone, Backing_cone, Left_cone, Right_cone = self.get_scan()
            
            # If turtlebot is moving forward og turning
            if turtlebot_moving:
                if Front_cone < TURNING_DISTANCE and Front_cone > SAFE_STOP_DISTANCE:
                    if Front_Left_cone < Front_Right_cone:
                        while Front_Right_cone > Front_Right_cone:
                            Turn_right(0.18, 0.95)
                    elif
            else:
                
                

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()



                        if Front_cone < TURNING_DISTANCE / 1.3:
                        if Front_Left_cone < Front_Right_cone:
                            Turn_right(0.25 * LINEAR_VEL, 1) 
                            rospy.loginfo('---4---') 
 
                        else:
                            Turn_left(0.25 * LINEAR_VEL, 1)
                            rospy.loginfo('---5---')




    