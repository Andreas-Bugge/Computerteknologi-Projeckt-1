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
STOP_DISTANCE = 0.15
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
TURNING_DISTANCE = 0.35 + LIDAR_ERROR
COLLISION_DISTANCE = 0.1

# Class for RGB sensor.
class RGB():
    # Used for not scanning the same victim multiple times.
    RGB_cooldown = 1
    
    def __init__(self):
        # Get I2C bus
        self.bus = smbus2.SMBus(1)

        time.sleep(0.5)

        # ISL29125 address, 0x44(68) (Standarden)
        # Select configuation-1 register: 0x01(01)
        # 0b00101(5) Operation: RGB, Range: 375 lux, Res: 16 Bits
        self.bus.write_byte_data(0x44, 0x01, 0x05)
        self.initial_values = self.Colour() 

    def Colour(self):
        # Selects the right registers
        data = self.bus.read_i2c_block_data(0x44, 0x09, 6)
        green = data[1] 
        red = data[3] 
        blue = data[5] 
        return green, red, blue
    
    def getVictim(self):
        lightdata = self.Colour()

        print('RED DATA:', lightdata[1])
        print('Initial values: ', self.initial_values[1])
        if RGB.RGB_cooldown < 1:
            if lightdata[1] - self.initial_values[1] >= 15:
                RGB.RGB_cooldown = 7
                return True
        else:
            RGB.RGB_cooldown -= 1
            return False

# Class for obstacle dection.
class Obstacle():  
    # Collision cooldown, so it doesn't count multiple collisions in one collision. 
    Collision_cooldown = 0
    Collision_counter = 0

    # Setting up interactions with the ROS-system.
    def __init__(self):
        self.RGBsensor = RGB()
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
        
        # Makes different cone arrays and gets the minimum value from each cone
        Full_scan = min(scan_filter)

        Front_cone = min(scan_filter[0:15] + scan_filter[344:359])
        Backing_cone = min(scan_filter[170:190])
        Front_Left_cone = min(scan_filter[16:30])
        Front_Right_cone = min(scan_filter[329:344])
        Left_cone = min(scan_filter[30:70])
        Right_cone = min(scan_filter[290:330])

        return Front_cone, Backing_cone, Front_Left_cone, Front_Right_cone, Left_cone, Right_cone, Full_scan
    
    # Function for measuring collisions
    def get_collision(self):
        _, _, _, _, _, _, Full_scan = self.get_scan()

        if Full_scan < COLLISION_DISTANCE and Obstacle.Collision_cooldown < 1:
            Obstacle.Collision_counter += 1
            Obstacle.Collision_cooldown = 5
            rospy.loginfo('Collision detected! %f', Obstacle.Collision_counter)
        else:
            Obstacle.Collision_cooldown -= 1


    def obstacle(self):
        # Initial stats for the program
        twist = Twist()
        turtlebot_moving = True
        Victims_found = 0
        angular_vel = 0.0

        # Speed stats
        Accumulated_speed = 0
        Speed_updates = 0

        # Runtime / Time settings
        Run_time = 30 * 2
        End_run = time.time() + Run_time

        # Sensor data
        Data = RGB()
        
        # ALGORITMS FOR TURTLEBOT MOVEMENTS
        # Turtlebot is moving forward
        def Forward (speed):
            twist.linear.x = speed
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            rospy.loginfo('Moving forward! %f', Front_cone)

        # Turning turtlebot to the right
        def Turn_right (speed, turn):   
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
            twist.linear.x = -speed
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
            Front_cone, Backing_cone, Front_Left_cone, Front_Right_cone, Left_cone, Right_cone, _ = self.get_scan()
            
            # Checking for collisions 
            self.get_collision()

            # Checking for victims
            if(Data.getVictim()):
                Victims_found += 1
                rospy.loginfo('New victim found, total victims: %i', Victims_found) 

            # If turtlebot is moving
            if turtlebot_moving:

                # Close to collision 
                if Front_cone < SAFE_STOP_DISTANCE:
                    # Backing up turtlebot for one second
                    Backing(-1 * LINEAR_VEL)
                    # Stop the turtlebot
                    turtlebot_moving = False  
                    rospy.loginfo('---1---') 

                # If it is possible to make a turn:
                # Soft right turn
                elif Front_cone < TURNING_DISTANCE:
                    if Front_Left_cone < Front_Right_cone:
                        Turn_right(LINEAR_VEL, 1) 
                        rospy.loginfo('---2---') 

                    # Soft left turn
                    else:
                        Turn_left(LINEAR_VEL, 1)
                        rospy.loginfo('---3---')

                    # Hard right/left turns if necessary
                    if Front_cone < TURNING_DISTANCE / 1.15:
                        if Front_Left_cone < Front_Right_cone:
                            Turn_right(0.5 * LINEAR_VEL, 1) 
                            rospy.loginfo('---4---') 
 
                        else:
                            Turn_left(0.5 * LINEAR_VEL, 1)
                            rospy.loginfo('---5---')
                    
                    if Front_cone < TURNING_DISTANCE / 1.3:
                        if Front_Left_cone < Front_Right_cone:
                            Turn_right(0.25 * LINEAR_VEL, 1) 
                            rospy.loginfo('---4---') 
 
                        else:
                            Turn_left(0.25 * LINEAR_VEL, 1)
                            rospy.loginfo('---5---')

             
                # If there is a clear way ahead, turtlebot will move forward.
                else:
                    Forward(LINEAR_VEL)
                    rospy.loginfo('---6---')
                    
                    if Left_cone < SAFE_STOP_DISTANCE:
                        angular_vel = LINEAR_VEL * (TURNING_DISTANCE - Front_cone) / TURNING_DISTANCE
                        Turn_right(LINEAR_VEL, 1)
                        rospy.loginfo('---8---')

                    elif Right_cone < SAFE_STOP_DISTANCE:
                        angular_vel = LINEAR_VEL * (TURNING_DISTANCE - Front_cone) / TURNING_DISTANCE
                        Turn_left(LINEAR_VEL, 1)
                        rospy.loginfo('---9---')
                    
                   
            # If turtlebot is NOT moving
            else: 
                if Front_cone < SAFE_STOP_DISTANCE and Left_cone < SAFE_STOP_DISTANCE and Right_cone < SAFE_STOP_DISTANCE:
                # Move backward until there is a clear path
                    while Front_cone < SAFE_STOP_DISTANCE:
                        Backing(-LINEAR_VEL)
                        Front_cone, _, _, _, _, _, _ = self.get_scan()
                        rospy.loginfo('---8---')
                    
                    turtlebot_moving = True


                # Deciding to turn around itself 
                if Front_cone < SAFE_STOP_DISTANCE:
                    Turn_factor = 0
                    first_scan_value = False

                    # Deciding which direction to turn around itself
                    if not first_scan_value:
                        Sum_right = Front_Right_cone + Right_cone
                        Sum_left = Front_Left_cone + Left_cone

                        if Sum_left < Sum_right:
                                Turn_factor = -1 # Turn right
                                rospy.loginfo('---11---')
                        else:
                                Turn_factor = 1 # Turn left
                                rospy.loginfo('---12---')

                        first_scan_value = True

                    # Turning untill clear way ahead
                    if first_scan_value:
                        
                        while Front_cone < SAFE_STOP_DISTANCE * 1.5:
                            Turning_around(Turn_factor)
                            Front_cone, _, _, _, _, _, _ = self.get_scan()
                            rospy.loginfo('---13---')

                        # After while loop start moving forward again
                        turtlebot_moving = True
                        rospy.loginfo('---14---')
                        
                # All else equal: if there is a clear way ahead we move forward.
                else:
                    Forward(LINEAR_VEL)
                    turtlebot_moving = True
                    rospy.loginfo('---15---')

            # Update stats
            Accumulated_speed += abs(twist.linear.x)
            Speed_updates += 1            

        # Print stats after run
        rospy.loginfo('Total victims found: %f', Victims_found)
        rospy.loginfo('Average speed: %f', Accumulated_speed / Speed_updates)
        rospy.loginfo('Total collisions: %f', Obstacle.Collision_counter)
        rospy.loginfo('Total run time: %f', Run_time)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()