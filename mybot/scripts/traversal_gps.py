#!/usr/bin/env python3

import rospy
import pyproj
import math
from tf import transformations
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class traversal:    
    def __init__(self):
        # Variable Declaration
        self.name_space = rospy.get_namespace()
        self.lat_final = 49.900072523036265
        self.long_final = 8.899873641023234
        self.lat_curr = 0
        self.long_curr = 0
        self.yaw_initial = 0
        self.fw_azimuth = 0
        self.bw_azimuth = 0
        self.distance = 0
        self.error_yaw = 0
        self.dist_precision = 0.25
        self.yaw_precision = 4
        self.regions = {}
        self.velocity = Twist()
        self.rate = rospy.Rate(10)

        # Subscribers
        self.sub_gps = rospy.Subscriber("/fix", NavSatFix, self.gps_cb)
        self.sub_imu = rospy.Subscriber("/imu", Imu, self.imu_cb) 
        self.sub_laser = rospy.Subscriber("/LaserScan", LaserScan, self.laser_cb)

        # Publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.spin()

    # GPS callback, real-time GPS data update
    def gps_cb(self, msg):
        self.lat_curr = msg.latitude
        self.long_curr = msg.longitude
        g = pyproj.Geod(ellps='WGS84')
        (az12, az21, dist) = g.inv(self.long_curr, self.lat_curr, self.long_final, self.lat_final)
        self.fw_azimuth = az12
        self.bw_azimuth = az21
        self.distance = dist

    # IMU callback, real-time IMU data update
    def imu_cb(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_initial = euler[2]
        self.yaw_initial = -self.yaw_initial*(180/(math.pi))
    
    # LiDAR callback, real-time LiDAR data update
    def laser_cb(self, msg):
        self.regions = { 'right':  min(min(msg.ranges[0:5]), 1), 'front':  min(min(msg.ranges[6:10]), 1), 'left':   min(min(msg.ranges[11:15]), 1) }
        self.error_yaw = self.fw_azimuth - self.yaw_initial
        self.decision()

    # Check whether robot has reached destination or not, if not execute planner
    def decision(self):
        if self.distance<self.dist_precision:
            self.stop()
        elif self.distance>self.dist_precision:
            self.globalPlanner()
        else:
            rospy.loginfo("[%s]\033[0;33m Unknown Case\033[0m" %self.name_space)
    
        self.rate.sleep()

    # Planner
    def globalPlanner(self):
        # If obstacle is present, execute obstacle avoidance 
        if((abs(self.error_yaw)>self.yaw_precision) and (self.regions['front']<=0.7 or self.regions['right']<=0.7 or self.regions['left']<=0.7)):  
            rospy.loginfo("[%s]\033[0;31m Currently Stuck, Executing Obstacle Avoidance\033[0m" %self.name_space)
            self.obstacle_avoidance()
        
        # If no obstacles, but orientation is not towards goal, navigate to goal while correcting orientation  
        elif((abs(self.error_yaw)>self.yaw_precision) and ((self.regions['front']>0.7 and self.regions['right']>0.7 and self.regions['left']>0.7))):
            rospy.loginfo("[%s]\033[0;32m Global Planner\033[0m" %self.name_space)
            # Proportional Controller for Velocity
            self.velocity.linear.x = min(self.distance*0.12,0.8)
            self.velocity.angular.z = -1*min(self.error_yaw*0.01,1)
            self.pub.publish(self.velocity)

        # If there are no obstacles, and the robot is already oriented in the right direction, go straght    
        else:
            self.velocity.linear.x=0
            self.velocity.angular.z=0
            self.pub.publish(self.velocity)
            self.go_straight()

    def go_straight(self):
        self.velocity.linear.x = min(self.distance*0.12,0.8)
        self.velocity.angular.z=0
        self.pub.publish(self.velocity)

    # Obstacle Avoidance, calculate required velocity based on location of obstacle with respect to bot
    def obstacle_avoidance(self):  
        if self.regions['front']<=0.7 and self.regions['right']<=0.7 and self.regions['left']>0.7:
            self.velocity.linear.x = min(self.regions['right']*0.5, 0.5)
            self.velocity.angular.z = 1.0
            self.pub.publish(self.velocity)
        elif self.regions['front']<=0.7 and self.regions['right']>0.7 and self.regions['left']<=0.7:
            self.velocity.linear.x = min(self.regions['left']*0.3, 0.5)
            self.velocity.angular.z = -1.0
            self.pub.publish(self.velocity)
        elif self.regions['front']<=0.7 and self.regions['right']>0.7 and self.regions['left']>0.7:
            self.velocity.linear.x = min(self.regions['left']*0.3, 0.5)
            self.velocity.angular.z = 1.0
            self.pub.publish(self.velocity)
        elif self.regions['front']<=0.7 and self.regions['right']<=0.7 and self.regions['left']<=0.7:
            self.velocity.linear.x = -0.2
            self.velocity.angular.z = 1.0
            self.pub.publish(self.velocity)

    def stop(self):
        self.velocity.linear.x = 0 
        self.velocity.linear.y = 0
        self.velocity.linear.z = 0
        self.velocity.angular.x= 0
        self.velocity.angular.y = 0
        self.velocity.angular.z = 0
        self.pub.publish(self.velocity)

if __name__ == "__main__":
    rospy.init_node("Traversal", anonymous=True)
    obj = traversal()
    