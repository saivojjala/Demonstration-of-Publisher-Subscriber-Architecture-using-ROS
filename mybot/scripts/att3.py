#!/usr/bin/env python3

#Importing necessary Libraries
import rospy
import pyproj
import math
from tf import transformations
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

#q1
# latitude: 49.9000373192
# longitude: 8.90005363405

#q2
# latitude: 49.9000365828
# longitude: 8.89994591423

# q3
# latitude: 49.8999640883
# longitude: 8.89994364338

#q4
# latitude: 49.8999623552
# longitude: 8.90005744686

class autonomous:

    def __init__(self):
        self.gps = 0
        self.gps_latitude = 0
        self.gps_longitude = 0
        self.lat_final = 49.90007000174165
        self.lng_final = 8.899886718601824
        self.imu = 0
        self.yaw_initial = 0
        self.fw_azimuth = 0
        self.bw_azimuth = 0
        self.distance = 0
        self.flag = 0
        self.rate = rospy.Rate(5)

    def main(self):
        #Settng flags, to call approriate function
        if self.flag == 0:
            self.rotation()
        if self.flag ==1:
            self.linear()

    def show_gps(self, coordinates):
        #Storing gps coordinates
        self.gps = coordinates
        self.gps_latitude = coordinates.latitude
        self.gps_longitude = coordinates.longitude

        #Using pyproj to calculate the azimuth and the distance
        g = pyproj.Geod(ellps='WGS84')
        (az12, az21, dist) = g.inv(self.gps_longitude, self.gps_latitude, self.lng_final, self.lat_final)
        self.fw_azimuth = az12
        self.bw_azimuth = az21
        self.distance = dist

        #fw_azimuth to 0-360 range
        if self.fw_azimuth < 0:
            self.fw_azimuth = self.fw_azimuth + 360
        else:
            self.fw_azimuth = self.fw_azimuth
        
    def show_imu(self, angles):
        self.imu = angles
        quaternion = (self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)

        #Converting from quaternion to euler
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_initial = euler[2]
        self.yaw_initial = self.yaw_initial*(180/(math.pi))

        #Changing yaw sign to match the convention associated with GPS

        self.yaw_initial = -self.yaw_initial

        #Convert yaw to 0-360 range
        if self.yaw_initial < 0:
            self.yaw_initial = self.yaw_initial + 360
        else:
            self.yaw_initial = self.yaw_initial

    def rotation(self):
        
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        velocity = Twist()

        while not rospy.is_shutdown():

            self.sub_gps = rospy.Subscriber("/fix", NavSatFix, self.show_gps)
            self.rate.sleep()
            self.sub_imu = rospy.Subscriber("/imu", Imu, self.show_imu) 
            self.rate.sleep()
            print(self.yaw_initial)
            print(self.fw_azimuth)

            #Rotation when the azimuth is greater than the yaw
            if(self.fw_azimuth > self.yaw_initial):
                
                #To find shortest angle, check whether difference is greater than or less than 180
                if(self.fw_azimuth - self.yaw_initial<180):

                    if(self.fw_azimuth - self.yaw_initial > 0.9):
                    
                        velocity.angular.z = 0.8
                        pub.publish(velocity)

                    else:
                        break

                else:
                    self.yaw_initial = self.yaw_initial + 360
                    
                    if(self.yaw_initial - self.fw_azimuth > 0.9):
                        velocity.angular.z = -0.8
                        pub.publish(velocity)

                    else:
                        break

            #Rotation when the yaw is greater than the azimuth
            elif(self.yaw_initial > self.fw_azimuth):
            
                #To find shortest angle, check whether difference is greater than or less than 180
                if(self.yaw_initial - self.fw_azimuth < 180):
                    
                    if(self.yaw_initial - self.fw_azimuth > 0.9):
                        velocity.angular.z = -0.8
                        pub.publish(velocity)

                    else:
                        break

                else:

                    if (self.fw_azimuth - self.yaw_initial > 0.9):
    
                        velocity.angular.z = 0.8
                        pub.publish(velocity)

                    else:
                        break

        velocity.angular.z = 0
        pub.publish(velocity)
        self.flag = 1

    def linear(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        velocity = Twist()

        while not rospy.is_shutdown():
            print(self.distance)

            if self.distance > 1:
                velocity.linear.x = 1
                pub.publish(velocity)
            
            else:
                break
        
        velocity.linear.x = 0
        pub.publish(velocity)
                  
if __name__ == "__main__":
    
    rospy.init_node("ATT", anonymous=True)
    obj = autonomous().main()
    rospy.spin()