#!/usr/bin/env python3

#Importing necessary Libraries
import rospy
import pyproj
import math
from tf import transformations
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
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

#origin
#latitude: 49.8999999955
#longitude: 8.8999999469

# latitude: 49.9000756035
# longitude: 8.90011774201


class autonomous:

    def __init__(self):
        self.gps = 0
        self.gps_latitude = 0
        self.gps_longitude = 0
        self.lat_final = 49.9000756035
        self.lng_final = 8.90011774201
        self.imu = 0
        self.yaw_initial = 0
        self.fw_azimuth = 0
        self.bw_azimuth = 0
        self.distance = 0
        self.flag = 0
        self.regions = {}
        self.us_left = 0
        self.us_right = 0
        self.rate = rospy.Rate(10)

    def main(self):
        
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        velocity = Twist()

        while not rospy.is_shutdown():

            self.sub_gps = rospy.Subscriber("/fix", NavSatFix, self.show_gps)
            self.rate.sleep()
            self.sub_imu = rospy.Subscriber("/imu", Imu, self.show_imu) 
            self.rate.sleep()
            self.sub_laser = rospy.Subscriber("/mybot/laser/scan", LaserScan, self.laser)
            self.rate.sleep()
            self.sub_us_left = rospy.Subscriber("sensor/us_left", Range, self.range_left)
            self.rate.sleep()
            self.sub_us_right = rospy.Subscriber("sensor/us_right", Range, self.range_right)
            self.rate.sleep()
            self.calc_bearing()
            # print("yaw:" + str(self.yaw_initial))
            print("fw azimuth:" + str(self.fw_azimuth))
            # print("distance:" + str(self.distance))
            print(self.regions, self.us_left, self.us_right)
          
            if self.distance > 0.8:

                if self.regions['front'] > 1.5 and self.regions['left'] > 1.5 and self.regions['right'] > 1.5:
                    print("1")
                
                    self.calc_bearing()
                    self.rotation(self.fw_azimuth)
                    while self.regions['front'] > 1.5 and self.regions['left'] > 1.5 and self.regions['right'] > 1.5 and self.distance > 0.8:
                        self.calc_bearing()
                        self.rotation(self.fw_azimuth)
                        self.linear()
            
                    self.stop()

                elif self.regions['front'] < 1.5 and self.regions['left'] > 1.5 and self.regions['right'] > 1.5:
                    print("2")
                    if self.regions['left']>=self.regions['right']:
                        self.rotation(self.yaw_initial-45) 
                        
                        while self.us_right < 3 and self.regions['front'] > 1.5:
                            self.linear()
                            
                        self.stop()
                            
                        if self.us_right > 2 and self.us_left > 2:
                            
                            self.calc_bearing()
                            self.rotation(self.fw_azimuth)
                            

                    else:
                        self.rotation(self.yaw_initial+45) 

                        while self.us_left < 3 and self.regions['front'] > 1.5:
                            self.linear()
                        
                        self.stop()

                        if self.us_right > 2 and self.us_left > 2:
                            self.calc_bearing()
                            self.rotation(self.fw_azimuth)
                           
                
                elif self.regions['front'] > 1.5 and self.regions['left'] > 1.5 and self.regions['right'] < 1.5:
                    print("3")
                    self.rotation(self.yaw_initial-45) 
                
                    while self.us_right < 3 and self.regions['front'] > 1.5:
                        self.linear()
                
                    self.stop()

                    if self.us_right > 2 and self.us_left > 2:
                        self.calc_bearing()
                        self.rotation(self.fw_azimuth)
                                
                elif self.regions['front'] > 1.5 and self.regions['left'] < 1.5 and self.regions['right'] > 1.5:
                    print("4")
                    self.rotation(self.yaw_initial+45)
                
                    while self.us_left < 3 and self.regions['front'] > 1.5:
                        self.linear()
                  
                    self.stop()

                    if self.us_right > 2 and self.us_left > 2:
                        self.calc_bearing()
                        self.rotation(self.fw_azimuth)
                  

                elif self.regions['front'] < 1.5 and self.regions['left'] > 1.5 and self.regions['right'] < 1.5:
                    print("5")
                    self.rotation(self.yaw_initial-45) 
                  
                    while self.us_right < 4 and self.regions['front'] > 1.5:
                        self.linear()
                        
                    self.stop()

                    if self.us_right > 2 and self.us_left > 2:
                        self.calc_bearing()
                        self.rotation(self.fw_azimuth)
                        
                elif self.regions['front'] < 1.5 and self.regions['left'] < 1.5 and self.regions['right'] > 1.5:
                    print("6")
                    self.rotation(self.yaw_initial+45) 
                    
                    while self.us_left < 3 and self.regions['front'] > 1.5:
                        self.linear()
                        
                    self.stop()

                    if self.us_right > 2 and self.us_left > 2:
                        self.calc_bearing()
                        self.rotation(self.fw_azimuth)
                        

                elif self.regions['front'] < 1.5 and self.regions['left'] < 1.5 and self.regions['right'] < 1.5:
                    print("7")
                    if self.regions['left']>=self.regions['right']:
                        self.rotation(self.yaw_initial-70) 
                        
                        while self.us_right < 3 and self.regions['front'] > 1.5:
                            self.linear()
                            
                        self.stop()

                        if self.us_right > 2 and self.us_left > 2:
                            self.calc_bearing()
                            self.rotation(self.fw_azimuth)
                            
                    else:
                        self.rotation(self.yaw_initial+70) 
                        
                        while self.us_left < 3 and self.regions['front'] > 1:
                            self.linear()
                            
                        self.stop()

                        if self.us_right > 2 and self.us_left > 2:
                            self.calc_bearing()
                            self.rotation(self.fw_azimuth)

                elif self.regions['front'] > 1.5 and self.regions['left'] < 1.5 and self.regions['right'] < 1.5:
                    print("8")
                    while self.us_left > 0.5 and self.us_right > 0.5 and self.regions['front'] > 1:
                        self.linear()
                                
                    self.stop()
                        
                    if self.us_right > 2 and self.us_left > 2:
                        self.calc_bearing()
                        self.rotation(self.fw_azimuth)
                        

                else:
                    print("9")
                    print(self.regions)
                   
            else:
                print("10")
                self.stop()
                

    def rotation(self, angle):
        
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        velocity = Twist()

        while not rospy.is_shutdown():

            #Rotation when the angle is greater than the yaw
            if(angle > self.yaw_initial):
                
                #To find shortest angle, check whether difference is greater than or less than 180
                if(angle - self.yaw_initial<180):

                    if(angle - self.yaw_initial > 1):
                    
                        velocity.angular.z = 0.5
                        pub.publish(velocity)

                    else:
                        break

                else:
                    self.yaw_initial = self.yaw_initial + 360
                    
                    if(self.yaw_initial - angle > 1):
                        velocity.angular.z = -0.5
                        pub.publish(velocity)

                    else:
                        break

            #Rotation when the yaw is greater than the angle
            elif(self.yaw_initial > angle):
            
                #To find shortest angle, check whether difference is greater than or less than 180
                if(self.yaw_initial - angle < 180):
                    
                    if(self.yaw_initial - angle > 1):
                        velocity.angular.z = -0.5
                        pub.publish(velocity)

                    else:
                        break

                else:

                    if (angle - self.yaw_initial > 1):
    
                        velocity.angular.z = 0.5
                        pub.publish(velocity)

                    else:
                        break

        velocity.angular.z = 0
        pub.publish(velocity)

    def linear(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        velocity = Twist()
        velocity.linear.x = 1
        pub.publish(velocity)

    def stop(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        velocity = Twist()

        velocity.linear.x = 0 
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x= 0
        velocity.angular.y = 0
        velocity.angular.z = 0
        pub.publish(velocity)
            
    def laser(self, msg):
        self.regions = { 'right':  min(min(msg.ranges[0:160]), 10), 'front':  min(min(msg.ranges[161:320]), 10), 'left':   min(min(msg.ranges[321:480]), 10) }

    def range_left(self, data):
        self.us_left = data.range

    def range_right(self, data):
        self.us_right = data.range
        
    def show_gps(self, coordinates):
        #Storing gps coordinates
        self.gps = coordinates
        self.gps_latitude = coordinates.latitude
        self.gps_longitude = coordinates.longitude

    def calc_bearing(self):
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
                 
if __name__ == "__main__":
    
    rospy.init_node("ATT", anonymous=True)
    obj = autonomous().main()
    rospy.spin()