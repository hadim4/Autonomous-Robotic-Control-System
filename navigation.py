#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry


class WallFollow:
    def __init__(self):

        # Read the Wall-Following controller paramters from params.yaml
	lidarscan_topic = rospy.get_param('~scan_topic')
	odom_topic = rospy.get_param('~odom_topic')
	drive_topic = rospy.get_param('~nav_drive_topic')

        self.center_offset = rospy.get_param('~CenterOffset')
        self.distance_left = rospy.get_param('~DistanceLeft')
        self.distance_right = rospy.get_param('~DistanceRight')

        self.angle_bl = rospy.get_param('~angle_bl')
        self.angle_al = rospy.get_param('~angle_al')    
        self.angle_br = rospy.get_param('~angle_br')
        self.angle_ar = rospy.get_param('~angle_ar')

        self.n_pts_l = rospy.get_param('~n_pts_l')
        self.n_pts_r = rospy.get_param('~n_pts_r')

        self.tau = rospy.get_param('~tau')
        self.k_d = rospy.get_param('~k_d')
        self.k_p = rospy.get_param('~k_p')

        self.vehicle_velocity = rospy.get_param('~vehicle_velocity')
        self.turn_velocity = rospy.get_param('~turn_velocity')
        self.velocity_zero = rospy.get_param('~velocity_zero')
	
	self.base_frame = rospy.get_param('~base_frame')

	self.wheelbase = rospy.get_param('~wheelbase')
	self.max_steering_angle = rospy.get_param('~max_steering_angle')

	self.heading_beam = rospy.get_param('~heading_beam_angle')
	self.scan_beams = rospy.get_param('~scan_beams')

	self.stop_distance = rospy.get_param('~stop_distance')
	self.stop_distance_decay =rospy.get_param('~stop_distance_decay')

        # Subscrbie to LiDAR scan and Odometry topics
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        # Publisher for the Drive topic
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

	self.steering = 0.0
	self.vel = 0.0
        velocity = 0
    
    def lidar_callback(self, data):  

	simulation = 0

        #all angles
	angle_ar = self.angle_ar
        angle_al = self.angle_al
	angle_br = self.angle_br
        angle_bl = self.angle_bl
        
        theta_r = angle_ar - angle_br #theta_r
        theta_l = angle_bl - angle_al #theta_l
	
	#Computing distances
        indices_br = int((angle_br)/ data.angle_increment)
	br = data.ranges[indices_br]
	if np.isinf(br):
		br = 12 #12 = max lidar distance
	if np.isnan(br):
		br = 0 #garbage value
		       
        indices_bl = int((angle_bl)/ data.angle_increment)
        bl = data.ranges[indices_bl]
	if np.isinf(bl):
		bl = 12
	if np.isnan(bl):
		bl = 0

        indices_al = int((angle_al)/ data.angle_increment)
        al = data.ranges[indices_al]
	if np.isinf(al):
		al = 12
	if np.isnan(al):
		al = 0
	
        indices_ar = int((angle_ar)/ data.angle_increment)
	ar = data.ranges[indices_ar]
	if np.isinf(ar):
		ar = 12
	if np.isnan(ar):
		ar = 0
 	
	print(angle_ar, angle_al, angle_br, angle_al)
	print(data.angle_min)
	print(data.angle_increment)

	print(len(data.ranges))
	print(indices_br, indices_bl, indices_al, indices_ar)

	numerator_r = ar * math.cos(theta_r) - br #numerator beta_r
	denominator_r = ar * math.sin(theta_r) #denominator beta_r
        numerator_l = al * math.cos(theta_l) - bl #numerator beta_l
	denominator_l = al * math.sin(theta_l) #denominator beta_l

	#betal and betar
	beta_r = math.atan(numerator_r/denominator_r)
        beta_l = math.atan(numerator_l/denominator_l)

	print("\nBeta_R: ",beta_r,"\nBeta_L: ",beta_l)
	print("Theta_R: ",theta_r)
	print("Ar: ",ar)

	#distances to left and right wall
	dr = br*math.cos(beta_r)
        dl = bl*math.cos(beta_l)

	alpha_r = beta_r + math.pi/2 -angle_br #alpha_r
	alpha_l = -beta_l + 3*math.pi/2 - angle_bl #alpha_l

	tilde = (dl-dr) - self.center_offset
	dlr_dot = -self.vel*math.sin(alpha_l) - self.vel*math.sin(alpha_r)
	print("alpha_l: ",alpha_l,"\nalpha_r: ",alpha_r,"\ndlr_dot: ",dlr_dot)

	pd_controller = self.k_p*tilde + self.k_d*dlr_dot
	print("pd_controller: ",pd_controller)

	#if speed is greater than 0 because we cant have 0 denominator
	if self.vel >= 0.01 or self.vel <= -0.01:
		
		self.steering = math.atan( (pd_controller*(self.wheelbase)) / ( (self.vel**2) * (math.cos(alpha_l) + math.cos(alpha_r)) ))
		print("Steering: ",self.steering)
		print(self.vel**2 * (math.cos(alpha_l) + math.cos(alpha_r)))
		
		#if steering angle is greater/ less than 0.41, then it gets set to 0.41
		if self.steering >= self.max_steering_angle:
			self.steering = self.max_steering_angle
		elif self.steering <= -self.max_steering_angle:
			self.steering = -self.max_steering_angle

	#increments 11.25deg - will be used to look right and left of direct center of pi
 	increment = int(self.heading_beam/data.angle_increment)
	
	beams = self.scan_beams/2 #center
	self.steering = self.steering
	
	#finds closest distance point within left and right beam
	dob = min(data.ranges[-increment+(beams):increment+(beams)])

	#exponential in formula
	e = math.exp(-max(dob-self.stop_distance,0/self.stop_distance_decay))

	#nomial vel*(exp term)
        self.velocity = self.vehicle_velocity*(1-e)

      # Publish steering angle and velocity commnads to the Drive topic
	drive_msg = AckermannDriveStamped()

	drive_msg.header.stamp = rospy.Time.now()
	drive_msg.header.frame_id = self.base_frame
	drive_msg.drive.steering_angle = self.steering
        drive_msg.drive.speed = self.velocity

	print(self.steering)
	self.drive_pub.publish(drive_msg)

    
 # The Odometry callback reads the actual vehicle velocity from VESC. 
    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x
	

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
