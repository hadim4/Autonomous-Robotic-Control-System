#!/usr/bin/env python
from __future__ import print_function 
from lib2to3.pytree import Node
from  numpy import array, dot
from quadprog import solve_qp

import sys
import math
from tokenize import Double
import numpy as np
import time


import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GapBarrier:
    def __init__(self):

        # Read the algorithm paramters from params.yaml
	lidarscan_topic = rospy.get_param('~scan_topic')
	odom_topic = rospy.get_param('~odom_topic')
	drive_topic = rospy.get_param('~nav_drive_topic')

	self.wheelbase = rospy.get_param('~wheelbase')

	self.n_pts_l = rospy.get_param('~n_pts_l')
        self.n_pts_r = rospy.get_param('~n_pts_r')

        self.tau = rospy.get_param('~tau')
        self.k_d = rospy.get_param('~k_d')
        self.k_p = rospy.get_param('~k_p')

        self.vehicle_velocity = rospy.get_param('~vehicle_velocity')
	self.max_steering_angle = rospy.get_param('~max_steering_angle')

	self.heading_beam = rospy.get_param('~heading_beam_angle')
	self.scan_beams = rospy.get_param('~scan_beams')

	self.stop_distance = rospy.get_param('~stop_distance')
	self.stop_distance_decay =rospy.get_param('~stop_distance_decay') 
        self.safe_distance = rospy.get_param('~safe_distance')
        self.center_offset = rospy.get_param('~CenterOffset')
	self.base_frame = rospy.get_param('~base_frame')
	self.fov = 0

        # Subscribing to lidar and odom topics
	rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        # Publisher for Drive topic 
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

	self.vel = 0.0
	self.steering = 0.0
	velocity = 0.0

        self.fov_range = []
	self.start = 0
	self.end = 0

	self.base_frame = rospy.get_param('~base_frame')


# Get lidar points within 70 FOV infront of AEV
    def preprocess_lidar(self, data):
        

	self.fov = 70*math.pi/180 # 70deg into rad
	center = self.scan_beams/2

	# 35deg increment left and right = 70deg fov total
	increment = int((self.fov/2)/(data.angle_increment)) 
	self.start = (center) - increment	
        self.end = (center) + increment

	self.fov_range = data.ranges[self.start:self.end]
                     

# Find maximum gap of non-zeros
    def find_max_gap(self, proc_ranges):
	
	# Makes closer obstacles equal to 0
	for i in range(len(self.fov_range)):
		if self.fov_range[i] < self.safe_distance:
			self.fov_range[i] = 0

	count1 = 0
	count2 = 0
	max_gap =[]
	nonZero_indicies = []

	for i in self.fov_range:
		count1 =+ 1

		# Makes greatest len list equal to max_gap
		if i != 0:
			count2 =+ 1
			nonZero_indicies.append(i)
			if len(nonZero_indicies) > len(max_gap):
				max_gap = nonZero_indicies

		# Resets non-zero list when a 0 appears
		else:
			nonZero_indicies.clear()

		return max_range

			
# Find desired direction of travel within max gap
    def find_best_point(self, start_i, end_i, proc_ranges):
	count = 0
	for i in max_gap:
		count =+ 1

		# Sum of all distances within max_gap
		distances += i
		desAngle_denominator = distances

		# Sum of all angles within max_gap
		angles = sum(data.angle_increment*i)
		desAngle_numerator = angles*distances

	desAngle = desAngle_numerator/desAngle_denominator
	return desAngle


    #def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, alpha):
         #pass
     
    

# Called whenever a new set of LiDAR data is received; bulk of our controller implementation 
    def lidar_callback(self, data):      

        GapBarrier.preprocess_lidar(self, data)

        # Setting up the QP for finding the two parallel barrier lines
        G = np.array([[1.0, 0.0,0.0],[0.0,0.1,0.0],[0.0,0.0,0.0001]]) #G matrix

	a = np.zeros(3) #a matrix

	b = np.ones(self.n_pts_l + self.n_pts_l)
	b = np.append(b, [-0.99, -0.99]) #b matrix
        print("B: ",len(b))

	# Computing C matrix
	pts_l = []
	pts_r = []
	count = 0
	for i in self.fov_range:
		distances = i
		index =  self.start + count
		angle1 = (index* data.angle_increment) + data.angle_min
		count +=1
		px = distances*math.cos(angle1) # x part of matrix
		py = distances*math.sin(angle1) # y part of matrix

		# Ranges on the right
		if angle1 > math.pi:
			pts_r.append([px,py,1.0])
		
		# Ranges on the left		
		if angle1 < math.pi:
			pts_l.append([-px,-py,-1.0])
		
			
	print("angle=", angle1)
	print("increment=", data.angle_increment)
	print("distances=", i)
	

		
	total_pts = pts_r + pts_l

	C = np.array(total_pts).T
        C_column = np.array([[0.0, 0.0], [0.0, 0.0], [1.0,-1.0]])
        C = np.hstack((C, C_column)) #C matrix

		

	print("C0:", C[0], "\nC1:",C[1], "\nC2:", C[2])	
	

        # Solving the QP problem to find the barrier line parameters 
	meq = 0
	x = solve_qp(G, a, C, b, meq)[0]
	w = np.array([x[0], x[1]])
	s = x[2]
	print("W Vector: ",w)
	print("S: ",s)
	

        # Computing the values of the variables needed for the implementation of feedback linearizing+PD controller
	wr = w / (s-1) #wr
	wl = w / (s+1) #wl
	print("wr0: ",wr, "\nwl0: ",wl)

	norm_l = np.dot(wl.T, wl) #norm_l
	norm_r = np.dot(wr.T, wr) #norm_r
	print("norm_l: ",norm_l,"\nnorm_r: ",norm_r)
	
	#distances to left and right	
	dl = 1/math.sqrt(norm_l)
	dr = 1/math.sqrt(norm_r)
	print("Dl:", dl, "\nDr", dr)
	
	#error between dl and dr
	dlr_error = (dl - dr)

	wl_hat = dl*wl 
	wr_hat = dr*wr 

	vs = np.array([self.vel, 0])
	
	dl_dot = np.dot(np.array([self.vel, 0]), wl_hat)
	dr_dot = np.dot(np.array([self.vel, 0]), wr_hat)

	dl_dot_error = dl_dot - dr_dot
	tilde = dlr_error - self.center_offset

	cos_alpha_l = np.dot(np.array([0, -1]), wl_hat)
	cos_alpha_r = np.dot(np.array([0, 1]), wr_hat)
	print("Cos_alpha_l: ",cos_alpha_l,"\nCos_alpha_r: ",cos_alpha_r)
	print("wl_hat: ", wl_hat,"\nwr_hat: ", wr_hat)

	controller = self.k_p*tilde + self.k_d*dl_dot_error
	
        # Compute the steering angle command
	self.steering = math.atan( (controller*self.wheelbase) / ( (self.vel**2) * (cos_alpha_l + cos_alpha_r) ))

	if self.steering >= self.max_steering_angle:
		self.steering = self.max_steering_angle
	elif self.steering <= -self.max_steering_angle:
		self.steering = -self.max_steering_angle

	print("Steering: ",self.steering)
        
        # Closest obstacle in a narrow field of view in front of the vehicle
        center = self.scan_beams/2
	increment = int(self.heading_beam/data.angle_increment)  
	d_ob = min(data.ranges[center-increment:center+increment])

        # Computing velocity
	e = math.exp(-max(d_ob-self.stop_distance, 0)/self.stop_distance_decay)
	self.velocity = self.vehicle_velocity*(1-e)

        # Publish the steering and speed commands to the drive topic
	drive_msg = AckermannDriveStamped()

	drive_msg.header.stamp = rospy.Time.now()
	drive_msg.header.frame_id = self.base_frame
	drive_msg.drive.steering_angle = self.steering
        drive_msg.drive.speed = self.velocity

	print(self.steering)
	self.drive_pub.publish(drive_msg)

    # Odometry callback 
    def odom_callback(self, odom_msg):
        # update current speed
         self.vel = odom_msg.twist.twist.linear.x


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
