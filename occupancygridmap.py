#!/usr/bin/env python

import numpy as np
import sys
import cv2
import time
import rospy
import tf2_ros
import math
import tf


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class OccupancyGridMap:
    def __init__(self):

        # Read paramters from params.yaml
        lidarscan_topic = rospy.get_param('~scan_topic')
        odom_topic = rospy.get_param('~odom_topic') # odom or odom_imu imu

        self.t_prev=rospy.get_time()
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.scan_beams=rospy.get_param('~scan_beams')
        
        # Read the map parameters from .yaml file
        self.occ_map_topic = rospy.get_param('~occ_map_topic')
        self.p_occ = rospy.get_param('~p_occ')
        self.p_free = rospy.get_param('~p_free')
        self.map_width = rospy.get_param('~map_width')
        self.map_height = rospy.get_param('~map_height')
        self.map_res = rospy.get_param('~map_res')
        self.object_size = rospy.get_param('~object_size')
        
        self.map_occ_grid_msg = OccupancyGrid()

        # Initialize the map meta info in the Occupancy Grid Message
        self.map_occ_grid_msg.header.frame_id = "odom"
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()
        self.map_occ_grid_msg.info.resolution = self.map_res
        self.map_occ_grid_msg.info.width = self.map_width
        self.map_occ_grid_msg.info.height = self.map_height
			
	# X and Y coordinates of cells
        self.map_origin_x = -(self.map_width * self.map_res)/ 2.0
        self.map_origin_y = -(self.map_height * self.map_res)/ 2.0
 
        self.map_occ_grid_msg.info.origin.position.x = self.map_origin_x
        self.map_occ_grid_msg.info.origin.position.y = self.map_origin_y
        self.map_occ_grid_msg.info.origin.orientation.w = 1.0
	
	
        # Initialize the cell probabilites to 0.5 (unknown) with all cell data in Occupancy Grid Message set to unknown 
        self.l_map = np.zeros(self.map_width * self.map_height) # sets all the values to unknown initially
        self.map_occ_grid_msg.data = [-1] * (self.map_width * self.map_height)

        js = np.arange(self.map_width)
        is_ = np.arange(self.map_height)
        jj, ii = np.meshgrid(js, is_)  # shape (H, W)
        self.cell_xs = (self.map_origin_x + self.map_res * (jj + 0.5)).ravel()
        self.cell_ys = (self.map_origin_y + self.map_res * (ii + 0.5)).ravel()

        # Subscribe to Lidar scan and odomery topics
        self.scan_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size = 1)


        # Create a publisher for the Occupancy Grid Map
        self.map_pub = rospy.Publisher(self.occ_map_topic, OccupancyGrid, queue_size=1)
	
	# AEV initial states
        self.x_rpos = 0.0
        self.y_rpos = 0.0
        self.yaw = 0.0
		
    	# Probability Constant Calculatiions
        self.l_free = math.log(self.p_free / (1-self.p_free))
        self.l_occ = math.log(self.p_occ / (1-self.p_occ))
        self.l_prior = 0


        self.l_max_clamp = 5.0
        self.l_min_clamp = -5.0

        self.scan_offset = rospy.get_param('~scan_distance_to_base_link')


# Uses the current LiDAR scan and Wheel Odometry data to publish to the Grid Occupancy map     
    def lidar_callback(self, data):      

    	# Current position of Robot
        current_x_rpos = self.x_rpos
        current_y_rpos = self.y_rpos
    	
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        # Transforming Lidar to Odom frame
        lidar_x = current_x_rpos + self.scan_offset * cos_yaw
        lidar_y = current_y_rpos + self.scan_offset * sin_yaw
        lidar_yaw = self.yaw - math.pi

        dx = self.cell_xs - lidar_x 
        dy = self.cell_ys - lidar_y 
        
        cos_lidar_yaw = math.cos(lidar_yaw)       
        sin_lidar_yaw = math.sin(lidar_yaw)      
   
        # Transform into  frame
        x_rel = (dx * cos_lidar_yaw) + (dy * sin_lidar_yaw)  # x distance from cell to Lidar
        y_rel = (-dx * sin_lidar_yaw) + (dy * cos_lidar_yaw) # y distance from cell to Lidar

      
        # Angle and Distance 
        cell_angle = np.arctan2(y_rel, x_rel) # atan2 looks at the sign of variables whereas arctan doesn't
        cell_angle = cell_angle % (2.0 * math.pi) # normalization
        cell_distance = np.sqrt(x_rel**2 + y_rel**2) # distance from center of cell to robot
        
        # Each cell angle gets an index
        beam_indices = ( (cell_angle - data.angle_min) / data.angle_increment).astype(int)

        valid_beam = (beam_indices >= 0) & (beam_indices < len(data.ranges))

        # Ranges now in a numpy array for easy look up
        ranges = np.array(data.ranges, dtype=np.float32)
	
	# Looks up measured distance from ranges measured if the cell has a valid beam
        lidar_distances = np.where(valid_beam, ranges[np.clip(beam_indices, 0, len(data.ranges) - 1)], np.nan) 

        valid_range = valid_beam & np.isfinite(lidar_distances) & (lidar_distances <= self.max_lidar_range)
 
 
        # if free
        # dividing the resolution size by 2 to know if we are within half of the object range
        is_free = valid_range & (lidar_distances > (cell_distance + (self.map_res / 2.0) ))
 
        # if Occupied
        is_occ = valid_range & (np.abs(cell_distance - lidar_distances) <= (self.map_res / 2.0) )
 
        # log-odds 
        self.l_map[is_free] += self.l_free - self.l_prior
        self.l_map[is_occ] += self.l_occ - self.l_prior
        
        np.clip(self.l_map,self.l_min_clamp,self.l_max_clamp, out=self.l_map)
         
        # log-odds to probabilities and Rviz naps 
        probability = 1.0 - 1.0 / (1.0 + np.exp(self.l_map))
 
        occupancy_data = np.full(len(self.l_map), -1, dtype=np.int8)
        occupancy_data[probability > self.p_occ] = 100    # occupied
        occupancy_data[probability < self.p_free] = 0     # free
 
        self.map_occ_grid_msg.data = occupancy_data.tolist()

        # Publish to map topic
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map_occ_grid_msg)
        
	
	

# Retrives the wheel odometry data from the publsihed odom_msg
     
    def odom_callback(self, odom_msg):

        self.x_rpos = odom_msg.pose.pose.position.x # AEV x
        self.y_rpos = odom_msg.pose.pose.position.y # AEV Y
	
	# Process of converting quaternion to yaw to get heading of AEV
        quat = odom_msg.pose.pose.orientation
        orientation = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation)   
    	
        self.yaw = yaw # AEV yaw
	

def main(args):
    rospy.init_node("occupancygridmap", anonymous=True)
    OccupancyGridMap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)



'''
will be deleted for loop implementation
def lidar_callback(self, data):

'''
