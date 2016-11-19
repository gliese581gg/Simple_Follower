
import rospy
import numpy as np
import cv2
import time
import json
import requests
import tf.transformations
import subprocess as sp
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist, Vector3
import datetime
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import math
import chance_constrained_tracker as ct
from numpy import linalg as LA
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import proc_input
import signal
from detecting_msg.msg import *

class Follower(object):
    ROS_TIMEOUT = 300
    ENABLE = 1
    obj_path = '/home/biturtle/AUPAIR/datas/target_recog/target.txt'
    TRACK_SIGNAL = 1
    ACC = 0.2
    timeout = 2.0
    VMAX = 250.0
    VMIN = -250.0
    RANGE = 1.0 #m
    WMAX = np.pi * 140.0 / 180.0
    WMIN = np.pi * -140.0 / 180.0

    def __init__(self):
	print 'Initialize follower'
	self.proc = proc_input.proc_input('follower',False,True,False)        
	#self.cctt = ct.chance_constrained_tracker(['var',self.VAR,'sen_len',self.RANGE,'Wmax',self.WMAX,'Wmin',self.WMIN,'Vmax',self.VMAX,'Vmin',self.VMIN])
	self.track_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
	self.mode_pub = rospy.Publisher('/AUPAIR/followori', Int32, queue_size=1)
	self.sub_ = rospy.Subscriber('/AUPAIR/detection_target_without_ori', detection_array, self.callback_detections)
	self.sub2_ = rospy.Subscriber('/AUPAIR/followori', Int32, self.callback_mode)
	self.detection_filtered = []

    def callback_mode(self,data):
	self.ENABLE = data.data

    def callback_detections(self,data):
	self.detection_filtered = []
	for i in range(len(data.detections)):
		if data.detections[i].classes == 'person' :
			self.detection_filtered.append([data.detections[i].classes,data.detections[i].x1,data.detections[i].y1,data.detections[i].x2,data.detections[i].y2,data.detections[i].prob,data.header.stamp.to_sec()])


    def track(self):

	print 'Start tracking'
        self.position = np.array([0.0,0.0,0.0])
	self.position_old = np.array([0.0,0.0,0.0])
	self.positions =[]
	self.odom = np.array([0,0,0])
	self.pred_table = np.zeros((3,5))
	self.velocity = np.zeros(2)
	self.compress_rate = 1
	self.timestamp = 0
	#self.subs3 = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odom_callback, queue_size=1)	
	time.sleep(1)
	s = time.time()	
	ss = time.time()
	while time.time()-ss < self.ROS_TIMEOUT: 
		print self.ENABLE
		if self.ENABLE == 0 : continue
	
		if not len(self.detection_filtered) > 0 :
			time.sleep(0.1)
			continue

		s = time.time()
		if abs(float(s) - self.detection_filtered[0][6]) < self.timeout and len(self.detection_filtered) > 0: 
			self.set_velo()
		else :
			print ('File not updated for %.2f seconds' % self.timeout)
			self.track_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0))) 

		time.sleep(0.15)



	
	#self.terminate()
	#self.subs2.unregister()    
        

    def set_velo(self):
	    signal.signal(signal.SIGINT, terminate)
	    #self.position = np.array([0,0,0])
	    #chose target (nearest target from previous frame)
	    dist = 999999
	    
	    mx = (self.detection_filtered[0][3] + self.detection_filtered[0][1])/2
	    my = (self.detection_filtered[0][4] + self.detection_filtered[0][2])/2
	    minz = 999999.0
	    print mx, my
	    xy = self.proc.get_cloud_by_time(mx,my,self.detection_filtered[0][6])[1]
	    #print xy
	    for i in range(self.detection_filtered[0][1],self.detection_filtered[0][3],10):
	    	tt = self.proc.get_cloud_by_time(i,my,self.detection_filtered[0][6])[1]
		if tt[2] < minz :
			minz = tt[2]
			self.position[2] = float(tt[2])
	    self.position[0] = float(xy[0])
	    self.position[1] = float(xy[1])
	    #print self.position
	    if self.position[0] == 0 and self.position[1] == 0 : return

	    #print np.sum((self.position-self.position_old)*(self.position-self.position_old))

	    #Predict next position
	    #self.pred_table[0] = self.pred_table[1]
	    #self.pred_table[1] = self.pred_table[2]
	    #self.pred_table[2] = np.array([self.position[2]*1000,self.position[0]*1000,self.odom[0]*1000,self.odom[1]*1000,self.odom[2]])
	    next_pos = np.array([self.position[2] * 1000,self.position[0] * 1000])  
	    #self.velocity = np.array(self.cctt.track([next_pos[0],next_pos[1]]))
	    target_x = 0.0
	    target_z = self.RANGE
	    x_mult = 3.0
	    z_mult = -0.5
	    w_min = -0.4
	    w_max = 0.4
	    v_min = -0.25
	    v_max = 0.25
	    self.velocity[1] = max(min(w_max,(target_x - self.position[0]) * x_mult),w_min)
	    self.velocity[0] = max(min(v_max,(target_z - self.position[2]) * z_mult),v_min)
	    #self.velocity[0] = max(min((self.velocity[0] + self.ACC),vel_temp[0]),(self.velocity[0] - self.ACC))
	    #self.velocity[1] = max(min((self.velocity[1] + self.ACC),vel_temp[1]),(self.velocity[1] - self.ACC))
	    print "distance and velocities are calculated : "
	    print next_pos
	    print self.velocity
	    print ""
	    self.track_pub.publish(Twist(Vector3(self.velocity[0],0,0),Vector3(0,0,self.velocity[1])))
	    self.position_old = self.position.copy()
	    #time.sleep(0.2)

def terminate(what,ever):
	raise KeyboardInterrupt("CTRL-C!")

def main():
    #track_pub = rospy.Publisher('track_enable', Int32, queue_size=1)
    #track_pub.publish(2) #Tracking On
    #time.sleep(60)
    #track_pub.publish(0) #Tracking Off
    signal.signal(signal.SIGINT, terminate)
    #time.sleep(10)
    tracker = Follower()
    tracker.track()

if __name__ == '__main__':
    main()



