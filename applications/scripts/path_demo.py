#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
import math

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class NavPath(object):
    def __init__(self):
	self.lastPos = None
        #self.lastPos.x = 0
        #self.lastPos.y = 0
        #self.lastPos.z = 0
        self._path = []
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
            
    def callback(self, msg):
        #rospy.loginfo(msg)
        curr_pose = msg.pose.pose
        curr_point = msg.pose.pose.position
        if not self.lastPos or self.distance(self.lastPos, curr_point) > 0.1:
		self.lastPos = curr_point
		self._path.append(self.lastPos)
                print(self._path)
		marker = Marker(type = Marker.LINE_STRIP, 
			scale= Vector3(0.06, 0.06, 0.06), 
			color = ColorRGBA(0.0, 1.0, 0.0, 0.8), 
			header=Header(frame_id='odom'),
		        points=self._path)
		print("published marker")
		self.marker_publisher.publish(marker)
            	#self._path.append(msg.foo.bar)
            

    def distance(self, pt1, pt2):
      xDist = pt1.x - pt2.x
      yDist = pt1.y - pt2.y
      zDist = pt1.z - pt2.z
      return math.sqrt(math.pow(xDist, 2) + math.pow(yDist, 2) + math.pow(zDist, 2))

def main():
    rospy.init_node('path_node')
    wait_for_time()
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()
    
if __name__ == '__main__':
    main()
