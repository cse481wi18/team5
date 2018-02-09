#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import perception
import rospy

def wait_for_time(): 
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('publish_saved_cloud')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print 'Publishes a saved point cloud to a latched topic'
        print 'Usage: rosrun applications publish_saved_cloud.py ~/cloud.bag'
        return
    path = argv[1]

    #rospy.loginfo("About to open mock camera")

    camera = perception.MockCamera()
    cloud = camera.read_cloud(path)

    if cloud is None:
        rospy.logerr('Could not load point cloud from {}'.format(path))
        return

    #rospy.loginfo("Before publishing")
    
    pub = rospy.Publisher('mock_point_cloud', PointCloud2, queue_size=1)       
    rate = rospy.Rate(2)

    #rospy.loginfo("Publisher registered")

    while not rospy.is_shutdown():
        cloud.header.stamp = rospy.Time.now()
        #rospy.loginfo(cloud.header)
        pub.publish(cloud)
        rate.sleep()                                          
    
    
if __name__ == '__main__':
    main()
