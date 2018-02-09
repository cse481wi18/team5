# Usage:
# camera = perception.MockCamera()
# cloud = camera.read_cloud('~/data/tags.bag')

import rosbag
from sensor_msgs.msg import PointCloud2

class MockCamera(object): 
    """A MockCamera reads saved point clouds.
    """
    def __init__(self):
        pass

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.
    
        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """
        bag = rosbag.Bag(path, 'r')

        # Get the first message of this topic generator, and get the 1th element (which is the actual msg)
        pc2_msg = next(bag.read_messages(topics=['head_camera/depth_registered/points']))[1]

        return pc2_msg
