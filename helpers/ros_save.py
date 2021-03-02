import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import rosbag


class RosSaver():
    """
    Class that saves different messages to a rosbag file.
    Uses the rosbag python API to write messages.
    """
    def __init__(self, rosbagfilename=None):
        if rosbagfilename:
            self.bag = rosbag.Bag(rosbagfilename, 'w', compression='bz2')
            print self.bag.get_compression_info()

    def __del__(self):
        print("Closing bag")
        self.bag.close()

    def save_clock(self, epoch):
        sim_clock = Clock()
        sim_clock.clock = rospy.Time.from_sec(epoch)
        self.bag.write('clock', sim_clock, t=rospy.Time.from_sec(epoch))

    def save_gps(self, gps):
        msg = NavSatFix()
        msg.header = build_header(gps.epoch, 'gps')

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position in degrees.
        msg.latitude = gps.lat
        msg.longitude = gps.lng

        # Altitude in metres.
        msg.altitude = gps.altitude

        msg.position_covariance[0] = 0
        msg.position_covariance[4] = 0
        msg.position_covariance[8] = 0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # save lat, lng at that time
        self.bag.write('virb360/gps/fix', msg, t=rospy.Time.from_sec(gps.epoch))

    def save_gps_speed(self, gps):
        """
        Save GPS speed as measured by the GPS on the FIT device.
        :param gps:
        :return:
        """
        msg = Vector3()
        # Position in degrees.
        msg.x = gps.speed
        msg.y = 0
        msg.z = 0
        # publish absolute speed
        self.bag.write('virb360/gps/speed', msg, t=rospy.Time.from_sec(gps.epoch))

    def save_gps_velocity(self, gps):
        """
        Save a 3D velocity vector.
        :param gps:
        :return:
        """
        msg = Vector3()
        # Position in degrees.
        msg.x = gps.velocity[0]
        msg.y = gps.velocity[1]
        msg.z = gps.velocity[2]
        # publish vx, vy, vz
        self.bag.write('virb360/gps/velocity', msg, t=rospy.Time.from_sec(gps.epoch))

    def save_image(self, image, epoch):
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        image_message.header = build_header(epoch, 'image')
        self.bag.write('image/image_raw', image_message, t=rospy.Time.from_sec(epoch))

    def save_image_compressed(self, image, epoch):
        """
        Publish a compressed image in ROS. In this case, publishing from a captured video frame.
        :param image: an opencv image
        :return:
        """
        image_message = CompressedImage()
        image_message.header = build_header(epoch, 'image')
        image_message.format = "png"
        image_message.data = np.array(cv2.imencode('.png', image)[1]).tostring()
        self.bag.write('image/image_raw/compressed', image_message, t=rospy.Time.from_sec(epoch))


def build_header(epoch, frame_id):
    header = Header()
    clock = rospy.Time.from_sec(epoch)
    header.stamp = clock
    header.frame_id = frame_id
    return header
