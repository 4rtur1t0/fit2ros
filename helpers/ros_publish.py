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


class RosPublisher():
    def __init__(self):
        self.clock_publisher = rospy.Publisher('clock', Clock, queue_size=10)
        # publish lat lng and altitude from gps
        self.gps_publisher = rospy.Publisher('virb360/gps/fix', NavSatFix, queue_size=10)
        # publish gps speed
        self.gps_speed_publisher = rospy.Publisher('virb360/gps/speed', Vector3, queue_size=10)
        # publish gps velocity from imu
        self.gps_velocity_publisher = rospy.Publisher('virb360/gps/velocity', Vector3, queue_size=10)
        # publish images as captured from videos
        self.image_publisher = rospy.Publisher('image/image_raw', Image, queue_size=10)
        # publish images as captured from videos
        #self.compressed_image_publisher = rospy.Publisher('image/image_raw/compressed', CompressedImage, queue_size=10)
        # finally init publication node
        rospy.init_node('fit2ros', anonymous=True)

    def publish_clock(self, epoch):
        sim_clock = Clock()
        sim_clock.clock = rospy.Time.from_sec(epoch)
        # rospy.loginfo(sim_clock)
        self.clock_publisher.publish(sim_clock)

    def publish_gps(self, gps):
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

        # publish lat, lng
        self.gps_publisher.publish(msg)

    def publish_gps_speed(self, gps):
        """
        Publish GPS speed as measured by the GPS on the FIT device.
        :param gps:
        :return:
        """
        msg = Vector3()
        # Position in degrees.
        msg.x = gps.speed
        msg.y = 0
        msg.z = 0
        # publish absolute speed
        self.gps_speed_publisher.publish(msg)

    def publish_gps_velocity(self, gps):
        """
        Publish a 3D velocity vector.
        :param gps:
        :return:
        """
        msg = Vector3()
        # Position in degrees.
        msg.x = gps.velocity[0]
        msg.y = gps.velocity[1]
        msg.z = gps.velocity[2]
        # publish vx, vy, vz
        self.gps_velocity_publisher.publish(msg)

    def publish_image(self, image, epoch):
        """
        Publish an image in ROS. In this case, publishing from a captured video frame.
        :param image: an opencv image
        :return:
        """
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        image_message.header = build_header(epoch, 'image')
        self.image_publisher.publish(image_message)

    def publish_image_compressed(self, image, epoch):
        """
        Publish a compressed image in ROS. In this case, publishing from a captured video frame.
        :param image: an opencv image
        :return:
        """
        image_message = CompressedImage()
        image_message.header = build_header(epoch, 'image')
        image_message.format = "png"
        image_message.data = np.array(cv2.imencode('.png', image)[1]).tostring()
        self.compressed_image_publisher.publish(image_message)


def build_header(epoch, frame_id):
    header = Header()
    clock = rospy.Time.from_sec(epoch)
    header.stamp = clock
    header.frame_id = frame_id
    return header
