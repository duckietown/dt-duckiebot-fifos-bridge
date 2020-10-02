import logging
import os
import cv2
from cv_bridge import CvBridge

import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage

logger = logging.getLogger('ROSClient')
logger.setLevel(logging.DEBUG)


class ROSClient:
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        # TODO not sure about this
        self.vehicle = os.getenv('VEHICLE_NAME')

        self.nsent_commands = 0
        self.nreceived_images = 0
        self.shutdown = False

        self.initialized = False

        # Initializes the node
        rospy.init_node('ROSClient')
        rospy.on_shutdown(self.on_shutdown)

        self.r = rospy.Rate(100)
        msg = 'ROSClient initialized.'
        logger.info(msg)

        topic = f'/{self.vehicle}/wheels_driver_node/wheels_cmd'

        self.cmd_pub = rospy.Publisher(topic, WheelsCmdStamped, queue_size=10)
        logger.info('publisher created')
        topic = f'/{self.vehicle}/camera_node/image/compressed'
        self.cam_sub = rospy.Subscriber(topic, CompressedImage, self._cam_cb)

        self.bridge = CvBridge()

        logger.info('subscriber created')

    def on_shutdown(self):
        self.shutdown = True
        msg = 'ROSClient on_shutdown will send 0,0 command now.'
        logger.info(msg)
        commands = {u'motor_right': 0.0, u'motor_left': 0.0}
        self.send_commands(commands)

    def _cam_cb(self, msg):
        """
        Callback to listen to last outputted camera image and store it
        """
        self.image_msg = msg
        self.image_data = msg.data
        self.initialized = True
        if self.nreceived_images == 0:
            msg = 'ROSClient received first camera image.'
            logger.info(msg)
        self.nreceived_images += 1

    def send_commands(self, cmds):
        """
        Publishes the wheel commands to ROS
        """
        time = rospy.get_rostime()
        cmd_msg = WheelsCmdStamped()
        cmd_msg.header.stamp.secs = time.secs
        cmd_msg.header.stamp.nsecs = time.nsecs
        cmd_msg.vel_right = cmds[u'motor_right']
        cmd_msg.vel_left = cmds[u'motor_left']
        if self.nsent_commands == 0:
            msg = 'ROSClient publishing first commands.'
            logger.info(msg)

        self.cmd_pub.publish(cmd_msg)
        self.nsent_commands += 1
