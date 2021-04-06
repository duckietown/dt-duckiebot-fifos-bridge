import logging
import os
import cv2
from cv_bridge import CvBridge
import numpy as np

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, LEDPattern
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.srv import SetCustomLEDPattern

logger = logging.getLogger('ROSClient')
logger.setLevel(logging.DEBUG)


class ROSClient:
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        # TODO not sure about this
        self.vehicle = os.getenv('VEHICLE_NAME')

        self.nsent_commands = 0
        self.nreceived_images = 0
        self.nreceived_encoder_left = 0
        self.nreceived_encoder_right = 0

        self.shutdown = False

        # we are initialized if we have received a camera image
        self.initialized = False

        # Initializes the node
        rospy.init_node('ROSClient')
        rospy.on_shutdown(self.on_shutdown)

        self.r = rospy.Rate(100)
        msg = 'ROSClient initialized.'
        logger.info(msg)

        cmd_topic = f'/{self.vehicle}/wheels_driver_node/wheels_cmd'
        self.cmd_pub = rospy.Publisher(cmd_topic, WheelsCmdStamped, queue_size=10)
        logger.info('wheel command publisher created')

        img_topic = f'/{self.vehicle}/camera_node/image/compressed'
        self.cam_sub = rospy.Subscriber(img_topic, CompressedImage, self._cam_cb)
        logger.info('camera subscriber created')

        left_encoder_topic = f'/{self.vehicle}/left_wheel_encoder_node/tick'
        self.left_encoder_sub = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped , self._left_encoder_cb)
        logger.info('left encoder subscriber created')

        right_encoder_topic = f'/{self.vehicle}/right_wheel_encoder_node/tick'
        self.right_encoder_sub = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped , self._right_encoder_cb)
        logger.info('right encoder subscriber created')

        # we arbitrarily take the resolution of the left encoder since we are assuming them to be the same
        # if this parameter is not set, we default to 0
        resolution = rospy.get_param(f'/{self.vehicle}/left_wheel_encoder_node/resolution', 0)
        if resolution != 0:
            logger.info(f'got resolution of {resolution} from encoder')
            self.resolution_rad = np.pi * 2/resolution
        else:
            logger.info('got resolution of 0, either no encoders or resolution param not read properly')
            self.resolution_rad = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0

        led_set_pattern_topic = f'/{self.vehicle}/led_emitter_node/set_custom_pattern'
        self.change_led_pattern = rospy.ServiceProxy(led_set_pattern_topic, SetCustomLEDPattern)
        
        self.bridge = CvBridge()


    def on_shutdown(self):
        self.shutdown = True
        msg = 'ROSClient on_shutdown will send 0,0 command now.'
        logger.info(msg)
        commands = {u'motor_right': 0.0, u'motor_left': 0.0}
        self.send_commands(commands)

    def _left_encoder_cb(self,msg):
        self.left_encoder_ticks = msg.data
        if self.nreceived_encoder_left == 0:
            msg = 'ROSClient received first data from left encoder'
            logger.info(msg)
        self.nreceived_encoder_left += 1

    def _right_encoder_cb(self, msg):
        self.right_encoder_ticks = msg.data
        if self.nreceived_encoder_right == 0:
            msg = 'ROSClient received first data from right encoder'
            logger.info(msg)
        self.nreceived_encoder_right += 1

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

    def change_leds(self, data):
        """
        Calls the change LED service
        """
        def createRGBAmsg(a):
            msg = ColorRGBA()
            msg.r = a[0]
            msg.g = a[1]
            msg.b = a[2]
            msg.a = 1
            return msg

        
        led_pattern = LEDPattern()
        time = rospy.get_rostime()
        led_pattern.header.stamp.secs = time.secs
        led_pattern.header.stamp.nsecs = time.nsecs
        c=createRGBAmsg(data[u'center'])
        fl=createRGBAmsg(data[u'front_left'])
        fr=createRGBAmsg(data[u'front_right'])
        bl=createRGBAmsg(data[u'back_left'])
        br=createRGBAmsg(data[u'back_right'])
        led_pattern.rgb_vals = [c,fl,fr,bl,br]
        led_pattern.color_mask = [1,1,1,1,1]
        led_pattern.frequency = 1.0
        led_pattern.frequency_mask = [1,1,1,1,1]
        try:
            resp = self.change_led_pattern(led_pattern)
            print(resp)
        except Exception as e:
            print(e)

