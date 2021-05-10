import logging
import os
import numpy as np

import rospy
import message_filters
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, LEDPattern
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.srv import SetCustomLEDPattern

logger = logging.getLogger('ROSClient')
logging.basicConfig()
logger.setLevel(logging.DEBUG)


class ROSClient:

    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.environ.get('VEHICLE_NAME', None)

        if self.vehicle is None:
            rospy.logerr("The variable VEHICLE_NAME is not set. Exiting...")
            exit(1)

        self.nsent_commands = 0
        self.nreceived_images = 0
        self.nreceived_encoders = 0

        self.shutdown = False

        # we are initialized if we have received a camera image
        self.initialized = False

        # Initializes the node
        rospy.init_node('ROSClient')
        rospy.on_shutdown(self.on_shutdown)

        # self.r = rospy.Rate(100)
        msg = 'ROSClient initialized.'
        logger.info(msg)

        cmd_topic = f'/{self.vehicle}/wheels_driver_node/wheels_cmd'
        self.cmd_pub = rospy.Publisher(cmd_topic, WheelsCmdStamped, queue_size=10)
        logger.info('wheel command publisher created')

        img_topic = f'/{self.vehicle}/camera_node/image/compressed'
        self.cam_sub = rospy.Subscriber(img_topic, CompressedImage, self._cam_cb)
        logger.info('camera subscriber created')

        # Setup the time synchronizer
        encoder_left_hz = rospy.get_param(f'/{self.vehicle}/left_wheel_encoder_node/publish_frequency', None)
        encoder_right_hz = rospy.get_param(f'/{self.vehicle}/right_wheel_encoder_node/publish_frequency', None)
        if encoder_left_hz is not None and encoder_right_hz is not None:
            # Setup subscribers
            left_encoder_topic = f'/{self.vehicle}/left_wheel_encoder_node/tick'
            right_encoder_topic = f'/{self.vehicle}/right_wheel_encoder_node/tick'
            self.left_encoder_sub = message_filters.Subscriber(left_encoder_topic, WheelEncoderStamped)
            self.right_encoder_sub = message_filters.Subscriber(right_encoder_topic, WheelEncoderStamped)
            # sync topics
            encoder_hz = min(encoder_left_hz, encoder_right_hz)
            logger.info(f"Encoders have frequencies: Left({encoder_left_hz}Hz), Right({encoder_right_hz}Hz)")
            logger.info(f"Synchronizing encoders at a frequency of {encoder_hz}Hz")
            self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
                [self.left_encoder_sub, self.right_encoder_sub], 10, 1.0 / encoder_hz)
            self.ts_encoders.registerCallback(self._encoder_cb)
        else:
            logger.info("The robot does not seem to have encoders. Skipping encoders integration.")

        # we arbitrarily take the resolution of the left encoder since we are assuming them to be the same
        # if this parameter is not set, we default to 0
        resolution = rospy.get_param(f'/{self.vehicle}/left_wheel_encoder_node/resolution', None)
        if resolution is not None:
            logger.info(f'Got resolution of {resolution} from encoder')
            self.resolution_rad = np.pi * 2/resolution
        else:
            logger.info("The robot does not seem to have encoders. Skipping encoders integration.")
            self.resolution_rad = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0

        led_set_pattern_topic = f'/{self.vehicle}/led_emitter_node/set_custom_pattern'
        self.change_led_pattern = rospy.ServiceProxy(led_set_pattern_topic, SetCustomLEDPattern)

    def on_shutdown(self):
        self.shutdown = True
        msg = 'ROSClient on_shutdown will send 0,0 command now.'
        logger.info(msg)
        commands = {u'motor_right': 0.0, u'motor_left': 0.0}
        self.send_commands(commands)

    def _encoder_cb(self, msg_left, msg_right):
        self.left_encoder_ticks = msg_left.data
        self.right_encoder_ticks = msg_right.data
        # ---
        if self.nreceived_encoders == 0:
            msg = 'ROSClient received first data from the encoders'
            logger.info(msg)
        # ---
        self.nreceived_encoders += 1

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
            # print(resp)
        except Exception as e:
            print(e)

