import logging
import os

import message_filters
import numpy as np

import rospy
from duckietown_msgs.msg import LEDPattern, WheelEncoderStamped, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ColorRGBA


__all__ = ["ROSClient"]


class ROSClient:
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.environ.get("VEHICLE_NAME", None)

        if self.vehicle is None:
            rospy.logerror("The variable VEHICLE_NAME is not set. Exiting...")
            exit(1)

        self.nsent_commands = 0
        self.nreceived_images = 0
        self.nreceived_encoders = 0
        self.image_data_timestamp = 0.0
        self.encoder_stamp = 0.0

        self.shutdown = False

        # we are initialized if we have received a camera image
        self.initialized = False

        # Initializes the node
        try:
            rospy.init_node("duckiebot-fifos-bridge", log_level=rospy.DEBUG, disable_rosout=False)
        except rospy.RosException as e:
            rospy.logerror(f"Failed to init_node {e}")
            exit(1)

        log_path = "/challenges/challenge-fifos-bridge-output"
        if not os.path.exists(log_path):
            os.makedirs(log_path)

        fh = logging.FileHandler(f"{log_path}/fifos-bridge-after-init_node.log")
        fh.setLevel(logging.DEBUG)
        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        # create formatter and add it to the handlers
        # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        # fh.setFormatter(formatter)
        # ch.setFormatter(formatter)
        # add the handlers to the logger
        root = logging.getLogger()
        root.addHandler(fh)
        root.addHandler(ch)

        try:
            rospy.on_shutdown(self.on_shutdown)
        except rospy.RosException as e:
            rospy.logerror(f"Failed to register on_shutdown callback {e}")
            exit(1)

        # self.r = rospy.Rate(100)
        msg = "ROSClient initialized."
        rospy.loginfo(msg)

        cmd_topic = f"/{self.vehicle}/wheels_driver_node/wheels_cmd"
        self.cmd_pub = rospy.Publisher(cmd_topic, WheelsCmdStamped, queue_size=10)
        rospy.loginfo("wheel command publisher created")

        led_topic = f"/{self.vehicle}/led_emitter_node/led_pattern"
        self.led_pub = rospy.Publisher(led_topic, LEDPattern, queue_size=10)
        rospy.loginfo("led publisher creaated")

        img_topic = f"/{self.vehicle}/camera_node/image/compressed"
        self.cam_sub = rospy.Subscriber(img_topic, CompressedImage, self._cam_cb)
        rospy.loginfo("camera subscriber created")

        # Setup the time synchronizer
        encoder_left_hz = rospy.get_param(f"/{self.vehicle}/left_wheel_encoder_node/publish_frequency", None)
        encoder_right_hz = rospy.get_param(
            f"/{self.vehicle}/right_wheel_encoder_node/publish_frequency", None
        )
        if encoder_left_hz is not None and encoder_right_hz is not None:
            # Setup subscribers
            left_encoder_topic = f"/{self.vehicle}/left_wheel_encoder_node/tick"
            right_encoder_topic = f"/{self.vehicle}/right_wheel_encoder_node/tick"
            self.left_encoder_sub = message_filters.Subscriber(left_encoder_topic, WheelEncoderStamped)
            self.right_encoder_sub = message_filters.Subscriber(right_encoder_topic, WheelEncoderStamped)
            # sync topics
            encoder_hz = min(encoder_left_hz, encoder_right_hz)
            rospy.loginfo(
                f"Encoders have frequencies: Left({encoder_left_hz}Hz), Right({encoder_right_hz}Hz)"
            )
            rospy.loginfo(f"Synchronizing encoders at a frequency of {encoder_hz}Hz")
            self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
                [self.left_encoder_sub, self.right_encoder_sub], 10, 1.0 / encoder_hz
            )
            self.ts_encoders.registerCallback(self._encoder_cb)
        else:
            rospy.loginfo("The robot does not seem to have encoders. Skipping encoders integration.")

        # we arbitrarily take the resolution of the left encoder since we are assuming them to be the same
        # if this parameter is not set, we default to 0
        resolution = rospy.get_param(f"/{self.vehicle}/left_wheel_encoder_node/resolution", None)
        if resolution is not None:
            rospy.loginfo(f"Got resolution of {resolution} from encoder")
            self.resolution_rad = np.pi * 2 / resolution
        else:
            rospy.loginfo("The robot does not seem to have encoders. Skipping encoders integration.")
            self.resolution_rad = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0

    def on_shutdown(self):
        self.shutdown = True
        msg = "ROSClient on_shutdown sends 0,0 command."
        rospy.loginfo(msg)
        commands = {"motor_right": 0.0, "motor_left": 0.0}
        self.send_commands(commands)

    def _encoder_cb(self, msg_left: WheelEncoderStamped, msg_right: WheelEncoderStamped):
        self.left_encoder_ticks = msg_left.data
        self.right_encoder_ticks = msg_right.data
        self.encoder_stamp = max(msg_left.header.stamp.to_sec(), msg_right.header.stamp.to_sec())

        # ---
        if self.nreceived_encoders == 0:
            msg = "ROSClient received first data from the encoders"
            rospy.loginfo(msg)
        # ---
        self.nreceived_encoders += 1

    def _cam_cb(self, msg: CompressedImage):
        """
        Callback to listen to last outputted camera image and store it
        """
        self.image_msg = msg
        self.image_data = msg.data

        self.image_data_timestamp = msg.header.stamp.to_time()
        self.initialized = True
        if self.nreceived_images == 0:
            msg = "ROSClient received first camera image."
            rospy.loginfo(msg)
        self.nreceived_images += 1

    def send_commands(self, cmds):
        """
        Publishes the wheel commands to ROS
        """
        time = rospy.get_rostime()
        cmd_msg = WheelsCmdStamped()
        cmd_msg.header.stamp = time
        cmd_msg.vel_right = cmds["motor_right"]
        cmd_msg.vel_left = cmds["motor_left"]
        if self.nsent_commands == 0:
            msg = "ROSClient publishing first commands."
            rospy.loginfo(msg)

        self.cmd_pub.publish(cmd_msg)
        self.nsent_commands += 1

    def change_leds(self, data):
        """
        Publishes the change in LEDs
        """

        def createRGBAmsg(a):
            msg = ColorRGBA()
            msg.r = a[0]
            msg.g = a[1]
            msg.b = a[2]
            msg.a = 1
            return msg

        led_pattern_msg = LEDPattern()
        c = createRGBAmsg(data["center"])
        fl = createRGBAmsg(data["front_left"])
        fr = createRGBAmsg(data["front_right"])
        bl = createRGBAmsg(data["back_left"])
        br = createRGBAmsg(data["back_right"])
        led_pattern_msg.rgb_vals = [fl, bl, c, br, fr]

        self.led_pub.publish(led_pattern_msg)
