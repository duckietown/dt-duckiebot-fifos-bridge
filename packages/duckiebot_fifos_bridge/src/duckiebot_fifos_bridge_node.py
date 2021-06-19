#!/usr/bin/env python3

import logging
import os
import signal
import sys
import time
from threading import Thread
from typing import List

import numpy as np

import rospy
from aido_schemas import (DB20ObservationsWithTimestamp, DB20OdometryWithTimestamp, GetCommands,
                          JPGImageWithTimestamp, protocol_agent_DB20_timestamps, RGB)
from duckiebot_fifos_bridge.rosclient import ROSClient
from zuper_nodes_wrapper.struct import MsgReceived
from zuper_nodes_wrapper.wrapper_outside import ComponentInterface

logger = logging.getLogger('DuckiebotBridge')
logger.setLevel(logging.DEBUG)


class DuckiebotBridge:

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        AIDONODE_DATA_IN = os.getenv("AIDONODE_DATA_IN", "/fifos/ego0-in")
        AIDONODE_DATA_OUT = os.getenv("AIDONODE_DATA_OUT", "/fifos/ego0-out")
        logger.info('DuckiebotBridge starting communicating with the agent.')
        self.ci = ComponentInterface(AIDONODE_DATA_IN, AIDONODE_DATA_OUT,
                                     expect_protocol=protocol_agent_DB20_timestamps,
                                     nickname='agent',
                                     timeout=3600)
        self.ci.write_topic_and_expect_zero('seed', 32)
        self.ci.write_topic_and_expect_zero('episode_start', {'episode_name': 'episode'})
        logger.info('DuckiebotBridge successfully sent to the agent the seed and episode name.')
        self.client = ROSClient()
        logger.info('DuckiebotBridge has created ROSClient.')

    def exit_gracefully(self, signum, frame):
        logger.info('DuckiebotBridge exiting gracefully.')
        self.client.on_shutdown()
        sys.exit(0)

    def run(self):
        nimages_received = 0
        t0 = time.time()
        t_last_transmitted = -1
        while True:
            if not self.client.initialized:
                if nimages_received == 0:
                    elapsed = time.time() - t0
                    msg = 'DuckiebotBridge still waiting for the first image: elapsed %s' % elapsed
                    logger.info(msg)
                    time.sleep(0.5)
                    continue

            current_data_timestamp = max(self.client.image_data_timestamp, self.client.encoder_stamp)
            if current_data_timestamp == t_last_transmitted:
                time.sleep(0.02)
                continue
            t_last_transmitted = current_data_timestamp

            jpg_data = self.client.image_data
            camera = JPGImageWithTimestamp(jpg_data, timestamp=self.client.image_data_timestamp)
            resolution_rad: float = float(np.pi * 2 / 135)  # FIXME: hardcoded

            axis_left_rad: float = float(self.client.left_encoder_ticks * self.client.resolution_rad)
            axis_right_rad: float = float(self.client.right_encoder_ticks * self.client.resolution_rad)
            odometry = DB20OdometryWithTimestamp(axis_left_rad=axis_left_rad, axis_right_rad=axis_right_rad,
                                                 resolution_rad=resolution_rad,
                                                 timestamp=self.client.encoder_stamp)
            obs = DB20ObservationsWithTimestamp(camera, odometry)
            if nimages_received == 0:
                logger.info('DuckiebotBridge got the first image from ROS.')

            self.ci.write_topic_and_expect_zero('observations', obs)
            gc = GetCommands(at_time=time.time())
            r: MsgReceived = self.ci.write_topic_and_expect('get_commands', gc, expect='commands')
            wheels = r.data.wheels
            lw, rw = wheels.motor_left, wheels.motor_right
            pwm_commands = {u'motor_right': rw, u'motor_left': lw}
            self.client.send_commands(pwm_commands)
            leds = r.data.LEDS
            c = RGBfloat2int(leds.center)
            fl = RGBfloat2int(leds.front_left)
            fr = RGBfloat2int(leds.front_right)
            bl = RGBfloat2int(leds.back_left)
            br = RGBfloat2int(leds.back_right)
            led_commands = {
                u'center': c,
                u'front_left': fl,
                u'front_right': fr,
                u'back_left': bl,
                u'back_right': br,
            }
            # self.client.change_leds(led_commands)

            if nimages_received == 0:
                logger.info('DuckiebotBridge published the first commands.')

            nimages_received += 1
            t_last_received = time.time()


def RGBfloat2int(from_fifo: RGB) -> List[int]:
    b = []
    for a in [from_fifo.r, from_fifo.g, from_fifo.b]:
        test_rgb_float_value(a)
        b.append(int(a * 255))
    return b


def test_rgb_float_value(channel: float):
    if channel > 1.0 or channel < 0.0:
        logger.error(f'LED value out of range {channel}')


def main():
    node = DuckiebotBridge()

    worker = Thread(target=node.run)
    worker.start()

    rospy.spin()


if __name__ == '__main__':
    main()
