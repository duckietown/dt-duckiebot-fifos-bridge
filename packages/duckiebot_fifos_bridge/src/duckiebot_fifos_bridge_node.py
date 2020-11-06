#!/usr/bin/env python3

import logging
import signal
import sys
import time
from typing import cast

import cv2
import numpy as np
from zuper_nodes_wrapper.struct import MsgReceived
from zuper_nodes_wrapper.wrapper_outside import ComponentInterface

from aido_schemas import (DB20Commands, DB20Observations, DB20Odometry, GetCommands, JPGImage,
                          protocol_agent_DB20)
from duckiebot_fifos_bridge.rosclient import ROSClient

logger = logging.getLogger('DuckiebotBridge')
logger.setLevel(logging.DEBUG)


class DuckiebotBridge:
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        AIDONODE_DATA_IN = '/fifos/agent-in'
        AIDONODE_DATA_OUT = '/fifos/agent-out'
        logger.info('DuckiebotBridge starting communicating with the agent.')
        self.ci = ComponentInterface(AIDONODE_DATA_IN, AIDONODE_DATA_OUT,
                                     expect_protocol=protocol_agent_DB20,
                                     nickname='agent',
                                     timeout=3600)
        self.ci.write_topic_and_expect_zero('seed', 32)
        self.ci.write_topic_and_expect_zero('episode_start', {'episode_name': 'episode'})
        logger.info('DuckiebotBridge successfully sent to the agent the seed and episode name.')
        self.client = ROSClient()
        logger.info('DuckiebotBridge has created ROSClient.')

    def exit_gracefully(self, signum, frame):
        logger.info('DuckiebotBridge exiting gracefully.')
        sys.exit(0)

    def run(self):
        nimages_received = 0
        t0 = time.time()
        t_last_received = None
        while True:
            if not self.client.initialized:
                if nimages_received == 0:
                    elapsed = time.time() - t0
                    msg = 'DuckiebotBridge still waiting for the first image: elapsed %s' % elapsed
                    logger.info(msg)
                    time.sleep(0.5)
                else:
                    elapsed = time.time() - t_last_received
                    if elapsed > 2:
                        msg = 'DuckiebotBridge has waited %s since last image' % elapsed
                        logger.info(msg)
                        time.sleep(0.5)
                    else:
                        time.sleep(0.01)
                continue

            jpg_data = self.client.image_data
            camera = JPGImage(jpg_data)
            resolution_rad: float = np.pi * 2 / 135

            axis_left_rad: float = self.client.left_encoder_ticks * self.client.resolution_rad
            axis_right_rad: float = self.client.right_encoder_ticks * self.client.resolution_rad
            odometry = DB20Odometry(axis_left_rad=axis_left_rad, axis_right_rad=axis_right_rad,
                                    resolution_rad=resolution_rad)
            obs = DB20Observations(camera, odometry)
            if nimages_received == 0:
                logger.info('DuckiebotBridge got the first image from ROS.')

            self.ci.write_topic_and_expect_zero('observations', obs)
            gc = GetCommands(at_time=time.time())
            r: MsgReceived = self.ci.write_topic_and_expect('get_commands', gc, expect='commands')
            wheels = r.data.wheels
            lw, rw = wheels.motor_left, wheels.motor_right
            commands = {u'motor_right': rw, u'motor_left': lw}

            self.client.send_commands(commands)
            if nimages_received == 0:
                logger.info('DuckiebotBridge published the first commands.')

            nimages_received += 1
            t_last_received = time.time()


def bgr2jpg(bgr: np.ndarray) -> bytes:
    compress = cv2.imencode('.jpg', bgr)[1]
    jpg_data = np.array(compress).tostring()
    return jpg_data


def main():
    node = DuckiebotBridge()
    node.run()


if __name__ == '__main__':
    main()
