#!/usr/bin/python3

import sys

import pygame
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class IOSound(Node):
    def __init__(self):
        super().__init__("io_sound")
        # ----Subscriber
        self.sub_sound_once = self.create_subscription(
            String, "sound_once", self.cllbck_sub_sound_once, 10
        )
        self.sub_sound_loop = self.create_subscription(
            String, "sound_loop", self.cllbck_sub_sound_loop, 10
        )

        # Pygame mixer sound
        # ------------------
        self.pygame_mixer_sounds = {}

        pygame.init()

    def cllbck_sub_sound_once(self, msg):
        filename = get_package_share_directory("pandu_ros2_kit") + "/assets/" + msg.data

        try:
            if msg.data not in self.pygame_mixer_sounds:
                self.pygame_mixer_sounds[msg.data] = pygame.mixer.Sound(filename)
            self.pygame_mixer_sounds[msg.data].play(loops=0, fade_ms=100)
            self.pygame_mixer_sounds[msg.data].set_volume(1.0)
        except Exception as e:
            self.get_logger().error(str(e))

    def cllbck_sub_sound_loop(self, msg):
        filename = get_package_share_directory("pandu_ros2_kit") + "/assets/" + msg.data

        try:
            if msg.data not in self.pygame_mixer_sounds:
                self.pygame_mixer_sounds[msg.data] = pygame.mixer.Sound(filename)
            self.pygame_mixer_sounds[msg.data].play(loops=-1, fade_ms=100)
            self.pygame_mixer_sounds[msg.data].set_volume(1.0)
        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)

    node_io_sound = IOSound()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_io_sound)
    executor.spin()


if __name__ == "__main__":
    main(sys.argv)
