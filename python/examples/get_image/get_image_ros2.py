# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

'''
This file:
python3 get_image_ros2.py 192.168.50.3

To run get_image.py:
python3 get_image.py 192.168.50.3 --image-sources frontleft_fisheye_image --image-sources frontright_fisheye_image

To record .db3 file on Pi:
ros2 bag record /front_left_image /front_right_image  -o mar12_11

To convert .db3 to .bag on Pi:
rosbags-convert mar12_11

'''

"""Simple image capture tutorial."""

import argparse
import sys
import os

import cv2
import numpy as np
from scipy import ndimage

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

ROTATION_ANGLE = {
    'back_fisheye_image': 0,
    'frontleft_fisheye_image': -78,
    'frontright_fisheye_image': -102,
    'left_fisheye_image': 0,
    'right_fisheye_image': 180
}


def pixel_format_type_strings():
    names = image_pb2.Image.PixelFormat.keys()
    return names[1:]


def pixel_format_string_to_enum(enum_string):
    return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)


cameras = ["frontleft_fisheye_image", "frontright_fisheye_image"]
# cameras = ["frontleft_fisheye_image"]

rate = 0.033 # 0.033 == 30fps, 0.05 == 20fps


class MinimalPublisher(Node):

    def __init__(self, argv):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'some_topic', 1)
        # self.image_publisher = self.create_publisher(Image, 'images', 2)
        self.front_left_image_publisher = self.create_publisher(CompressedImage, 'front_left_image', 0)
        self.front_right_image_publisher = self.create_publisher(CompressedImage, 'front_right_image', 0) 
        
        timer_period = rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.cameras = cameras
        self.bridge = CvBridge()

        # self.image_publishers = []
        # for source in self.cameras:
        #     self.image_publishers.append(self.create_publisher(Image, source, 1))

        # Parse args
        self.parser = argparse.ArgumentParser()
        bosdyn.client.util.add_base_arguments(self.parser)
        self.parser.add_argument('--list', help='list image sources', action='store_true')
        self.parser.add_argument('--auto-rotate', help='rotate right and front images to be upright',
                            action='store_true')
        # self.parser.add_argument('--image-sources', help='Get image from source(s)', action='append')
        self.parser.add_argument('--image-service', help='Name of the image service to query.',
                            default=ImageClient.default_service_name)
        self.parser.add_argument(
            '--pixel-format', choices=pixel_format_type_strings(),
            help='Requested pixel format of image. If supplied, will be used for all sources.')

        self.options = self.parser.parse_args(argv)

        # Create robot object with an image client.
        self.sdk = bosdyn.client.create_standard_sdk('image_capture')
        self.robot = self.sdk.create_robot(self.options.hostname)
        password = os.environ['BOSDYN_CLIENT_PASSWORD']
        self.robot.authenticate(username="admin", password=password)
        bosdyn.client.util.authenticate(self.robot)
        self.robot.sync_with_directory()
        self.robot.time_sync.wait_for_sync()

        self.image_client = self.robot.ensure_client(self.options.image_service)

        # Optionally list image sources on robot.
        if False: # NITZAN if self.list:
            image_sources = self.image_client.list_image_sources()
            print('Image sources:')
            for source in image_sources:
                print('\t' + source.name)




    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        
        if self.i < 300:
            # Capture and save images to disk
            pixel_format = pixel_format_string_to_enum(self.options.pixel_format)
            image_request = [
                build_image_request(source, pixel_format=pixel_format) 
                for source in cameras # NITZAN options.image_sources
            ]
            image_responses = self.image_client.get_image(image_request)
            
            images = [0, 0]
            for n, image in enumerate(image_responses):
                num_bytes = 1  # Assume a default of 1 byte encodings.
                if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                    dtype = np.uint16
                    extension = '.png'
                else:
                    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                        num_bytes = 3
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                        num_bytes = 4
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                        num_bytes = 1
                    elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
                        num_bytes = 2
                    dtype = np.uint8
                    extension = '.jpg'
                img = np.frombuffer(image.shot.image.data, dtype=dtype)
                if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                    try:
                        # Attempt to reshape array into a RGB rows X cols shape.
                        img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_bytes))
                    except ValueError:
                        # Unable to reshape the image data, trying a regular decode.
                        img = cv2.imdecode(img, -1)
                else:
                    img = cv2.imdecode(img, -1)

                resize = False
                rotate = False # HIGH COMPUTATION COST. Resizing beforehand helps.
                if resize:
                    img = cv2.resize(img, (320, 240))
                if rotate and image.source.name in ROTATION_ANGLE:
                    img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name]) 
                self.size = [len(img), len(img[0])]

                image_message = None
                
                if n == 0:
                    image_message = self.bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
                    images[0] = image_message
                    
                elif n == 1:
                    image_message = self.bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
                    images[1] = image_message
                else:
                    print("n", n)     

            self.front_left_image_publisher.publish(images[0])
            self.front_right_image_publisher.publish(images[1])
           
                

            if self.i % 10 == 0:
                print("Loop:", self.i)
                if self.size:
                    print("Size:", self.size)

            self.i += 1        
 




if __name__ == '__main__':
    rclpy.init()
    try: 
        minimal_publisher = MinimalPublisher(sys.argv[1:])
        rclpy.spin(minimal_publisher)
    except Exception as e:
        raise e
    else:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

