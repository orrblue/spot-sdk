# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

# To run
# roscore
# cd Code/spot/spot-sdk/python/examples/get_image
# python3 get_image.py $ROBOT_IP --image-sources frontleft_fisheye_image --auto-rotate --pixel-format PIXEL_FORMAT_RGB_U8

# To record
# cd Data/spot/
# rosbag record -O image_data.bag /image_topic
# rosbag record -O image_data_dec_20_3.bag /image_front_left_rgb /image_gripper_rgb /image_external


'''
IDR if this is good or bad sys.path
>>> sys.path
['', '/opt/ros/noetic/lib/python3/dist-packages', 
'/usr/lib/python3/dist-packages', '/usr/lib/python38.zip', 
'/usr/lib/python3.8', '/usr/lib/python3.8/lib-dynload', 
'/home/nitzan/spot_env/lib/python3.8/site-packages']

This $PYTHONPATH might be needed to run get_image.py
(spot_env) echo $PYTHONPATH 
/opt/ros/noetic/lib/python3/dist-packages:
/usr/lib/python3/dist-packages

but to run rosbag record, $PYTHONPATH should be only
/opt/ros/noetic/lib/python3/dist-packages


current .bddf files on robot:
20231215_161000_spot-BD-32360001_data-buffer.bddf
    /home/logs/data-buffer/20231215_161000_spot-BD-32360001_data-buffer.bddf
20231215_161500_spot-BD-32360001_data-buffer.bddf (open?)
    /home/logs/data-buffer/20231215_161500_spot-BD-32360001_data-buffer.bddf.unfinished






'''

"""Simple image capture tutorial."""

import argparse
import sys
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np
from scipy import ndimage

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request

ROTATION_ANGLE = {
    'back_fisheye_image': 0,
    'frontleft_fisheye_image': -78,
    'frontright_fisheye_image': -102,
    'left_fisheye_image': 0,
    'right_fisheye_image': 180,
    'hand_color_image': 0
}


def pixel_format_type_strings():
    names = image_pb2.Image.PixelFormat.keys()
    return names[1:]


def pixel_format_string_to_enum(enum_string):
    return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)


def main(argv):
    # Parse args
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--list', help='list image sources', action='store_true')
    parser.add_argument('--auto-rotate', help='rotate right and front images to be upright',
                        action='store_true')
    parser.add_argument('--image-sources', help='Get image from source(s)', action='append')
    parser.add_argument('--image-service', help='Name of the image service to query.',
                        default=ImageClient.default_service_name)
    parser.add_argument(
        '--pixel-format', choices=pixel_format_type_strings(),
        help='Requested pixel format of image. If supplied, will be used for all sources.')

    options = parser.parse_args(argv)

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('image_capture')
    robot = sdk.create_robot(options.hostname)
    robot.authenticate(username="admin", password="bn537eyxetcd")
    bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()
    robot.time_sync.wait_for_sync()
    print("Image Service")
    print(options.image_service)
    print(options)

    image_client = robot.ensure_client(options.image_service)

    

    # Raise exception if no actionable argument provided
    if not options.list and not options.image_sources:
        parser.error('Must provide actionable argument (list or image-sources).')


    cap = cv2.VideoCapture(2)  # Use '0' for default webcam

    rospy.init_node('img_publisher', anonymous=True)
    pub_front_left_rgb = rospy.Publisher('image_front_left_rgb', CompressedImage, queue_size = 0)
    pub_gripper_rgb = rospy.Publisher('image_gripper_rgb', CompressedImage, queue_size = 0)
    pub_webcam = rospy.Publisher('image_external', CompressedImage, queue_size = 0)
    rate = rospy.Rate(20)

    for i in range(100):

        # Optionally list image sources on robot.
        if options.list:
            image_sources = image_client.list_image_sources()
            print('Image sources:')
            for source in image_sources:
                print('\t' + source.name)

        # Optionally capture one or more images.
        if options.image_sources:
            # Capture and save images to disk
            pixel_format = pixel_format_string_to_enum(options.pixel_format)
            
            image_request = [
                build_image_request(source, pixel_format=pixel_format) for source in ["frontleft_fisheye_image", "hand_color_image"] # NITZAN options.image_sources
            ]
            image_responses = image_client.get_image(image_request)

            for image in image_responses:
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

                if options.auto_rotate:
                    img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name])                


                resized_img = cv2.resize(img, (640, 480))

                # Compress the image
                ## PROBABLY UNNECESSARY AS ALREADY JPEG, but I couldn't make it work without it
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # Adjust JPEG quality as needed
                _, compressed_data = cv2.imencode('.jpg', resized_img, encode_param)

                
                 # Convert the compressed data to bytes
                comp_img_msg = CompressedImage()
                comp_img_msg.format = "jpeg"
                comp_img_msg.data = compressed_data.tobytes()

                # Publish
                if (image.source.name == "frontleft_fisheye_image"):
                    pub_front_left_rgb.publish(comp_img_msg)
                elif (image.source.name == "hand_color_image"):
                    pub_gripper_rgb.publish(comp_img_msg)

            ret = None
            webcam_frame = None
            ret, webcam_frame = cap.read()

            if ret:
                resized_webcam_frame = cv2.resize(webcam_frame, (640, 480))

                cv2.imshow('Webcam', resized_webcam_frame)

                # Compress the image
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # Adjust JPEG quality as needed
                _, compressed_data = cv2.imencode('.jpg', resized_webcam_frame, encode_param)

                # Convert the compressed data to bytes
                webcam_img_msg = CompressedImage()
                webcam_img_msg.format = "jpeg"
                webcam_img_msg.data = compressed_data.tobytes()

                # Publish the Webcam message
                pub_webcam.publish(webcam_img_msg)


                # Convert image to bgr8 (uncompressed and publish)
                # ros_image = bridge.cv2_to_imgmsg(resized_img, encoding="bgr8")
                # pub.publish(ros_image)
                # print("Image Dim:", ros_image.width, ros_image.height)

        print(i)
        rate.sleep()

            

            # Save the image from the GetImage request to the current directory with the filename
            # matching that of the image source.
            # image_saved_path = image.source.name
            # image_saved_path = image_saved_path.replace(
            #     '/', '')  # Remove any slashes from the filename the image is saved at locally.
            # cv2.imwrite(image_saved_path + extension, img)
    cap.release()
    cv2.destroyAllWindows()
    return True


if __name__ == '__main__':
    try:
        if not main(sys.argv[1:]):
            sys.exit(1)
    except Exception as exc:
        print("Exception thrown", exc)
        sys.exit(1)
