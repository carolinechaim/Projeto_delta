#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mobilenet_simples as mnet



def processa(frame, objeto):

    '''Use esta funcao para basear o processamento do seu robo'''

    mob_slice = None

    result_frame, LOCALIZADO, startX, startY, endX, endY = mnet.detect(frame, objeto)

    centro = (frame.shape[1]//2, frame.shape[0]//2)

    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - int(length/2), point[1]),  (point[0] + int(length/2), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - int(length/2)), (point[0], point[1] + int(length/2)),color ,width, length)

    cross(result_frame, centro, [255,0,0], 1, 17)

    if (result_frame is not None) and LOCALIZADO:
        mob_slice = result_frame[startY:endY,startX:endX]
        cv2.imshow("MOB", mob_slice)
        cv2.waitKey(1)

    return  LOCALIZADO
