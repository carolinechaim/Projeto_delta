#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy

import numpy as np

from numpy import array, uint8

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import math


ranges = None
minv = 0
maxv = 10

distancia = 100

bridge = CvBridge()

goal = "blue"

# Limiares obtidos via color picker
rl1, rl2 = (array([0,  80,  20], dtype=uint8), array([5, 255, 255], dtype=uint8)) # red lower
ru1, ru2 = (array([175,  80,  20], dtype=uint8), array([180, 255, 255], dtype=uint8)) # red upper
g1, g2 = (array([45, 80, 20], dtype=uint8), array([ 65, 255, 255], dtype=uint8))
b1, b2 = (array([105,  80,  20], dtype=uint8), array([125, 255, 255], dtype=uint8))

def mask(hsv, a1, a2):
    return cv2.inRange(hsv, a1, a2)


def auto_canny(image, sigma=0.5):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    #image = cv2.blur(image, ksize=(5,5)) # blur se necessário
    #cv2.imshow("filter", image)
    #cv2.waitKey(0)
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged


def maior_circulo(mask, color):
    """ Retorna os dados do maior círculo existente na imagem. E uma imagem com este círculo desenhado"""
    tem_circulo = False
    maior_centro = (0,0)
    maior_raio = 0
    
    bordas = auto_canny(mask)
    
    # acumulador levemente ajustado
    circles=cv2.HoughCircles(image=bordas,method=cv2.HOUGH_GRADIENT,dp=2.2,minDist=250,param1=50,param2=100,minRadius=30,maxRadius=100)
    
    bordas_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

    output =  bordas_bgr

    if circles is not None:        
        circles = np.uint16(np.around(circles))
        
        for i in circles[0,:]:
            tem_circulo = True
            # draw the outer circle
            cv2.circle(output,(i[0],i[1]),i[2],color,2)
            # draw the center of the circle
            cv2.circle(output,(i[0],i[1]),2,color,3)
            if i[2] > maior_raio:
                maior_centro = (int(i[0]), int(i[1]))
                maior_raio = i[2]
                                
    return tem_circulo, maior_centro, maior_raio, output


def processa_circulos_controle(img_bgr, OBJETO):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    blue = mask(hsv, b1, b2)
    green = mask(hsv, g1, g2)
    red_lower = mask(hsv, rl1, rl2)
    red_upper = mask(hsv, ru1, ru2)
    red = cv2.bitwise_or(red_lower, red_upper)
    cor_b = (255,0,0)
    cor_g = (0,255,0)
    cor_r = (0, 0,255)

    if OBJETO == "red_sphere":
        tem, centro, raio, img = maior_circulo(red, cor_r)

    elif OBJETO == "blue_sphere":
        tem, centro, raio, img = maior_circulo(blue, cor_b)

    elif OBJETO == "green_sphere":
        tem, centro, raio, img = maior_circulo(green, cor_g)

    else:
        img = None
        tem = False
        centro = (0,0)
        raio = 0

    if (img_bgr is not None) and tem:

        x1 = centro[0] - raio
        y1 = centro[1] - raio
        x2 = centro[0] + raio
        y2 = centro[1] + raio

        sphere_slice = img_bgr[y1:y2, x1:x2].copy()

        if sphere_slice is not None:

            cv2.imshow("SPHERE", sphere_slice)
            cv2.waitKey(1)

        else:
            print("retornando nulo")

    return tem  

    

# Variaveis de controle

circle_visible = False
circle_delta = 0


### Fim do trecho inserido no gabarito


def scaneou(dado):
    global ranges
    global minv
    global maxv
    global distancia
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max
    for m in ranges[0: 5]:
        if minv < m < maxv:
            if m < distancia:
                distancia = m    
    for m in ranges[355: 360]:
        if minv < m < maxv:  
            if m < distancia:
                distancia = m                  
 
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global circle_visible
    global circle_delta
    print("frame")
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv2.imshow("Camera", cv_image)
        tem_red, delta_red, img_r, tem_green, delta_green, img_g, tem_blue, delta_blue, img_b = processa_circulos_controle(cv_image)

        if goal == "blue":
            circle_visible = tem_blue
            circle_delta = delta_blue
            cv2.imshow("Blue", img_b)        
            cv2.waitKey(1)                        
        if goal == "green":
            circle_visible = tem_green
            circle_delta = delta_green
            cv2.imshow("Green", img_g)
            cv2.waitKey(1)
        if goal == "red":
            circle_visible = tem_red
            circle_delta = delta_red
            cv2.imshow("Red", img_r)
            cv2.waitKey(1)            

    except CvBridgeError as e:
        print('ex', e)
