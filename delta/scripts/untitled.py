
#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Sugerimos rodar com:
# roslaunch turtlebot3_gazebo  turtlebot3_empty_world.launch 
#
# Esta solução pode ser vista em: https://youtu.be/GKDZPcwf2WU


from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from tf import transformations


x = None
y = None
rad_z = 0.0

contador = 0
pula = 50

def recebe_odometria(data):
    global x
    global y
    global rad_z
    global contador

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos_rad = transformations.euler_from_quaternion(lista)
    rad_z = angulos_rad[2]
    angulos = np.degrees(angulos_rad)    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

v_ang = 0.3
v_lin = 0.3


def go_to(x1, y1, pub):
    x0 = x # Vai ser atualizado via global e odometria em um thread paralelo
    y0 = y # global e odometria (igual ao acima)
    delta_x = x1 - x0
    delta_y = y1 - y0

    h = math.sqrt(delta_x**2 + delta_y**2) # Distancia ate o destino. Veja 
    # https://web.microsoftstream.com/video/f039d50f-3f6b-4e01-b45c-f2bffd2cbd84

    while h > 0.3:      
        print("Goal ", x1,",",y1)
        # Rotacao
        ang_goal = math.atan2(delta_y,delta_x)  
        ang_atual = rad_z # rad_z muda automaticamente via global e odometria
        dif_ang = ang_goal - ang_atual
        delta_t = abs(dif_ang)/v_ang
        # Twist
        if dif_ang > 0.0:
            vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,v_ang))
        elif dif_ang <=0:
            vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,-v_ang))    
        # publish
        pub.publish(vel_rot)
        # sleep
        rospy.sleep(delta_t)
        zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
        pub.publish(zero)
        rospy.sleep(0.1)
        # Translacao
        delta_t = h/v_lin
        linear = Twist(Vector3(v_lin,0,0), Vector3(0,0,0))
        pub.publish(linear)
        rospy.sleep(delta_t)
        pub.publish(zero)
        rospy.sleep(0.1)  
        x0 = x
        y0 = y
        delta_x = x1 - x0
        delta_y = y1 - y0
        h = math.sqrt(deltScreencast)

if __name__=="__main__":

    rospy.init_node("q3")

    # Vertices do triangulo
    lado = 3.5
    # O (0,0) foi incluido duas vezes. Isso nao eh estritamente necessario
    verts = [(0,0), (-lado/2,0), (0, lado*math.sqrt(3)/2.0),(lado/2,0), (0,0)]


    # Velocidades
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))

    linear = Twist(Vector3(v_lin,0,0), Vector3(0,0,0))
    

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    velocidade_saida.publish(zero)
    rospy.sleep(1.0) # contorna bugs de timing    

    while not rospy.is_shutdown():
        rospy.sleep(0.5)    
        for p in verts:
            go_to(p[0],p[1], velocidade_saida)
            rospy.sleep(1.0)