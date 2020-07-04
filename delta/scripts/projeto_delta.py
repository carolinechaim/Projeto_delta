#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import visao_module

import detecta_esferas

global FINAL

####################
GOAL = ["red_sphere","bird"]
####################

REMOVE = ["red_sphere"]


PROCURANDO = False
LOCALIZADO_SPHERE = False
LOCALIZADO_MOB = False

REMOVE =[]

FINAL = False

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0




x_odon = 0
y_odon= 0
rad_z = 0.0

contador = 0
pula = 50

frame = "camera_link"


tfl = 0

tf_buffer = tf2_ros.Buffer()

def recebe(msg):
    global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
    global y
    global z
    global id
    for marker in msg.markers:
        id = marker.id
        marcador = "ar_marker_" + str(id)

        print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
        header = Header(frame_id=marcador)
        # Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
        # Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
        # Nao ser que queira levar angulos em conta
        trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
        
        # Separa as translacoes das rotacoes
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        # ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
        # Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
        # no eixo X do robo (que e'  a direcao para a frente)
        t = transformations.translation_matrix([x, y, z])
        # Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
        r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
        z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
        v2 = numpy.dot(m, z_marker)
        v2_n = v2[0:-1] # Descartamos a ultima posicao
        n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
        x_robo = [1,0,0]
        cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
        angulo_marcador_robo = math.degrees(math.acos(cosa))

        # Terminamos
        print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))






def recebe_odometria(data):
    global x_odon
    global y_odon
    global rad_z
    global contador

    x_odon = data.pose.pose.position.x
    y_odon = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos_rad = transformations.euler_from_quaternion(lista)
    rad_z = angulos_rad[2]
    angulos = np.degrees(angulos_rad)    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x_odon, y_odon,angulos[2]))
    contador = contador + 1

v_ang = 0.3
v_lin = 0.3



def go_to(x1, y1, pub, booleano):

    global PROCURANDO

    x0 = x_odon # Vai ser atualizado via global e odometria em um thread paralelo
    y0 = y_odon # global e odometria (igual ao acima)
    delta_x = x1 - x0
    delta_y = y1 - y0

    h = math.sqrt(delta_x**2 + delta_y**2) # Distancia ate o destino. Veja 
    # https://web.microsoftstream.com/video/f039d50f-3f6b-4e01-b45c-f2bffd2cbd84
    print("Goal ", x1,",",y1)
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))

    while h > 0.5:   

        PROCURANDO = False

        print("Goal ", x1,",",y1)
        # Rotacao
        ang_goal = math.atan2(delta_y,delta_x)  
        ang_atual = rad_z # rad_z muda automaticamente via global e odometria
        dif_ang = ang_goal - ang_atual
        delta_t = abs(dif_ang)/v_ang
        # Twist
        zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
        print (ang_atual)
        print (ang_goal)
        if dif_ang > 0.0:
            vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,v_ang))
        elif dif_ang <=0:
            vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,-v_ang))    
        # publish
        pub.publish(vel_rot)
        # sleep
        rospy.sleep(delta_t)
        
        pub.publish(zero)
        rospy.sleep(0.1)
        # Translacao
        delta_t = h/v_lin
        linear = Twist(Vector3(v_lin,0,0), Vector3(0,0,0))
        pub.publish(linear)
        rospy.sleep(delta_t)
        pub.publish(zero)
        rospy.sleep(0.1)
        x0 = x_odon
        y0 = y_odon
        delta_x = x1 - x0
        delta_y = y1 - y0
        h = math.sqrt(delta_x**2 + delta_y**2)

    print("no eixo correto")

    pub.publish(zero)
    rospy.sleep(0.1)

    if booleano:

        PROCURANDO = True

        print ("dando volta 360:")


        vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,v_ang))
        # publish
        pub.publish(vel_rot)
        # sleep
        rospy.sleep(math.pi * 2/v_ang)    





# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    #print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global LOCALIZADO_SPHERE
    global LOCALIZADO_MOB

    LOCALIZADO_SPHERE = False
    LOCALIZADO_MOB = False

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")


        depois = time.clock()

        saida_net = temp_image

        if FINAL:
            cv2.putText(temp_image,"FINAL DE SIMULACAO",(200,275), font,1,(255,255,255),2,cv2.LINE_AA)


        if PROCURANDO:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(temp_image,"Estado:",(0,415), font,1,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(temp_image,"     Andando",(0,445), font,1,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(temp_image," -> Giro 360 / Procurando",(0,475), font,1,(255,255,255),2,cv2.LINE_AA)
            
            for i in GOAL:
                LOCALIZADO_SPHERE = detecta_esferas.processa_circulos_controle(temp_image, i)
                if LOCALIZADO_SPHERE:
                    font = cv2.FONT_HERSHEY_SIMPLEX 
                    print ("objeto {obj} encontrado".format(obj = i))
                    if i in REMOVE:
                        REMOVE.remove(i)


                    #cv2.putText(temp_image,"objeto {obj} encontrado".format(obj = i),(0,30), font,1,(255,255,255),2,cv2.LINE_AA)
  
            for i in GOAL:

                LOCALIZADO_MOB =  visao_module.processa(temp_image,i) 

                if LOCALIZADO_MOB:
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    print ("objeto {obj} encontrado".format(obj = i))
                    if i in REMOVE:
                        REMOVE.remove(i)
                    #cv2.putText(temp_image,"objeto {obj} encontrado".format(obj = i),(0,50),font,1,(255,255,255),2,cv2.LINE_AA)
        else:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(temp_image,"Estado:",(0,415), font,1,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(temp_image," -> Andando",(0,445), font,1,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(temp_image,"     Giro 360 / Procurando",(0,475), font,1,(255,255,255),2,cv2.LINE_AA)

        cv_image = saida_net.copy()

        if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
            cv2.imshow("cv_image no loop principal", cv_image)
            cv2.waitKey(1)        

    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos


    verts = [(0.0 , 0.0, False),(0.0 , -3.5, True), (-3.0 , -3.5,False),(-6.0 , -3.5,False), (-9.0 , -3.5,True),(-12.0,-4,False),(-15.0,-4,False),
    (-18.0,-4,False), (-19.0,-4,False), (-19.0,0,True)]

    REMOVE = GOAL
    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    try:

        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

        while not rospy.is_shutdown():

            for r in resultados:
                print(r)

            for p in verts:
                GOAL = REMOVE
                if len(GOAL) == 0:
                    FINAL = True
                    print ("final de simulacao")
                else:

                    go_to(p[0],p[1], velocidade_saida, p[2])
                    rospy.sleep(1.0)
            velocidade_saida.publish(vel)

            #rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
