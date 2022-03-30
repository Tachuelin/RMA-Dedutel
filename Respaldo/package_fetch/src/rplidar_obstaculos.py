#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan



MIN_DIST = 1.0      #Distancia Mínima para detectar obstáculos = 1 metro 
MAX_DIST = 3.5   

#Definimos Variables
left = ahead = right = 1.0  #valores mínimos de cada región del láser
           
           

def callback(mensaje):

    #Declaración de variables globales
    global left, ahead, right
    
    scan_range = [] #Definimos una lista vacía

    for i in range(len(mensaje.ranges)):
        if mensaje.ranges[i] == float('Inf'): #Si el sensor no detecta nada 
            scan_range.append(MAX_DIST)        #le asigamos un valor de 3.5m
        else:
            scan_range.append(mensaje.ranges[i]) #Si tiene un valor, lo agregamos a nuestra lista scan_range

    left = min(scan_range[0:120.98]) #muestras de 0:79
    ahead = min(scan_range[239.02:299.01]) #muestras de 80:159
    right = min(scan_range[300.01:359]) #muestras de 160:239

    print("Left = %f , Ahead = %f , Right = %f" % (left,ahead,right))

def nodo():

    rospy.init_node('nodo_detector_obstaculos', anonymous=True)

    #para procesar la data del laser rplidar nos suscribimos al topico /base_scan
    #creamos la varobale scan_sub que es de tipo Subscriber

    scan_sub = rospy.Subscriber('base_scan', LaserScan, callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        #EL robot ira hacia adelante, por lo que el laser no detecta ningun obstaculo
        if (left > MIN_DIST and ahead  > MIN_DIST and right > MIN_DIST):
            print('sin obstaculos')
        
        #El robot ira hacia la derecha, por que el laser detecta un obstaculo 
        # a una distancia menos a 1 metro a la izquierda del robot
        elif (left < ahead and left < right and left < MIN_DIST):
            print ('RMA irá hacia la derecha')
        
        #El robot ira hacia atras, por que el laser detecta un obstaculo
        # a una distancia menos de 1 metro enfrente del robot
        elif (ahead < left and ahead < right and ahead < MIN_DIST):
            print('RMA ira hacia atras')

        #EL robot ira hacia la izquierda, por que el laser detecta un obstaculo
        # a una distancia menos de 1 metro a la derecha del robot
        elif (right < left and right < ahead and right < MIN_DIST):
            print('RMA ira hacia la izquierda')

        rate.sleep()
        rospy.spin()

if __name__=='__main__':
    nodo()

  
