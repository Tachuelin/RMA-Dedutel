#!/usr/bin/env python3
# encoding: utf-8

#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8


import rospy                                                    #Importamos ropsy (interface de python-ROS)
from sensor_msgs.msg import LaserScan                           #Importamos el tipo de mensaje Lasersan
from geometry_msgs.msg import Twist                             #Importamos el tipo de mensaje Twist
import random

#Definimos Constantes
MIN_DIST = 1.0      #Distancia Mínima para detectar obstáculos = 1 metro 
MAX_DIST = 3.5      #Distancia Máxima = 3.5 metros

#Definimos Variables
left = ahead = right = 1.0  #valores mínimos de cada región del láser
linear_x = 0.5              #velocidad lineal = 0.5 m/s
angular_z = 0.5             #velocidad angular = 0.5 rad/seg

'''
 Función callback - Procesa las 240 muestras del láser para detectar obstáculos
 y nos devuelve el valor mínimo de las áreas de la derecha,izquierda y enfrente 
 del láser del robot

'''

def callback(mensaje):

    #Declaración de variables globales
    global left, ahead, right
    
    scan_range = [] #Definimos una lista vacía

    for i in range(len(mensaje.ranges)):
        if mensaje.ranges[i] == float('Inf'): #Si el sensor no detecta nada 
            scan_range.append(MAX_DIST)        #le asigamos un valor de 3.5m
        else:
            scan_range.append(mensaje.ranges[i]) #Si tiene un valor, lo agregamos a nuestra lista scan_range

    #Regiones = 240/3 = 80 muestras

    left = min(scan_range[0:79]) #muestras de 0:79
    ahead = min(scan_range[80:159]) #muestras de 80:159
    right = min(scan_range[160:239]) #muestras de 160:239

    # print("Left = %f , Ahead = %f , Right = %f" % (left,ahead,right))

    
def nodo():                                    # Definimos una función nodo

    rospy.init_node('nodo_detect_avoid')   # Inicializamos nuestro nodo y le asignamos un nombre = nodo_detect_obstacles

    
    '''
    Para procesar la data del láser del robot nos subcribimos al tópico /base_scan
    Creamos la variable scan_sub que es de tipo Subscriber
    '''
                                #Name Topic|tipo de mensaje|función
    scan_sub = rospy.Subscriber('/base_scan', LaserScan, callback)

    '''
    Como nuestro robot va a realizar movimientos, creamos la variable velocity_publisher que es de tipo publisher
    y publicamos el tópico /base_controller/command
    '''   
                                             # Name Topic   |tipo de mensaje|límite de 10 mensajes en cola 
    velocity_publisher = rospy.Publisher('/base_controller/command', Twist, queue_size=10)

    rate = rospy.Rate(10) #10Hz

    while not rospy.is_shutdown():  

        vel_msg = Twist()       #Definimos una variable de tipo Twist

        action_description = ''  #Definimos una variable de tipo String, para almacenar el tipo de movimiento del robot
        
        #Condiciones del robot cuando detecta un obstáculo menor a 1 metro

        #1- El robot irá hacia adelante, por que el láser no detecta ningún obstáculo a una distancia menor a 1 metro.
        if (left > MIN_DIST and ahead > MIN_DIST and right > MIN_DIST):
            action_description = 'Forward' #Asignamos el tipo de movimiento
            vel_msg.linear.x = linear_x    #Asignamos la velocidad lineal 

        #2- El robot gira hacia la derecha, por que el láser detecta un obstáculo 
        #   a una distancia menor a 1 metro a la izquierda del robot.
        elif left < ahead and left < right and left < MIN_DIST:
            action_description = 'Turn right'   #Asignamos el tipo de movimiento
            vel_msg.linear.x = linear_x     #Asignamos la velocidad lineal 
            vel_msg.angular.z = -angular_z  #Asignamos la velocidad angular

        #3- El robot dará marcha atrás, porque el láser detecta un obstáculo 
        #   a una distancia menor a 1.0 metro, enfrente del robot.
        elif ahead < left and ahead < right and ahead < MIN_DIST:
            action_description = 'Backforward' #Asignamos el tipo de movimiento
            vel_msg.linear.x = -linear_x - 1.0             
            if random.random() > 0.5:
                vel_msg.angular.z = -angular_z - 1.0    #Gira hacia la derecha                           
            else:
                vel_msg.angular.z = angular_z + 1.0     #Gira hacia la izquierda          

        #4- El robot gira hacia la izquierda, por que el láser detecta un obstáculo 
        #   a una distancia menor a 1.0 metro a la derecha del robot.
        elif right < left and right < ahead and right < MIN_DIST:
            action_description = 'Turn left' #Asignamos el tipo de movimiento
            vel_msg.linear.x = linear_x     #Asignamos la velocidad lineal
            vel_msg.angular.z = angular_z   #Asignamos la velocidad angular

        else:
            action_description = 'unknown case' #Asignamos el tipo de movimiento

        rospy.logwarn(action_description)    #Imprimimos el tipo de movimiento en pantalla

        velocity_publisher.publish(vel_msg)  #publicamos nuestro mensaje vel_msg para mover la base del robot
                                                    
        rate.sleep()                         #Loop 10 times per second


if __name__ == '__main__':                   # Llamamos a la función principal main
    try:
        nodo()                               # Lamamos a la función nodo    
    except rospy.ROSInterruptException :     # Check si hay una excepción - Ctrl-C para terminar la ejecución del nodo
        pass

