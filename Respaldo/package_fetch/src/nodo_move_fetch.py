#!/usr/bin/env python3                         
# encoding: utf-8
    
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
    
    
import rospy                                  #Importamos ropsy (interface de python-ROS) 
from geometry_msgs.msg import Twist           #Importamos el tipo de mensaje Twist                                                          
    
    
def nodo():                                   #Definimos una función nodo                                   
    
    rospy.init_node('nodo_move_base')         #Inicializamos nuestro nodo y le asignamos un nombre = nodo_move_base
            
                                                # Name Topic   |tipo de mensaje|límite de 10 mensajes en cola
    velocity_publisher = rospy.Publisher('/base_controller/command',Twist, queue_size=10)   
                                                                                            
    vel_msg = Twist()         #Definimos una variable de tipo Twist
    
    vel_msg.linear.x = -0.3    #Asignamos una velocidad lineal de 0.5 m/s
    vel_msg.angular.z = 0.4   #Asignamos una velocidad angular de 0.1 rad/seg
    
    rate = rospy.Rate(10)     #Crea un objeto Rate a 10hz
    
    while not rospy.is_shutdown():              #Bucle While
            
        velocity_publisher.publish(vel_msg)     #publicamos nuestro mensaje vel_msg para mover la base del robot                                                #Publicamos un mensaje de tipo String en nuestro topico example 
        
        rate.sleep()                            #Loop 10 times per second           
    
if __name__ == '__main__':                                  
    try:
        nodo()                                 # Lamamos a la función nodo
    except rospy.ROSInterruptException :       # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
            pass