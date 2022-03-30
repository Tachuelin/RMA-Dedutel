#!/usr/bin/env python3                         
# encoding: utf-8
     
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
#para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
     
     
import rospy                                                #Importamos ropsy (interface de python-ROS) 
from std_msgs.msg import String                             #Importamos tipo de mensaje String
     
def nodo():                                                 #Definimos una función nodo                                   
        
    rospy.init_node('nodo_publisher')                       #Inicializamos nuestro nodo y le asignamos un nombre = nodo_publisher
        
    pub = rospy.Publisher('example', String, queue_size=10) #Definimos nuestro topico con nombre example y tipo de mensaje String
                                                                #con un límite de 10 mensajes en cola 
    rate = rospy.Rate(10)                                   #Crea un objeto Rate a 10hz (loop 10 times per second)
        
    while not rospy.is_shutdown():                          #Bucle While - hasta pulsar Ctrl-C
            
        mensaje = "Nodo Publisher"                          #Declaramos una variable mensaje y asignamos una cadena de caracteres
            
        rospy.loginfo(mensaje)                              #Imprime en pantalla mensajes logs de tipo Info
            
        pub.publish(mensaje)                                #Publicamos un mensaje de tipo String en nuestro tópico example 
            
        rate.sleep()                          
     
if __name__ == '__main__':                                  #Llamamos a la función principal main
    try:
        nodo()                                              # Lamamos a la función nodo
    except rospy.ROSInterruptException:                     # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass