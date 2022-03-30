#!/usr/bin/env python3                         
# encoding: utf-8
     
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
#para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
     
     
import rospy                                                #Importamos ropsy (interface de python-ROS) 
from std_msgs.msg import String                             #Importamos tipo de mensaje String
     
     
def callback(mensaje):
    rospy.loginfo("I heard %s", mensaje.data)
     
def nodo():                                                 #Definimos una función nodo                                   
        
    rospy.init_node('nodo_subscriber')                      #Inicializamos nuestro nodo y le asignamos un nombre = transmisor    
            
    rospy.Subscriber("example", String, callback)           #Realizamos la subscripción al tópico example con tipo de mensaje String     
        
    rospy.spin()                                            #Mantiene corriendo el script hasta que se detiene la ejecución con Crtl+C
     
if __name__ == '__main__':                                  #Llamamos a la función principal main
    try:
        nodo()                                              # Lamamos a la función nodo
    except rospy.ROSInterruptException:                     # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass