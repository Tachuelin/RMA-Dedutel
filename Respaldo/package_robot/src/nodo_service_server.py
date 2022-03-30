#!/usr/bin/env python3                         
# encoding: utf-8
    
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
    
    
import rospy                                                                #Importamos ropsy (interface de python-ROS) 
    
from package_robot.srv import SumaTwoInts, SumaTwoIntsResponse              #Importamos módulos generados por nuestro servicio 
#from  package_robot.srv import SumarDos, SumaTwoIntsResponse
    
def handle_suma_two_ints(req):                                              #Definimos para procesar la data enviada por el Cliente
    print ("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))      #Imprimimos en pantalla los valores que recibimos
    return SumaTwoIntsResponse(req.a + req.b)                            #Retornamos al Cliente, el resultado de la suma de dos números enteros
    
def nodo():                                                                 #Definimos una función nodo                                   
    
    rospy.init_node('nodo_suma_two_ints_server')                            #Inicializamos nuestro nodo y le asignamos un nombre = nodo_suma_two_ints_server
    
    #Declaramos nuestro Servicio Server    
    #Name Service|Clase Servicio|Función para procesar la data enviada por el Cliente     
    s = rospy.Service('suma_two_ints', SumaTwoInts, handle_suma_two_ints)   
    
    print("Ready to add two ints.") #Imprimimos un mensaje en pantalla
    
    rospy.spin()                    #Mantiene corriendo el script hasta que se pulsa Crtl+C
    
if __name__ == '__main__':                                  
    try:
        nodo()                                                              # Lamamos a la función nodo
    except rospy.ROSInterruptException:                                     # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass