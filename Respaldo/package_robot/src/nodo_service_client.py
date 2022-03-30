#!/usr/bin/env python3                         
# encoding: utf-8
    
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
    
    
import rospy                              #Importamos ropsy (interface de python-ROS) 
from package_robot.srv import SumaTwoInts #Importamos el modulo SumaTwoInts generado por nuestro servicio 
    
    
def add_two_ints_client(x, y):               #Definimos una función para enviar la data al Service Server y obtener el resultado
    rospy.wait_for_service('suma_two_ints')  #Esperamos el servicio si no está listo                                                                
    try:
        #Definimos el Servicio Cliente en la variable add_two_ints
                                            #Name Service|Clase Servicio
        add_two_ints = rospy.ServiceProxy('suma_two_ints', SumaTwoInts)  
        resp = add_two_ints(x, y)       #Enviamos la data para ser procesada en el Service Server
        return resp.sum                 #Retornamos el resultado de la operación
    except rospy.ServiceException as e :    #Si hay una excepción durante el procesamiento lo imprimimos
        print("Service call failed: %s"%e)
    
def nodo():                                       #Definimos una función nodo                                   
    
    rospy.init_node('nodo_suma_two_ints_client')  #Inicializamos nuestro nodo y le asignamos un nombre = nodo_suma_two_ints_client
    
    #Definimos dos variables x & y para realizar la suma de los dos números enteros
    x = 12                                         
    y = 8
    
    print("Requesting %s+%s"%(x, y))           #Imprimimos en pantalla las variables que vamos a sumar
    
    #Imprimimos el resultado de la operación de los dos números enteros
    #La operación de suma la relizamos en la función add_two_ints_client
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
    
    
if __name__ == '__main__':                                  
    try:
        nodo()                               # Lamamos a la función nodo
    except rospy.ROSInterruptException:      # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass