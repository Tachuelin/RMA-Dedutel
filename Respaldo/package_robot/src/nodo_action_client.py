#!/usr/bin/env python3                         
# encoding: utf-8
    
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
    
import rospy                                                    #Importamos ropsy (interface de python-ROS) 
import actionlib                                                #Importamos actionlib para hacer uso de acciones en ROS
from package_robot.msg import DoCarWashAction, DoCarWashGoal    #Importamos los mensajes de nuestra acción
    
    
def feedback_cb(msg):                                           #Definimos una función feedback_cb
    
    print('Feedback received -> '+str(msg)+'%')                 #Imprimimos en pantalla el feebback que envía el Action Server
    
    
def call_server():                                                        #Definimos una función call_server
    
    client = actionlib.SimpleActionClient('do_wash_car', DoCarWashAction) #Declaramos nuestra Acción Cliente con nombre do_wash_car        
    
    client.wait_for_server()                                              #Si el Action Server no está disponible; esperamos
    
    goal = DoCarWashGoal()                                                #Definimos nuestra variable de tipo Goal
    
    goal.number_of_cars = 20                                               #Definimos el número de Automóviles
    
    client.send_goal(goal, feedback_cb=feedback_cb)                       #Enviamos nuestro objetivo, y pasamos una función de Feedback
    
    client.wait_for_result()                                              #Esperamos el resultado hasta que el Action Server procese todo
    
    result = client.get_result()                                          #Obtenemos el resultado
    
    return result                                                         #Retornamos el resultado
    
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('nodo_action_client')                            #Definimos el nombre de nuestro nodo
    
        result = call_server()                                           #Llamamos a nuestra función call_server()
    
        print("The result is: ", result)                                 #Imprimimos en pantalla el resultado
    
    except rospy.ROSInterruptException :                                 #Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass