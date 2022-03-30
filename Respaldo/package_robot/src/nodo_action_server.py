#!/usr/bin/env python3                         
# encoding: utf-8
    
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
    
    
import rospy                                                        #Importamos ropsy (interface de python-ROS) 
    
import actionlib                                                    #Importamos actionlib para hacer uso de acciones en ROS
    
from package_robot.msg import DoCarWashAction,DoCarWashFeedback,DoCarWashResult #Importamos los mensajes de nuestra acción
    
class DoActionServer:
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer('do_wash_car', DoCarWashAction, self.execute, False)  #Declaramos nuestra Acción Server con nombre do_wash_car        
        self.server.start()
        print("Running action server do_wash_car ...")
        
    def execute(self, goal):
    
        feedback = DoCarWashFeedback()                           #Declaramos una variable de tipo Feedback
        result = DoCarWashResult()                               #Declaramos una variable de tipo Result
        rate = rospy.Rate(1)                                     #Loop 1hz
    
        for x in range(0,goal.number_of_cars):
            result.total_cars_cleaned += 1
            feedback.percent_cars_complete = (result.total_cars_cleaned*100.0)/goal.number_of_cars
            self.server.publish_feedback(feedback)                #Publicamos el feedback
            rate.sleep()
    
        self.server.set_succeeded(result)                         #Publicamos el resultado
    
    
if __name__ == '__main__':
    rospy.init_node('nodo_action_server')
    server = DoActionServer()                                      #Creamos una instancia de la Clase DoActionServer
    rospy.spin()                                                   #Mantiene corriendo el script hasta que se detiene la ejecución del script con Crtl+C