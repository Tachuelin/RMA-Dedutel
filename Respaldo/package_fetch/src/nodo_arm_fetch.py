#!/usr/bin/env python3
# encoding: utf-8
 
#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8
 
import rospy      #Importamos ropsy (interface de python-ROS) 
import actionlib  #Importamos actionlib para hacer uso de acciones en ROS
 
#Importamos los mensajes de la acción FollowJointTrajectory - Action y Goal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
 
#Importamos los mensajes de tipo JointTrajectory - JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
 
#Importamos math para convertir Grados a Radianes en la generación de las trayectorias del Brazo del robot
import math
 
#Creamos una Lista de los nombres de las Articulaciones del Brazo del robot
arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
 
#Creamos 6 trayectorias, para el movimiento del brazo robot, los valores de cada articulación estan dado en radianes
arm_joint_positions0  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
arm_joint_positions1  = [math.radians(-80), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
arm_joint_positions2  = [0.0, math.radians(-70), 0.0, 0.0, 0.0, 0.0, 0.0]
arm_joint_positions3  = [0.0, 0.0, 0.0, math.radians(-100), 0.0, 0.0, 0.0]
arm_joint_positions4  = [0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-100), 0.0]
arm_joint_positions5  = [0.0, math.radians(-45), 0.0, math.radians(-45), 0.0, math.radians(90), 0.0]
 
 
if __name__ == "__main__":
    
    rospy.init_node("nodo_move_arm") #Definimos el nombre de nuestro nodo
 
    rospy.loginfo("Waiting for arm_controller...") #Imprimimos en pantalla 
 
                #Creamos una Acción Cliente|nombre de la acción :follow_joint_trajectory|especificación de la acción
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    
    arm_client.wait_for_server() #Si el Acción Server no está disponible, esperamos
 
    rospy.loginfo("...connected.") #Imprimimos en pantalla 
 
    '''
    trajectory_msgs/JointTrajectory.msg
        Header header
        string[] joint_names
        JointTrajectoryPoint[] points
    '''
    
    trajectory = JointTrajectory() #Declaramos una variable de tipo JointTrajectory.msg
 
    trajectory.joint_names = arm_joint_names  #Asignamos la lista de los nombres de las articulaciones del Brazo del Robot
    
    '''
    trajectory_msgs/JointTrajectoryPoint.msg
 
        Each trajectory point specifies either positions[, velocities[, accelerations]]
        or positions[, effort] for the trajectory to be executed.
        All specified values are in the same order as the joint names in JointTrajectory.msg
 
        float64[] positions
        float64[] velocities
        float64[] accelerations
        float64[] effort
        duration time_from_start
 
    '''
    #Agregamos al final de la Lista trajectory.points el tipo de mensaje JointTrajectoryPoint.msg
    trajectory.points.append(JointTrajectoryPoint())
    
    trajectory.points[0].positions = arm_joint_positions5 #Asignamos la trayectoria a ejecutar
    
    trajectory.points[0].velocities = [0.0] * len(arm_joint_positions0) #Asignamos los valores por default = 0
    
    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions0) #Asignamos los valores por default = 0
 
    trajectory.points[0].time_from_start = rospy.Duration(1.0) #Asignamos la duración de 1seg para iniciar
    
    arm_goal = FollowJointTrajectoryGoal() #Creamos una variable de tipo Goal
 
    arm_goal.trajectory = trajectory #Asignamos nuestra trayectoria hacia la trayectoria de nuestro objetivo(goal)
 
    arm_goal.goal_time_tolerance = rospy.Duration(0.0) #Asignamos la duración de 0seg como tiempo de tolerancia de nuestro objetivo
 
    rospy.loginfo("Setting positions...") #Imprimimos en pantalla 
    
    arm_client.send_goal(arm_goal) #Enviamos nuestro objetivo hacia el Acción Server de nuestro robot Fetch
 
    arm_client.wait_for_result(rospy.Duration(6.0)) #Esperamos 6 segundos para obtener los resultados
    
    rospy.loginfo("...done") #Si todo se ha realizado correctamente imprimimos en pantalla el mensaje done(hecho)