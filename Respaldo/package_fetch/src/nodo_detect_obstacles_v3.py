#!/usr/bin/env python3
# encoding: utf-8

#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8


import rospy                                                          # Importamos ropsy (interface de python-ROS)
from sensor_msgs.msg import LaserScan                                 #Importamas el tipo de mensaje Lasersan


def callback(mensaje):      #Definimos una función callback
    
    scan_range = [] #Definimos una lista vacia

    for i in range(len(mensaje.ranges)):
        if mensaje.ranges[i] == float('Inf'):    #Si el sensor no detecta nada 
            scan_range.append(3.5)               #le asigamos un valor de 3.5m
        else:
            scan_range.append(mensaje.ranges[i]) #Si tiene un valor, lo agregamos a nuestra lista scan_range

    #Regiones = 240/3 = 80 muestras

    left = min(scan_range[0:79])        #muestras de 0:79

    ahead = min(scan_range[80:159])     #muestras de 80:159

    right = min(scan_range[160:239])    #muestras de 160:239

    print("Left = %f , Ahead = %f , Right = %f" % (left,ahead,right))

    
def nodo():                                                             # Definimos una función nodo

    rospy.init_node('nodo_detect_obstacles')                            # Inicializamos nuestro nodo y le asignamos un nombre

    #Nos suscribimos al tópico /base_scan
                               #Name Topic|tipo de mensaje|función
    scan_sub = rospy.Subscriber('/base_scan', LaserScan, callback)

    rospy.spin()                                                        # Mantiene corriendo el script hasta que se detiene la ejecución del script con Crtl+C


if __name__ == '__main__':                                              # Llamamos a la función principal main
    try:
        nodo()                                                          # Lamamos a la función nodo    
    except rospy.ROSInterruptException :                                #Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass