#!/usr/bin/env python3
# encoding: utf-8

#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 3 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script : encoding: utf-8

import rospy

def nodo():  # Definimos una función nodo

    rospy.init_node('nodo1')
    # Declaramos una variable mensaje y asignamos una cadena de caracteres
    mensaje = "Hola Mundo"

    #print(mensaje)  # Imprime en consola el valor de la variable mensaje

    rospy.loginfo(mensaje)

if __name__ == '__main__':
    try:
        nodo()  # Llamamos a la función nodo
    except rospy.ROSInterruptException:
        pass #Si hay una excepción lo imprimimos