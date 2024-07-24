#!/usr/bin/env python

import serial
import rospy

from geometry_msgs.msg import Vector3

# Configura el puerto serial y la velocidad de baudios (ajusta según tu configuración)
ser = serial.Serial('/dev/ttyACM0', 9600)  # Reemplaza 'COM3' con el puerto serial correcto

def send_to_serial(Vector3):

    ctr_w_mr=Vector3.x
    ctr_w_ml=Vector3.y
    serial_ctr_data= f"{ctr_w_mr} {ctr_w_ml}/n"

    ser.write(serial_ctr_data.encode('utf-8'))


def serial_talker():

    rospy.init_node('serial_talker', anonymous=True) #inicio nodo
    pub = rospy.Publisher('/motorsVel', Vector3, queue_size=10) #genero un publisher que publique un topico la velocidades de los motores en rad/s

    rospy.Subscriber('/ctr_motorsVel',Vector3,send_to_serial) #genero un susbcriber que se subscriba a un topico y envie los datos por serial

    rate = rospy.Rate(50) # 50hz

    while not rospy.is_shutdown(): #loop

        if ser.in_waiting > 0: # evaluo si hay datos en el puerto
            data = ser.readline().decode('utf-8',errors='ignore').strip() # lee el dato, transforma a string, y elimina cualquier espacio en blaco al inicio o final
            
            try:

                w_mr, w_ml = map(float,data.split()) #divide la cadena y lo transforma a flotantante
                msg_vel=Vector3(x=w_mr,y=w_ml,z=0.0) #generamos el mensaje tipo vector3 y le asignamos sus datos respectivos
                pub.publish(msg_vel) #publicamos el topico con el mensaje creado
            except ValueError:
                rospy.logwarn("Reiceved Invalid Numbers: %s", data)

        rate.sleep() # tiempo de espera de 50hz

if __name__ == '__main__':
    try:
        serial_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close
        rospy.loginfo("Serial Port Close. Finish Comunication") # out screeen
