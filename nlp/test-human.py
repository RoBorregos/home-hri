#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

# ESTE CODIGO SIMULA LA ENTRADA DE TEXTO POR PARTE DE UN USUARIO
# PARA PROBAR EL FUNCIONAMIENTO DE LOS NODOS DE NLP

def voz_humana():
    rospy.init_node('voz_humana', anonymous=True)
    pub = rospy.Publisher('/RawInput', String, queue_size=10)
    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        user_input = input()
        pub.publish(user_input)
        
        rate.sleep()
        

if __name__ == '__main__':
    voz_humana()
