#!/usr/bin/env python
import rospy
from math import pi, sin, cos, acos
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/mira/joint1_position_controller/command
"""

def MiraJointMover():

    rospy.init_node('joint_mover', anonymous=True)
    rospy.loginfo("Mira JointMover Initialising...")
    joint_mover_topic_name = '/mrm/joint1_position_controller/command'
    move_base = rospy.Publisher(joint_mover_topic_name, Float64, queue_size=1)
    rospy.on_shutdown(onShutdown)
    loop(move_base)

def loop(topic):
    angulo = Float64()

    while True:
        print '#' * 25
        print 'Enter a value to publish'
        entrada = raw_input('>>> ')
        if entrada == 'Q' or entrada == 'q':
            break
        else:
            try:
                angulo.data = float(entrada)
                for i in range(4):
                    topic.publish(angulo)
                    rospy.sleep(0.2)
            except:
                rospy.loginfo('inserte un valor valido')
        
def onShutdown():
    rospy.loginfo('Cerrando programa')

if __name__ == "__main__":
    MiraJointMover()