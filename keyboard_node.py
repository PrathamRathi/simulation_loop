#!/usr/bin/env python

#https://github.com/tuomasjjrasanen/python-uinput/blob/master/examples/keyboard.py
#https://stackoverflow.com/Questions/5714072/simulate-keystroke-in-linux-with-python

#import stuff
import rospy
import math
from simulation_loop.msg import joystick_values
import autopy, time

def publisher():
    #defines a topic and sends it the message file
    pub = rospy.Publisher('jstick_values', joystick_values, queue_size = 10)

    #gives a name to our node
    rospy.init_node('publisher', anonymous=True)

    #sets node frequency at 5hz
    rate = rospy.Rate(5)

    
    msg_to_publish = joystick_values()
    
    
    while(not rospy.is_shutdown):
        if abs(x_vector) > .5 and abs(y_vector) > .5:
            #take in float_x and float_y from orange demo
            x_vector = 0
            y_vector = 0
            
            #Apply scalars
            x_value = 1 * x_vector
            y_value = 1 * y_vector

            move_x(x_value)
            move_y(y_value)
            
            msg_to_publish.x = x_value
            msg_to_publish.y = y_value
            pub.publish(msg_to_publish)
            
        rate.sleep()

def move_x(x_change):
    x_button = 'd'
    if(x_change < 0):
        x_button = 'a'

    autopy.key.toggle(x_button, True)
    time.sleep(abs(x_change))
    autopy.key.toggle(x_button, False)

def move_y(y_change):
    y_button = 'w'
    if(y_change < 0):
        y_button = 's'

    autopy.key.toggle(y_button, True)
    time.sleep(abs(y_change))
    autopy.key.toggle(y_button, False)

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass


