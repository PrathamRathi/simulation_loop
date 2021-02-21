#!/usr/bin/env python

#import stuff
import rospy
import math
from simulation_loop.msg import joystick_values
import uinput

def keyPresser():
    #defines a topic and sends it the message file
    pub = rospy.Publisher('jstick_values', joystick_values, queue_size = 10)

    #gives a name to our node
    rospy.init_node('keyPresser', anonymous=True)

    #sets node frequency at 5hz
    rate = rospy.Rate(5)

    
    msg_to_publish = joystick_values()
    #take in float_x and float_y from orange demo
    x_vector = 0
    y_vector = 0

    #values that will be sent to the joystick
    x_value = 0
    y_value = 0

    events = (
        uinput.ABS_X + (0, 255, 0, 0),
        uinput.ABS_Y + (0, 255, 0, 0),
    )
    
    while(not rospy.is_shutdown):
        if abs(x_vector) > .5 and abs(y_vector) > .5:

            #Apply scalars
            x_value = 1 * x_vector
            y_value = 1 * y_vector

            with uinput.Device(events) as device:
                device.emit(uinput.ABS_X, x_value)
                device.emit(uinput.ABS_Y, y_value)
            
            msg_to_publish.x = x_value
            msg_to_publish.y = y_value
            pub.publish(msg_to_publish)
            
        rate.sleep()


if __name__ == "__main__":
    try:
        keyPresser()
    except rospy.ROSInterruptException:
        pass
