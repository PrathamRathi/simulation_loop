from PIL import Image
from numpy import asarray
import os
import rospy
import math
from somthing.msg import array
import autopy

INPUT_DIRECTORY = "/home/pratham-rathi/Unity-Robotics-Hub/ROSTesting/Assets/Images"
REST_TIME = 5

def numpyPub():
    #defines a topic and sends it the message file
    pub = rospy.Publisher('numpyTopic', array, queue_size = 10)

    #gives a name to our node
    rospy.init_node('numpyPub', anonymous=True)

    autopy.key.toggle('f9', True)
    autopy.key.toggle('f9', False)
    msg_to_publish = getArr()
    #sets node frequency at 5hz
    rate = rospy.Rate(REST_TIME)

    pub.publish(msg_to_publish)

    rate.sleep()
    
    

def getArr():
    # load the image
    os.chdir(INPUT_DIRECTORY) 

    for imageName in os.listdir(INPUT_DIRECTORY):  
        image = Image.open('imageName')
        data = asarray(image)
        os.remove(imageName)
    
    return data

if __name__ == "__main__":
    try:
        numpyPub()
    except rospy.ROSInterruptException:
        pass


#USE FOR LATER
#print(type(data))
# summarize shape
#print(data.shape)

# create Pillow image
#image2 = Image.fromarray(data)
#print(type(image2))
# summarize image details
#print(image2.mode)
#print(image2.size)
