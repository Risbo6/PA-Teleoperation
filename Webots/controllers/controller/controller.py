from controller import Robot
import rospy
from std_msgs.msg import Int16, Float64, Float64MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from diff_drive import joystickToDiff
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

import utm
import math

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class robot_param:
    def __init__(self, speed=0.1, leftSpeed=0, rightSpeed=0, direction=1):
        self.speed = speed
        self.direction = direction
        self.leftSpeed = leftSpeed
        self.rightSpeed = rightSpeed

r_param = robot_param()

def wheel_speed(x):
    r_param.speed = x.data/100


def wheel_direction(x):
    r_param.direction = x.data
    change_speed()


def change_speed():
    if r_param.direction == 1:
        r_param.leftSpeed = 100*r_param.speed
        r_param.rightSpeed = 100*r_param.speed

    if r_param.direction == 2:
        r_param.leftSpeed = -100*r_param.speed
        r_param.rightSpeed = -100*r_param.speed

    if r_param.direction == 3:
        r_param.leftSpeed  =  40*r_param.speed
        r_param.rightSpeed = -40*r_param.speed

    if r_param.direction == 4:
        r_param.leftSpeed  = -40*r_param.speed
        r_param.rightSpeed =  40*r_param.speed

def joy_callback(data): 
    #r_param.leftSpeed, r_param.rightSpeed = joystickToDiff(data.axes[3]*0.05, data.axes[1], -1, 1, -100, 100) #bride le mouvement en x a 25% pour tourner moins vite
    #r_param.leftSpeed = r_param.leftSpeed*r_param.speed
    #r_param.rightSpeed = r_param.rightSpeed*r_param.speed
    pass

def odometry_callback(data):
    r_param.leftSpeed, r_param.rightSpeed = joystickToDiff(data.data[1]*0.05, data.data[0], -1, 1, -100, 100) #bride le mouvement en x a 25% pour tourner moins vite
    r_param.leftSpeed = r_param.leftSpeed*r_param.speed
    r_param.rightSpeed = r_param.rightSpeed*r_param.speed



        
rospy.init_node('listener', anonymous=True)

rospy.Subscriber("/direction", Int16, wheel_direction)
rospy.Subscriber("/speed", Int16, wheel_speed)
rospy.Subscriber("/joy", Joy, joy_callback)
rospy.Subscriber("/odometry", Float64MultiArray, odometry_callback)

pub_yaw = rospy.Publisher("/yaw", Float64, queue_size=10)
pub_pitch = rospy.Publisher("/pitch", Float64, queue_size=10)
pub_roll = rospy.Publisher("/roll", Float64, queue_size=10)
pub_accel = rospy.Publisher("/accel", Float64, queue_size=10)
pub_currentspeed = rospy.Publisher("/currentspeed", Float64, queue_size=10)

pub_gps = rospy.Publisher("/gps", Float64MultiArray, queue_size=10)
gps_data = Float64MultiArray()



TIME_STEP = 64
robot = Robot()

wheels = []
wheelsNames = ['FL_WHEEL', 'BL_WHEEL', 'L_WHEEL','FR_WHEEL', 'BR_WHEEL', 'R_WHEEL']

inertial_unit = robot.getDevice('inertial_unit')
inertial_unit.enable(TIME_STEP)

front_lidar = robot.getDevice('front_lidar')
front_lidar.enable(TIME_STEP)
front_lidar.enablePointCloud()

accelerometer = robot.getDevice('accelerometer')
accelerometer.enable(TIME_STEP)

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)



#img = mpimg.imread('/home/boris/catkin_ws/src/gui/scripts/lena.png')
#imgplot = plt.imshow(img)
#plt.show()



image_pub = rospy.Publisher("/image_topic", Image, queue_size=10)
bridge = CvBridge()
#img = cv2.imread('/home/boris/catkin_ws/src/gui/scripts/lena.png',0)



for i in range(6):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


x = y = x_old = y_old = 0

while robot.step(TIME_STEP) != -1:
    wtf = front_lidar.getPointCloud('buffer')

    #print(gps.getValues())

    pub_roll.publish(inertial_unit.getRollPitchYaw()[0])
    pub_yaw.publish(inertial_unit.getRollPitchYaw()[2])
    pub_pitch.publish(inertial_unit.getRollPitchYaw()[1])
    pub_accel.publish(accelerometer.getValues()[0])

    gps_data.data = gps.getValues()
    pub_gps.publish(gps_data)

    wheels[0].setVelocity(r_param.leftSpeed)
    wheels[1].setVelocity(r_param.leftSpeed)
    wheels[2].setVelocity(r_param.leftSpeed)
    
    wheels[3].setVelocity(r_param.rightSpeed)
    wheels[4].setVelocity(r_param.rightSpeed)
    wheels[5].setVelocity(r_param.rightSpeed)
    

    img = np.array(camera.getImageArray(), dtype=np.uint8)
    image_message = bridge.cv2_to_imgmsg(img)
    image_pub.publish(image_message)


    #Estimation de la vitesse avec latitude/longitude
    x, y,_,_ = utm.from_latlon(gps.getValues()[0], gps.getValues()[1])

    v_x = x - x_old
    v_y = y - y_old

    x_old = x
    y_old = y

    v = math.sqrt(v_x**2 + v_y**2)*10 #Estimation
    pub_currentspeed.publish(v)

    print(v)

    
    


   

    

    

    
    

