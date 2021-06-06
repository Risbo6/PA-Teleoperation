#!/usr/bin/env python3.8

import rospy
import math
import random
import time

from std_msgs.msg import Int16, Float64, Float64MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as ros_img

import kivy        
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.uix.image import Image
from kivy.uix.widget import Widget
from kivy.graphics.texture import Texture
from kivy.clock import Clock, mainthread
import cv2
from cv_bridge import CvBridge





    
class GUI(MDApp):
        #Variable d'instance
        def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.img = None #Image reçue de webots
                self.webcam = None #Image reçue de la webcam
                self.image_fps = 30 #FPS max de la video sur le GUI
                self.speed_limit = 0 #Speed limiter
                self.deadManSwitchMotion = False #Dead man switch pour le deplacement, touche R2
                self.odometry_data = Float64MultiArray()

                self.screen=Builder.load_file('/home/boris/catkin_ws/src/gui/scripts/gui.kv') #Fichier contenant les layout

                #Initialisation des differents subscriber
                rospy.Subscriber("/gps", Float64MultiArray, self.gps_callback) #Type array de float
                rospy.Subscriber("/yaw", Float64, self.yaw_callback)
                rospy.Subscriber("/pitch", Float64, self.pitch_callback)
                rospy.Subscriber("/roll", Float64, self.roll_callback)
                rospy.Subscriber("/currentspeed", Float64, self.currentspeed_callback)
                rospy.Subscriber('/joy', Joy, self.joy_callback)
                rospy.Subscriber("/image_topic", ros_img, self.ros_image_callback)
                rospy.Subscriber("/webcam_topic", ros_img, self.ros_webcam_callback)
                
                #Clock pour l'actualisation de l'image (live feed)
                Clock.schedule_interval(self.clock_callback, 1/self.image_fps)
                
                #Instance pour conversion image CV2-ROS
                self.bridge = CvBridge()


        #Voir .kv file, screen = root
        def build(self):
                self.theme_cls.theme_style = "Dark"
                #self.theme_cls.primary_palette = "Red" 
                return self.screen

        #Decorateur mainthread obligatoire. Si les callbacks sont appeles depuis un autre thread, ça
        #cause des problemes avec openGL et l'affichage freeze sur le GUI 
        @mainthread
        def yaw_callback(self, data):    
                self.screen.ids.top_axe.alpha = round(data.data*180/math.pi)

                #Affichage yaw sur GUI de -180 a 180 degres
                if data.data*180/math.pi > 90 and data.data*180/math.pi < 180 : 
                        self.screen.ids.yaw_deg.text = (f'{round(270-data.data*180/math.pi):.0f}°')

                elif data.data*180/math.pi > 0 and data.data*180/math.pi < 90 : 
                        self.screen.ids.yaw_deg.text = (f'{round(-90-data.data*180/math.pi):.0f}°')

                else :
                        self.screen.ids.yaw_deg.text = (f'{round(-90-data.data*180/math.pi):.0f}°')

                #Affichage du point cardinal sur le GUI
                if data.data*180/math.pi < 22.5 and data.data*180/math.pi > -22.5 :     
                        self.screen.ids.yaw_points_card.text = "North" 

                elif data.data*180/math.pi < 67.5 and data.data*180/math.pi > 22.5 :     
                        self.screen.ids.yaw_points_card.text = "North West" 

                elif data.data*180/math.pi < 112.5 and data.data*180/math.pi > 67.5 :     
                        self.screen.ids.yaw_points_card.text = "West" 

                elif data.data*180/math.pi < 157.5 and data.data*180/math.pi > 112.5 :     
                        self.screen.ids.yaw_points_card.text = "South West" 

                elif data.data*180/math.pi > -67.5 and data.data*180/math.pi < -22.5 :     
                        self.screen.ids.yaw_points_card.text = "North East" 

                elif data.data*180/math.pi > -112.5 and data.data*180/math.pi < -67.5 :     
                        self.screen.ids.yaw_points_card.text = "East" 

                elif data.data*180/math.pi > -157.5 and data.data*180/math.pi < -112.5 :     
                        self.screen.ids.yaw_points_card.text = "South East" 

                else : 
                        self.screen.ids.yaw_points_card.text = "South" 



                
        def gps_callback(self, data):
                self.screen.ids.MyMapView.center_on(data.data[0], data.data[1])
                self.screen.ids.MyMapView.current_lat = data.data[0]
                self.screen.ids.MyMapView.current_lon = data.data[1]

                self.screen.ids.latitude_label.text = (f'Latitude: {data.data[0]:.5f}')
                self.screen.ids.longitude_label.text = (f'Longitude: {data.data[1]:.5f}')

        @mainthread
        def currentspeed_callback(self, data):
                self.screen.ids.current_speed.prog_value = data.data*15
                self.screen.ids.current_speed_label.text = (f'{data.data:.1f} m/s')


        @mainthread
        def roll_callback(self, data):
                self.screen.ids.front_axe.alpha = round(-data.data*180/math.pi)
                self.screen.ids.roll_deg.text = (f'{round(-data.data*180/math.pi):.0f}°')


        @mainthread
        def pitch_callback(self, data):
                self.screen.ids.side_axe.alpha = round(-data.data*180/math.pi)
                self.screen.ids.pitch_deg.text = (f'{round(data.data*180/math.pi):.0f}°')


        @mainthread
        def joy_callback(self, data): 
                ##Dead man switch LT
                if data.axes[2] < 0 :
                        self.screen.ids.left_joystick.joy_inside_color = 0, 1, 0, 0.5
                        self.screen.ids.right_joystick.joy_inside_color = 0, 1, 0, 0.5
                        self.deadManSwitchMotion = True # deadmanswtich appuye
                else : 
                        self.screen.ids.left_joystick.joy_inside_color = 1, 0, 0, 0.5
                        self.screen.ids.right_joystick.joy_inside_color = 1, 0, 0, 0.5
                        self.deadManSwitchMotion = False # deadmanswtich relarche


                ##Affichage des joysticks
                self.screen.ids.left_joystick.joystick_x = 0
                self.screen.ids.left_joystick.joystick_y = data.axes[1]
                #self.screen.ids.left_joystick.text = (f'[{-data.axes[0]:.2f}, {data.axes[1]:.2f}]')
                self.screen.ids.right_joystick.joystick_x = -data.axes[3]
                self.screen.ids.right_joystick.joystick_y = 0


                ## Envoi des consignes de vitesse lin et rot, seulement si dead man switch presse
                if self.deadManSwitchMotion == True :
                        self.odometry_data.data = data.axes[1], data.axes[3]
                        pub_odometry.publish(self.odometry_data)
                else :
                        self.odometry_data.data = 0,0
                        pub_odometry.publish(self.odometry_data)
                        pass



                ##Reglage du speed limiter
                if round(data.axes[6]) == -1 :
                        self.speed_limit += 10
                        if self.speed_limit > 100 :
                                self.speed_limit = 100


                if round(data.axes[6]) == 1 :
                        self.speed_limit -= 10
                        if self.speed_limit < 0 :
                                self.speed_limit = 0

                self.screen.ids.speed_limiter.prog_value = self.speed_limit
                self.screen.ids.speed_limit_label.text = (f'{self.speed_limit:.0f}%')
                pub_speed.publish(self.speed_limit)

        
        def screenshoot_press(self, *args):
                path = '/home/boris/Pictures/screenshot' + str(random.randrange(1e8,1e9)) + '.jpg'

                if self.img is not None :
                        screenshot = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
                        screenshot = cv2.flip(screenshot, 1)
                        cv2.imwrite(path, screenshot)
                        self.screen.ids.screenshot_button.text = 'Saved as ' + path
                        
                        


        def slider_speed(self, slider_value):
                pub_speed.publish(int(slider_value))


        def ros_image_callback(self, img_msg):
                self.img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

        
        def ros_webcam_callback(self, img_msg):
                self.webcam = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        def clock_callback (self, dt):
                img = self.img
                webcam = self.webcam

                if img is not None : #Vrai seulement lorsqu'une image est reçue depuis le robot
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        img = cv2.flip(img, 0)
                        img = cv2.flip(img, 1)
                        w_img, h_img, _ = img.shape  
                        texture_img = Texture.create(size=(h_img, w_img))
                        texture_img.blit_buffer(img.flatten(), colorfmt='rgb', bufferfmt='ubyte')
                        self.screen.ids.im2.texture = texture_img

                if webcam is not None : #Vrai seulement lorsqu'une image est reçue depuis la webcam
                        webcam = cv2.cvtColor(webcam, cv2.COLOR_BGR2RGB)
                        webcam = cv2.flip(webcam, 0)
                        webcam = cv2.flip(webcam, 1)
                        w_webcam, h_webcam, _ = webcam.shape  
                        texture_webcam = Texture.create(size=(h_webcam, w_webcam))
                        texture_webcam.blit_buffer(webcam.flatten(), colorfmt='rgb', bufferfmt='ubyte')
                        self.screen.ids.im1.texture = texture_webcam






                


if __name__ == '__main__':
        GUI = GUI()
        rospy.init_node('simple_gui', anonymous=True)
        pub_speed = rospy.Publisher('/speed', Int16, queue_size=10)
        pub_odometry = rospy.Publisher("/odometry", Float64MultiArray, queue_size=10)
        

        
        
        GUI.run()


        



        






