#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('pocketsphinx')
import roslib
import rospy
import math
import subprocess
import os, signal
import time
import psutil

from std_msgs.msg import String

from sound_play.libsoundplay import SoundClient

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.get_children(recursive=True):
        proc.terminate()
    process.terminate()

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.1
        self.buildmap = False
        self.follower = False
        self.navigation = False
        self.msg = Twist()

        # Create the sound client object
        self.soundhandle = SoundClient()
       
        rospy.sleep(1)
        self.soundhandle.stopAll()
       
        # Announce that we are ready for input
        rospy.sleep(1)
        self.soundhandle.say('Hi, my name is Petrois')
        rospy.sleep(3)
        self.soundhandle.say("Say one of the navigation commands")

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        if msg.data.find("fast") > -1:
            if self.speed != 0.3:
                self.soundhandle.say('Speeding up')
                if self.msg.linear.x > 0:
                    self.msg.linear.x = 0.3
                elif self.msg.linear.x < 0:
                    self.msg.linear.x = -0.3
                if self.msg.angular.z >0:
                    self.msg.angular.z = 0.3
                elif self.msg.angular.z < 0:
                    self.msg.angular.z = -0.3
                self.speed = 0.3
            else:
                self.soundhandle.say('Already at full speed')

        if msg.data.find("half") > -1:
            if self.speed != 0.2:
                self.soundhandle.say('Going at half speed')
                if self.msg.linear.x > 0:
                    self.msg.linear.x = 0.2
                elif self.msg.linear.x < 0:
                    self.msg.linear.x = -0.2
                if self.msg.angular.z >0:
                    self.msg.angular.z = 0.2
                elif self.msg.angular.z < 0:
                    self.msg.angular.z = -0.2
                self.speed = 0.2
            else:
                self.soundhandle.say('Already at half speed')

        if msg.data.find("slow") > -1:
            if self.speed != 0.1:
                self.soundhandle.say('Slowing down')
                if self.msg.linear.x > 0:
                    self.msg.linear.x = 0.1
                elif self.msg.linear.x < 0:
                    self.msg.linear.x = -0.1
                if self.msg.angular.z >0:
                    self.msg.angular.z = 0.1
                elif self.msg.angular.z < 0:
                    self.msg.angular.z = -0.1
                self.speed = 0.1
            else:
                self.soundhandle.say('Already at slow speed')

        if msg.data.find("forward") > -1:
            self.soundhandle.play(1)    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif msg.data.find("left") > -1:
            self.soundhandle.play(1)
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:        
                self.msg.angular.z = self.speed*2
        elif msg.data.find("right") > -1:
            self.soundhandle.play(1)    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:        
                self.msg.angular.z = -self.speed*2
        elif msg.data.find("back") > -1:
            self.soundhandle.play(1)
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:
            self.soundhandle.play(1)
            self.msg = Twist()

################################# follower commands
        
        if msg.data.find("follow me") > -1:
            if self.follower == False:
                self.msg = Twist()
                self.proc1 = subprocess.Popen(['roslaunch', 'pioneer_utils', 'simple-follower.launch'])
                self.soundhandle.say('Okay. Show me the way')
                self.follower = True
            else:
                self.soundhandle.say('Already in follower mode')
		
        elif msg.data.find("stop follower") > -1:
            if self.follower == True:
                self.msg = Twist()
                print 'proc1 = ', self.proc1.pid
                self.proc1.terminate()
                kill(self.proc1.pid)
                self.proc1.kill()
                self.follower = False
                self.soundhandle.say('Follower mode disabled')
            else:
                self.soundhandle.say('Hey, I wasnt following you')

################################ map commands

        if msg.data.find("build map") > -1:
            if self.buildmap == False:
                self.soundhandle.say('Building map with slam gmapping')
                rospy.sleep(2)
                self.soundhandle.say('Visualizing map')
                self.msg = Twist()
                self.proc2 = subprocess.Popen(['roslaunch', 'p2os_launch', 'gmapping.launch'])
                self.proc3 = subprocess.Popen(['roslaunch', 'pioneer_utils', 'rviz-gmapping.launch'])
                self.buildmap = True
            else:
                self.soundhandle.say('Already building a map')


        elif msg.data.find("save map") > -1:
            if self.buildmap == True:
                self.msg = Twist()
                self.proc4 = subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', 'new_map'])
                rospy.sleep(6)
                print 'map saved at ~/.ros directory as new_map.pgm new_map.yaml'
                self.soundhandle.say('Map saved successfully')
            else:
                self.soundhandle.say('I am not building any map so there is no map to save')
		
        elif msg.data.find("stop map") > -1:
            if self.buildmap == True:
               self.msg = Twist() 
               print 'proc2 = ', self.proc2.pid
               self.proc2.terminate()
               kill(self.proc2.pid)
               self.proc2.kill()
               print 'proc3 = ', self.proc3.pid
               self.proc3.terminate()
               kill(self.proc3.pid)
               self.proc3.kill()
               print 'proc4 = ', self.proc4.pid
               self.proc4.terminate()
               kill(self.proc4.pid)
               self.proc4.kill()
               self.buildmap = False
               self.soundhandle.say('Building map stopped')
            else:
               self.soundhandle.say('I am not building any map')

################################ navigation commands

        if msg.data.find("navigate") > -1:
            if self.navigation == False:
                self.soundhandle.say('Starting navigation stack')
                rospy.sleep(2)
                self.soundhandle.say('Visualizing costmaps')
                self.msg = Twist()
                self.proc5 = subprocess.Popen(['roslaunch', 'pioneer_utils', 'navigation_p3at.launch'])
                self.proc6 = subprocess.Popen(['roslaunch', 'pioneer_utils', 'rviz-navigation.launch'])
                self.navigation = True
            else:
                self.soundhandle.say('Already in navigation mode')


        elif msg.data.find("stop navigation") > -1:
            if self.navigation == True:
                self.msg = Twist()
                print 'proc5 = ', self.proc5.pid
                self.proc5.terminate()
                kill(self.proc5.pid)
                self.proc5.kill()
                print 'proc6 = ', self.proc6.pid
                self.proc6.terminate()
                kill(self.proc6.pid)
                self.proc6.kill()
                self.navigation = False
                self.soundhandle.say('Navigation stopped')
            else:
                self.soundhandle.say('I am not in navigation mode')
        
        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

