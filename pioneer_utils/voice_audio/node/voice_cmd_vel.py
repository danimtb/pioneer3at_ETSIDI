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
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.get_children(recursive=True):
        proc.terminate()
    process.terminate()

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.1
        self.buildmap = False
        self.follower = False
        self.navigation = False
        self.msg = Twist()
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)
        # .txt file with name and x, y coordinates for location
        self.map_locations = rospy.get_param("~map_locations")
        # odometry topic name
        self.odometry_topic = rospy.get_param("~odometry_topic", "odom")
        # cmd_vel topic name
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "cmd_vel")
                       
        self.locations = dict()
        fh=open(self.map_locations)
        for line in fh:
            name = line.rstrip().split(":")
            temp = str(line.rstrip().rsplit(":", 1)[1])
            coordinates = temp.split()
            self.locations[name[0]] = Pose(Point(float(coordinates[0]), float(coordinates[1]), 0.000), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0)))

        # Create the sound client object
        self.soundhandle = SoundClient()
       
        rospy.sleep(1)
        self.soundhandle.stopAll()
        
         # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
       
        # Announce that we are ready for input
        rospy.sleep(1)
        self.soundhandle.say('Hi, my name is Petrois')
        rospy.sleep(2)
        self.soundhandle.say("Say one of the navigation commands")

        # publish to cmd_vel, subscribe to speech output
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=2)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        if msg.data.find("fast") > -1:
            if self.speed != 0.3:
                self.soundhandle.say('Speeding up')
                self.set_speed(0.3)
            else:
                self.soundhandle.say('Already at full speed')

        if msg.data.find("half") > -1:
            if self.speed != 0.2:
                self.soundhandle.say('Going at half speed')
                self.set_speed(0.2)
            else:
                self.soundhandle.say('Already at half speed')

        if msg.data.find("slow") > -1:
            if self.speed != 0.1:
                self.soundhandle.say('Slowing down')
                self.set_speed(0.1)
            else:
                self.soundhandle.say('Already at slow speed')

        if msg.data.find("forward") > -1:
            self.forward()
        elif msg.data.find("right") > -1:
            self.right()
        elif msg.data.find("left") > -1:
            self.left()
        elif msg.data.find("back") > -1:
            self.backward()
        elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:
            self.stop()

################################# follower commands
        
        if msg.data.find("follow me") > -1:
            self.run_follower(True)
        elif msg.data.find("stop follower") > -1:
           self.run_follower(False)


################################ map commands

        if msg.data.find("build map") > -1:
           self.build_map(True)

        elif msg.data.find("save map") > -1:
            self.save_map()
		
        elif msg.data.find("stop map") > -1:
            self.build_map(False)

################################ navigation commands

        if msg.data.find("kk") > -1: #OJOOOOOOOOOOOOOOOOOOO cambiar
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

        elif msg.data.find("kk") > -1: #OJOOOOOOOOOOOOO cambiar
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
                
        elif msg.data.find("navigate to") > -1:
            if len(msg.data.split()) > 2:
                if msg.data.rsplit("navigate to ", 1)[1] in self.locations.keys():
                    self.send_goal(msg.data.rsplit("navigate to ", 1)[1])

        self.pub.publish(self.msg)
        
    def send_goal(self, location_name):
        location = self.locations.get(location_name)
        # Set up goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = location
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo(self.goal)
            
        # Let the user know where the robot is going next
        rospy.loginfo("Going to: " + str(location_name))
        self.soundhandle.say("Going to " + str(location_name))
            
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)

    def run_follower(self, on):
        if on and self.follower == False:
            self.msg = Twist()
            subprocess.Popen(['roslaunch', 'pioneer_utils', 'simple-follower.launch'])
            self.soundhandle.say('Okay. Show me the way')
            self.follower = True
        elif on and self.follower:
            self.soundhandle.say('Already in follower mode')
        elif on == False and self.follower:
            self.msg = Twist()
            subprocess.Popen(['rosnode', 'kill', 'turtlebot_follower'])
            self.follower = False
            self.soundhandle.say('Follower mode disabled')

    def forward(self):
        self.soundhandle.play(1)    
        self.msg.linear.x = self.speed
        self.msg.angular.z = 0

    def backward(self):
        self.soundhandle.play(1)
        self.msg.linear.x = -self.speed
        self.msg.angular.z = 0
   
    def left(self):
        self.soundhandle.play(1)
        self.msg.linear.x = 0
        self.msg.angular.z = self.speed*2
        
    def right(self):
        self.soundhandle.play(1)    
        self.msg.linear.x = 0
        self.msg.angular.z = -self.speed*2

    def stop(self):
        self.move_base.cancel_goal()
        self.run_follower(False)
        self.msg = Twist()
        self.soundhandle.play(1)

    def set_speed(self, vel):
        if self.msg.linear.x > 0:
            self.msg.linear.x = vel
        elif self.msg.linear.x < 0:
            self.msg.linear.x = -vel
        if self.msg.angular.z >0:
            self.msg.angular.z = vel
        elif self.msg.angular.z < 0:
            self.msg.angular.z = -vel
            self.speed = vel

    def build_map(self, on):
        if on and self.buildmap == False:
            self.stop()
            self.soundhandle.say('Building map with slam gmapping')
            subprocess.Popen(['rosnode', 'kill', 'amcl'])
            subprocess.Popen(['rosnode', 'kill', 'map_server'])
            subprocess.Popen(['roslaunch', 'p2os_launch', 'gmapping.launch'])
            self.buildmap = True
        elif on and self.buildmap:
            self.soundhandle.say('Already building a map')
        elif on == False and self.buildmap:
            self.stop()
            subprocess.Popen(['rosnode', 'kill', 'slam_gmapping'])
            self.buildmap = False
            subprocess.Popen(['roslaunch', 'pioneer_utils', 'floor_zero-map.launch'])
            subprocess.Popen(['roslaunch', 'p2os_launch', 'amcl.launch'])
            self.soundhandle.say('Building map stopped')
        else:
            self.soundhandle.say('I am not building any map')

    def save_map(self):
        if self.buildmap == True:
            self.msg = Twist()
            subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', 'new_map'])
            rospy.sleep(4)
            print 'map saved at ~/.ros directory as new_map.pgm new_map.yaml'
            self.soundhandle.say('Map saved successfully')
        else:
            self.soundhandle.say('I am not building any map so there is no map to save')

    def cleanup(self):
        # stop the robot!
        self.stop()
        self.pub.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

