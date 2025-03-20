#!/usr/bin/env python3

# example class to control UR5e robot
#
# Sebastjan Slajpah @ Robolab, 2022
#

import rospy
from geometry_msgs.msg import Twist, Vector3, WrenchStamped
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse



class RobotControl():
    

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)

        cmd_vel_topic='/twist_controller/command'
        self._check_wrench_ready()

        # start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()   

        
            
        self.wrench_subscriber = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)


        rospy.wait_for_service('/ur_hardware_interface/zero_ftsensor')
        rospy.loginfo("Zero FT sensor ready.")
        self.kalibracija = rospy.ServiceProxy('/ur_hardware_interface/zero_ftsensor',Trigger)
        zagon = TriggerRequest()
        # resp = self.kalibracija(zagon)
        # print(resp)

        self.ctrl_c = False
        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.shutdownhook)
        



    def _check_wrench_ready(self):
        self.wrench_msg = None
        rospy.loginfo("Checking wrench ...")
        while self.wrench_msg is None and not rospy.is_shutdown():
            try:
                self.wrench_msg = rospy.wait_for_message("/wrench", WrenchStamped, timeout=1.0)
                rospy.logdebug("Current /wrench READY=>" + str(self.wrench_msg))

            except:
                rospy.logerr("Current /wrench not ready yet, retrying for getting scan")
        rospy.loginfo("Checking /wrench...DONE")
        return self.wrench_msg

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
 
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_robot()#robotcontrol_object.move_straight_time('up',0.0, 3) # objekt robotcontrol_object.move_straight_time('up',0.0, 3) direction' up' speed 0.o  3=time (s)
        self.ctrl_c = True

    def wrench_callback(self, msg):
        self.wrench_msg = msg

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear = Vector3(0.0, 0.0, 0.0)
        self.cmd.angular = Vector3(0.0, 0.0, 0.0)
        self.publish_once_in_cmd_vel()

    def vodenje_robota(self):


        self.size = 300
        self.bufferx = []
        self.buffery = []
        self.bufferz = []

        self.running_sumx = 0
        self.running_sumy = 0
        self.running_sumz = 0

        while not self.ctrl_c:
            self.sile = self.wrench_msg.wrench.force

            if len(self.bufferx) == self.size:
                self.running_sumx-= self.bufferx.pop(0)  # Remove oldest value from sum
            self.bufferx.append(self.sile.x)
            self.running_sumx+= self.sile.x
            silaAVRGx = (self.running_sumx / len(self.bufferx) if self.bufferx else 0)


            if len(self.buffery) == self.size:
                self.running_sumy -= self.buffery.pop(0)  # Remove oldest value from sum
            self.buffery.append(self.sile.y)
            self.running_sumy += self.sile.y
            silaAVRGy= (self.running_sumy / len(self.buffery) if self.buffery else 0)


            if len(self.bufferz) == self.size:
                self.running_sumz -= self.bufferz.pop(0)  # Remove oldest value from sum
            self.bufferz.append(self.sile.z)
            self.running_sumz += self.sile.z
            silaAVRGz = (self.running_sumz / len(self.bufferz) if self.bufferz else 0)


            self.pub = rospy.Publisher('/PovprecenX', Float32, queue_size=10)
            self.pub.publish(silaAVRGx+2)
            self.pub = rospy.Publisher('/PovprecenY', Float32, queue_size=10)
            self.pub.publish(silaAVRGy+2)
            self.pub = rospy.Publisher('/PovprecenZ', Float32, queue_size=10)
            self.pub.publish(silaAVRGz+2)
            

            hitrost_x = -0.01*self.sile.x
            hitrost_y = -0.01*self.sile.y
            hitrost_z = -0.01*self.sile.z
            
            if (abs(self.sile.x)>2 or abs(self.sile.y)>2 or abs(self.sile.z)>2):
                #self.cmd.linear.x = 0
                #self.cmd.linear.z = 0
                #self.cmd.linear.y = 0

                """ self.cmd.linear.x = -0.025*silaAVRGy
                self.cmd.linear.y = -0.035*silaAVRGx
                self.cmd.linear.z = -0.025*silaAVRGz 
                """
            
                self.cmd.linear.x = -0.010*silaAVRGy
                self.cmd.linear.y = -0.015*silaAVRGx
                self.cmd.linear.z = -0.010*silaAVRGz


            else:
                self.cmd.linear.x = 0
                self.cmd.linear.y = 0
                self.cmd.linear.z = 0
                #self.cmd.linearrospy.wait_for_service('/ur_hardware_interface/zero_ftsensor')

        

            #self.vel_publisher.publish(self.cmd)
            #print(self.cmd)


        



if __name__ == '__main__':

    robotcontrol_object = RobotControl()
    try:
        
        
        robotcontrol_object.vodenje_robota()
    except rospy.ROSInterruptException:
        pass





        