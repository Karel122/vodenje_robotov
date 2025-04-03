#!/usr/bin/env python3

# example class to control UR5e robot
#
# Sebastjan Slajpah @ Robolab, 2022
#
import actionlib
import rospy
from geometry_msgs.msg import Twist, Vector3, WrenchStamped
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController 



konec = [-2.115241050720215, -1.4767769140056153, -0.6145289579974573, -1.139075593357422, 1.5942487716674805, -0.6085966269122522]

JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]





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



        
    def switch(self, start, stop):
        print("Switch")
        rospy.wait_for_service('/controller_manager/switch_controller')     
        switch_client = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        print("Prižigam konroler")
        switch_client(
            start_controllers=[start],
            stop_controllers=[stop],
            strictness=2  # 2 = STRICT
            )
        print("konec")

    def switchStop(self, stop):
        print("Čakam service")
        rospy.wait_for_service('/controller_manager/switch_controller')     
        switch_client = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        print("Prižigam konreoler")
        switch_client(
            start_controllers=[],
            stop_controllers=[stop],
            strictness=2  # 2 = STRICT
            )
        print("konroler zamenjan")

    def switchStart(self, start):
        print("Čakam service")
        rospy.wait_for_service('/controller_manager/switch_controller')     
        switch_client = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        print("Prižigam konreoler")
        switch_client(
            start_controllers=[start],
            stop_controllers=[],
            strictness=2  # 2 = STRICT
            )
        print("konec")

    def move_to_joint_position(self, joint_goal):
        # Action client za pozicijski kontroler
        print("Move to joint position")
        client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        print("Čakam na /scaled_pos_joint_traj_controller...")
        client.wait_for_server()
        print("Povezan z action serverjem.")

        # Pripravi action goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'elbow_joint', 'shoulder_lift_joint' , 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint'
    
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_goal
        point.time_from_start = rospy.Duration(6.0)  # trajanje premika

        goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now()

        print("Pošiljam ciljno točko sklepov...")
        client.send_goal(goal)
        client.wait_for_result()
        print(" Robot dosegel ciljno pozicijo.")


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
            self.pub.publish(silaAVRGx)
            self.pub = rospy.Publisher('/PovprecenY', Float32, queue_size=10)
            self.pub.publish(silaAVRGy)
            self.pub = rospy.Publisher('/PovprecenZ', Float32, queue_size=10)
            self.pub.publish(silaAVRGz)
            

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
            
                self.cmd.linear.x = -0.005*silaAVRGy
                self.cmd.linear.y = -0.010*silaAVRGx
                self.cmd.linear.z = -0.005*silaAVRGz


            else:
                self.cmd.linear.x = 0
                self.cmd.linear.y = 0
                self.cmd.linear.z = 0
                #self.cmd.linearrospy.wait_for_service('/ur_hardware_interface/zero_ftsensor')

        

            self.vel_publisher.publish(self.cmd)
            #print(self.cmd)

    def pomik_dol(self):
        print("premik dol")
        self.sile = self.wrench_msg.wrench.force


    



        while not self.ctrl_c and abs(self.sile.z) < 5:
            self.sile = self.wrench_msg.wrench.force
            self.cmd.linear.x = 0
            self.cmd.linear.z = -0.05
            self.cmd.linear.y = 0
            self.vel_publisher.publish(self.cmd)
            # print(self.cmd)


        self.cmd.linear.x = 0
        self.cmd.linear.z = 0
        self.cmd.linear.y = 0
        self.vel_publisher.publish(self.cmd)
        #print(self.cmd)

    def z_Kontrola(self):

        print("z kontrola")
        self.size = 300
        self.bufferz = []
        self.running_sumz = 0

        while not self.ctrl_c:
            self.sile = self.wrench_msg.wrench.force

            if len(self.bufferz) == self.size:
                self.running_sumz -= self.bufferz.pop(0)  # Remove oldest value from sum
            self.bufferz.append(self.sile.z)
            self.running_sumz += self.sile.z
            silaAVRGz = (self.running_sumz / len(self.bufferz) if self.bufferz else 0)


            self.cmd.linear.x = 0.02
            self.cmd.linear.z = -0.005*(silaAVRGz+3)
            self.vel_publisher.publish(self.cmd)





if __name__ == '__main__':

    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_to_joint_position(konec)
        robotcontrol_object.switch("twist_controller", "scaled_pos_joint_traj_controller")
        robotcontrol_object.pomik_dol()

        robotcontrol_object.z_Kontrola()
        robotcontrol_object.switch("scaled_pos_joint_traj_controller", "twist_controller")


        #switch(start,stop)

        #robotcontrol_object.move_to_joint_position(konec)
        #robotcontrol_object.switch("twist_controller", "scaled_pos_joint_traj_controller")
        #robotcontrol_object.switch("scaled_pos_joint_traj_controller", "twist_controller")
        #robotcontrol_object.switchStart("twist_controller")
        #robotcontrol_object.z_Kontrola()
        #robotcontrol_object.pomik_dol()
        #robotcontrol_object.vodenje_robota()
    except rospy.ROSInterruptException:
        pass





        