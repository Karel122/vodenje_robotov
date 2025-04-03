from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import Twist, Vector3, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController 


konec = [-1.732813835144043, -1.4782617849162598, -0.7301242987262171, -1.574956556359762, 1.538994312286377, -0.7403929869281214]

JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]



def switch(start, stop):
    print("Čakam service")
    rospy.wait_for_service('/controller_manager/switch_controller')     
    switch_client = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    print("posodabljam stanje")
    switch_client(
        start_controllers=[start],
        stop_controllers=[stop],
        strictness=2  # 2 = STRICT
        )
    print("konec")


def move_to_joint_position(joint_goal):
        # Action client za pozicijski kontroler
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



if __name__ == '__main__':

    try:
        rospy.init_node('MickinPremik')
          

        # switch('twist_controller', 'joint_state_controller')
        switch( 'scaled_pos_joint_traj_controller','twist_controller',)
    except rospy.ROSInterruptException:
        pass



	
