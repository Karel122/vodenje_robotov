#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController

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

def switch_to_twist_controller():
    print("Zaganjam preklop na twist_controller...")
    rospy.wait_for_service('/controller_manager/switch_controller')

    try:
        switch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        response = switch(
            start_controllers=['twist_controller'],
            stop_controllers=['scaled_pos_joint_traj_controller'],
            strictness=2  # 2 = STRICT
        )
        if response.ok:
            print(" Uspešno preklopljeno na twist_controller.")
        else:
            print("⚠️ Preklop ni uspel.")
    except rospy.ServiceException as e:
        print(f"Napaka pri preklopu kontrolerjev: {e}")



if __name__ == '__main__':
    try:
        rospy.init_node('move_and_switch_controller')

        #  Pozicije sklepov (v radianih)
        joint_positions = [-1.732813835144043, -1.4782617849162598, -0.7301242987262171, -1.574956556359762, 1.538994312286377, -0.7403929869281214]
         
        

        # 1. Premik v točko
        move_to_joint_position(joint_positions)

        # 2. Preklop na twist kontroler
        switch_to_twist_controller()

        # 3. Tu lahko nadaljujete z regulacijo sile...

    except rospy.ROSInterruptException:
        pass