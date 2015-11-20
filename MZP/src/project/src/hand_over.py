#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
from sensor_msgs.msg import Range
import exp_quat_func as eqf
import numpy as np
from scipy import linalg
# import IR file

listener = None

def quat_to_rbt(pos, rot):
    omega, theta = eqf.quaternion_to_exp(np.array([rot[0], rot[1], rot[2], rot[3]]))
    rbt = eqf.create_rbt(omega, theta, np.array([pos[0], pos[1], pos[2]]))
    return rbt

def callback(message):
    global mail_range
    mail_range = message.range


def IR_range():
    range_msg  = rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
  





def main():
    rospy.init_node('moveit_node')
    listener = tf.TransformListener()
    #listener.waitForTransform('left_hand_camera', 'base')

    while not rospy.is_shutdown():
        try:
            (trans_marker_cam, rot_marker_cam) = listener.lookupTransform('ar_marker_3','left_hand_camera', rospy.Time())
            (trans_cam_hand, rot_cam_hand) = listener.lookupTransform('left_hand_camera', 'left_hand', rospy.Time())
            (trans_hand_base, rot_hand_base) = listener.lookupTransform('left_hand', 'base', rospy.Time())
            break
        except:
            continue

    # Find the transform between the AR tag and the base
    g_marker_cam = quat_to_rbt(trans_marker_cam, rot_marker_cam)
    g_cam_hand = quat_to_rbt(trans_cam_hand, rot_cam_hand)
    g_hand_base = quat_to_rbt(trans_hand_base, rot_hand_base)

    g_final = linalg.inv(np.dot(np.dot(g_marker_cam, g_cam_hand), g_hand_base))
    print('final')
    print(g_final)


    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
   # rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    #Set up the left gripper
    left_gripper = baxter_gripper.Gripper('left')


    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"


    #x, y, and z position
    x_pos = "%.3f" % (g_final[0,3])
    y_pos = "%.3f" % (g_final[1,3])
    z_pos = "%.3f" % (g_final[2,3])

    print(x_pos)
    print(y_pos)
    print(float(z_pos)+0.05)


    goal_1.pose.position.x = float(x_pos)
    goal_1.pose.position.y = float(y_pos)
    goal_1.pose.position.z = float(z_pos) + 0.07
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.0
    goal_1.pose.orientation.y = -1.0
    goal_1.pose.orientation.z = 0.0
    goal_1.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_1)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()
    #Plan a path
    left_plan = left_arm.plan()
 

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
    left_arm.execute(left_plan)

    
    rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
    rospy.sleep(1)
    print(mail_range)

    #Check using IR to see of the mail is close enough
    incre = 0.01
    while mail_range > 0.049 and incre < 0.05:
        print('Too far from the mail. Trying again...')
        rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
        rospy.sleep(1)
        print(mail_range)
        goal_1.pose.position.z = float(z_pos) + 0.07 - incre
        incre += 0.01
            #Set the goal state to the pose you just defined
        left_arm.set_pose_target(goal_1)

        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()
        #Plan a path
        left_plan = left_arm.plan()
 
        #Execute the plan
        raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
        left_arm.execute(left_plan)
    print(mail_range)
    #Close the Gripper
    raw_input('Press <Enter> to close the gripper')

    left_gripper.close(timeout=60)
    print('Gripper closed')
        
        

    #Second goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_2 = PoseStamped()
    goal_2.header.frame_id = "base"

    #x, y, and z position
    goal_2.pose.position.x = 0.8
    goal_2.pose.position.y = 0.5
    goal_2.pose.position.z = 0.0
    
    #Orientation as a quaternion
    goal_2.pose.orientation.x = 0.0
    goal_2.pose.orientation.y = -1.0
    goal_2.pose.orientation.z = 0.0
    goal_2.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_2)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 2: ')
    left_arm.execute(left_plan)


    #Release the Gripper
    raw_input('Press <Enter> to open the gripper')
    left_gripper.open(block=True)
    print('Gripper opened')


if __name__ == '__main__':
    #rospy.init_node('catch1')
    main()
