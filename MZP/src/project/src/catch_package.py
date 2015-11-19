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
import exp_quat_func as eqf
import numpy as np

listener = None

def quat_to_rbt(pos, rot):
    omega, theta = eqf.quaternion_to_exp(np.array([rot[0], rot[1], rot[2], rot[3]]))
    rbt = eqf.create_rbt(omega, theta, np.array([pos[0], pos[1], pos[2]]))
    return rbt

def rel_rbt(a_t, a_r, b_t, b_r):
    rbt_a = quat_to_rbt(a_t, a_r)
    rbt_b = quat_to_rbt(b_t, b_r)
    return eqf.compute_gab(rbt_a, rbt_b)

def main():
    rospy.init_node('moveit_node')
    listener = tf.TransformListener()
    #listener.waitForTransform('left_hand_camera', 'base')

    while not rospy.is_shutdown():
        try:
            (trans_marker_cam, rot_marker_cam) = listener.lookupTransform('ar_marker_3','left_hand_camera', rospy.Time())
            (trans_cam_hand, rot_cam_hand) = listener.lookupTransform('left_hand_camera', 'left_hand', rospy.Time())
            (trans_base_hand, rot_base_hand) = listener.lookupTransform('left_hand', 'left_gripper_base', rospy.Time())
            (trans_grip_base, rot_grip_base) = listener.lookupTransform('left_gripper_base', 'left_gripper', rospy.Time())
            (trans_init, rot_init) = listener.lookupTransform('base', 'left_gripper', rospy.Time())

            break
        except:
            continue

    # Find the transform between the AR tag and the left gripper
    print('initial pose')
    print(trans_init)

    trans_marker_hand = rel_rbt(trans_marker_cam,rot_marker_cam,trans_cam_hand,rot_cam_hand)
    trans_grip_hand = rel_rbt(trans_base_hand,rot_base_hand,trans_grip_base,rot_grip_base)
    trans_final = np.dot(trans_marker_hand, trans_grip_hand)
    print('final')
    print(trans_final)


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

    start_point = PoseStamped()
    start_point.header.frame_id = "base"


    #x, y, and z position
    x_pos = "%.2f" % (trans_init[0] - trans_final[1,3])
    y_pos = "%.2f" % (trans_init[1] - trans_final[0,3])
    z_pos = "%.2f" % (trans_init[2] - trans_final[2,3])

    print(trans_init)
    print(trans_final)
    print(x_pos)
    print(y_pos)
    print(z_pos)


    goal_1.pose.position.x = 0.8
    # float(x_pos)
    goal_1.pose.position.y = 0.5
    # float(y_pos)
    goal_1.pose.position.z = 0
    # float(z_pos) + 0.05
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = 0.0
    goal_1.pose.orientation.y = -1.0
    goal_1.pose.orientation.z = 0.0
    goal_1.pose.orientation.w = 0.0


    start_point.pose.position.x = 0.6
    start_point.pose.position.y = 0.684
    start_point.pose.position.z = 0.188
    
    #Orientation as a quaternion
    start_point.pose.orientation.x = 0.0
    start_point.pose.orientation.y = -1.0
    start_point.pose.orientation.z = 0.0
    start_point.pose.orientation.w = 0.0
    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_1)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()
    #Plan a path
    left_plan = left_arm.plan()
 

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
    left_arm.execute(left_plan)

    # #Close the Gripper
    # raw_input('Press <Enter> to close the gripper')

    # left_gripper.close(timeout=60)
    # print('Gripper closed')


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


    # #Release the Gripper
    # raw_input('Press <Enter> to open the gripper')
    # left_gripper.open(block=True)
    # print('Gripper opened')


if __name__ == '__main__':
    #rospy.init_node('catch1')
    main()