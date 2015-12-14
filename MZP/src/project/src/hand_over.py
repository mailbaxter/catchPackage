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
from ar_track_alvar.msg import AlvarMarker, AlvarMarkers
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

listener = None

# Instantiate CvBridge
bridge = CvBridge()

def quat_to_rbt(pos, rot):
    omega, theta = eqf.quaternion_to_exp(np.array([rot[0], rot[1], rot[2], rot[3]]))
    rbt = eqf.create_rbt(omega, theta, np.array([pos[0], pos[1], pos[2]]))
    return rbt

def callback(message):
    global mail_range
    mail_range = message.range

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.png', cv2_img)


def callbackAR(message):
    global ar_name
    try:
        ar_name = message.markers[0].id
    except:
        a=1

def main():
    rospy.init_node('moveit_node')
    listener = tf.TransformListener()
    #listener.waitForTransform('left_hand_camera', 'base')

    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    #rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    #right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    #right_arm.set_planner_id('RRTConnectkConfigDefault')
    #right_arm.set_planning_time(10)

    #Set up the left gripper
    left_gripper = baxter_gripper.Gripper('left')


    #Original pose ------------------------------------------------------

    original = PoseStamped()
    original.header.frame_id = "base"

    #the pose the baxter will first turn to
    original.pose.position.x = 0.210
    original.pose.position.y = 0.907
    original.pose.position.z = 0.205
    
    original.pose.orientation.x = -0.689
    original.pose.orientation.y = -0.013
    original.pose.orientation.z = -0.035
    original.pose.orientation.w = 0.723

    left_arm.set_pose_target(original)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    left_plan = left_arm.plan()

    raw_input('Press <Enter> to move the left arm to the original pose: ')
    left_arm.execute(left_plan)

    # First goal pose to pick up ----------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('ar_pose_marker',AlvarMarkers, callbackAR)
            rospy.sleep(1)
            arTagName = 'ar_marker_' + str(ar_name)
            print(arTagName)
            # Set up your subscriber and define its callback
            image_topic = "/cameras/left_hand_camera/image"
            my_image_msg = rospy.wait_for_message(image_topic, Image)
            image_callback(my_image_msg)
            break
        except:
            continue
    arTagName = 'ar_marker_' + str(ar_name)
    # Display the image on Baxter's screen
    display_pub = rospy.Publisher('/robot/xdisplay',Image, latch=True)
    display_pub.publish(my_image_msg)
    rospy.sleep(1)
    #TODO- generate the recognized image and publish it
    # Check http://sdk.rethinkrobotics.com/wiki/Display_Image_-_Code_Walkthrough

    while not rospy.is_shutdown():
        try:
            (trans_marker_cam, rot_marker_cam) = listener.lookupTransform(arTagName,'left_hand_camera', rospy.Time())
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




    #x, y, and z position
    x_pos = "%.3f" % (g_final[0,3])
    y_pos = "%.3f" % (g_final[1,3])
    z_pos = "%.3f" % (g_final[2,3])

    print(x_pos)
    print(y_pos)
    print(z_pos)


    goal_1.pose.position.x = float(x_pos)
    goal_1.pose.position.y = float(y_pos) - 0.14
    goal_1.pose.position.z = float(z_pos)
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = -0.689
    goal_1.pose.orientation.y = -0.013
    goal_1.pose.orientation.z = -0.035
    goal_1.pose.orientation.w = 0.723

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
    while mail_range > 0.06 and incre < 0.05:
        print('Too far from the mail. Trying again...')
        rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
        rospy.sleep(1)
        print(mail_range)
        goal_1.pose.position.y = float(y_pos) - 0.14 + incre
        incre += 0.01
            #Set the goal state to the pose you just defined
        left_arm.set_pose_target(goal_1)

        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()
        #Plan a path
        left_plan = left_arm.plan()
 
        #Execute the plan
        raw_input('Press <Enter> to move the left arm to get the mail: ')
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

    if ar_name in (1, 3, 12):

        #x, y, and z position
        goal_2.pose.position.x = 0.7
        goal_2.pose.position.y = 0.6
        goal_2.pose.position.z = -0.15
    elif ar_name in (0,9,15,16):
        #x, y, and z position
        goal_2.pose.position.x = 0.7
        goal_2.pose.position.y = 0.3
        goal_2.pose.position.z = -0.15
    else:
        #x, y, and z position
        goal_2.pose.position.x = 0.7
        goal_2.pose.position.y = 0
        goal_2.pose.position.z = -0.15
    
    #Orientation as a quaternion
    goal_2.pose.orientation.x = 0.0
    goal_2.pose.orientation.y = -1.0
    goal_2.pose.orientation.z = 0.0
    goal_2.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal_2)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to the zipcode region: ')
    left_arm.execute(left_plan)


    #Release the Gripper
    raw_input('Press <Enter> to open the gripper')
    left_gripper.open(block=True)
    print('Gripper opened')


if __name__ == '__main__':
    #rospy.init_node('catch1')
    main()
