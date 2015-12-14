#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
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


# def callbackAR(message):
#     global ar_name
#     try:
#         print('callback AR')
#         ar_name = message.markers[0].id
#         print(ar_name)
#     except:
#         a=1

def move_left_to(goal):
    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal)
    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()
    #Plan a path
    left_plan = left_arm.plan()
    raw_input('Press <Enter> to move the left arm to the drop-off pile: ')
    #Execute the plan
    left_arm.execute(left_plan)

def catch_from_hand(goal):
    # print(goal.pose.position.x)
    # print(goal.pose.position.y)
    # print(goal.pose.position.z)

    goal_adj = PoseStamped()
    goal_adj.header.frame_id = "base"

    #the pose the baxter will first turn to
    goal_adj.pose.position.x = goal.pose.position.x
    goal_adj.pose.position.y = goal.pose.position.y
    goal_adj.pose.position.z = goal.pose.position.z
    
    goal_adj.pose.orientation.x = goal.pose.orientation.x
    goal_adj.pose.orientation.y = goal.pose.orientation.y
    goal_adj.pose.orientation.z = goal.pose.orientation.z
    goal_adj.pose.orientation.w = goal.pose.orientation.w

    left_arm.set_pose_target(goal_adj)
    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()
    #Plan a path
    pick_mail_plan = left_arm.plan()
    raw_input('Press <Enter> to move the left arm to catch from hand: ')
    left_arm.execute(pick_mail_plan)

    rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
    rospy.sleep(1)
    # print(mail_range)
    #Move to starting pile  -----------------------------------------------------
    #Check using IR to see of the mail is close enough
    incre = 0.01
    while mail_range > 0.06 and incre < 0.05:
        print('Too far from the mail. Trying again...')
        rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
        rospy.sleep(1)
        print(mail_range)
        goal_adj.pose.position.z = goal.pose.position.y - 0.14 + incre
        incre += 0.01
        #Set the goal state to the pose you just defined
        left_arm.set_pose_target(goal_adj)
        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()
        #Plan a path
        pick_mail_plan = left_arm.plan()
         
        #Execute the plan
        raw_input('Press <Enter> to move the left arm to the adjusted pick-up pile: ')
        left_arm.execute(pick_mail_plan)
    # print(mail_range)

def pick_from_pile(goal):

    # print(goal.pose.position.x)
    # print(goal.pose.position.y)
    # print(goal.pose.position.z)

    goal_adj = PoseStamped()
    goal_adj.header.frame_id = "base"

    #the pose the baxter will first turn to
    goal_adj.pose.position.x = goal.pose.position.x
    goal_adj.pose.position.y = goal.pose.position.y
    goal_adj.pose.position.z = goal.pose.position.z
    
    goal_adj.pose.orientation.x = goal.pose.orientation.x
    goal_adj.pose.orientation.y = goal.pose.orientation.y
    goal_adj.pose.orientation.z = goal.pose.orientation.z
    goal_adj.pose.orientation.w = goal.pose.orientation.w

    left_arm.set_pose_target(goal_adj)
    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()
    #Plan a path
    pick_mail_plan = left_arm.plan()
    raw_input('Press <Enter> to move the left arm to the pick-up pile: ')
    left_arm.execute(pick_mail_plan)

    rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
    rospy.sleep(1)
    # print(mail_range)
    #Move to starting pile  -----------------------------------------------------
    #Check using IR to see of the mail is close enough
    incre = 0.05
    while mail_range > 0.049 and incre < 0.1:
        print('Too far from the mail. Trying again...')
        rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
        rospy.sleep(1)
        print(mail_range)
        goal_adj.pose.position.z = goal.pose.position.z - incre
        incre += 0.01
        #Set the goal state to the pose you just defined
        left_arm.set_pose_target(goal_adj)
        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()
        #Plan a path
        pick_mail_plan = left_arm.plan()
         
        #Execute the plan
        raw_input('Press <Enter> to move the left arm to the adjusted pick-up pile: ')
        left_arm.execute(pick_mail_plan)
    # print(mail_range)


def hand_over():
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

    # Add a collison object
    # Build the attached object
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    #shape.dimensions.resize(3)
    # print(shape.dimensions)
    shape.dimensions.append(0.6)
    shape.dimensions.append(1.5)
    shape.dimensions.append(0.6)
    # print(shape)

    # Create pose
    obj_pose = Pose()
    obj_pose.position.x = 0.665
    obj_pose.position.y = 0.062
    obj_pose.position.z = -0.7
    obj_pose.orientation.x = 0
    obj_pose.orientation.y = 0
    obj_pose.orientation.z = 0
    obj_pose.orientation.w = 1

    # create collision object
    cobj = CollisionObject()
    cobj.primitives = shape
    cobj.primitive_poses=obj_pose
    cobj.header.frame_id='base'
    cobj.id = 'Table'

    # create collision object message
    # ATTACHED_COLLISION_OBJECT.link_name = TCP_LINK_NAME
    # ATTACHED_COLLISION_OBJECT.object = cobj
    # ATTACHED_COLLISION_OBJECT.object.header.frame_id = TCP_LINK_NAME
    # ATTACHED_COLLISION_OBJECT.touch_links.push_back("gripper_body")

    # Attache it to the scene
    cobj.operation = CollisionObject.ADD
    attach_object_publisher = rospy.Publisher('collision_object',CollisionObject,queue_size = 10)
    attach_object_publisher.publish(cobj)
    rospy.sleep(1)

    #Set up the left gripper
    left_gripper = baxter_gripper.Gripper('left')
    input=1
    pile_1=[]
    pile_2=[]
    pile_3=[]

    while input:

        #Original pose ------------------------------------------------------
        original = PoseStamped()
        original.header.frame_id = "base"

        #the pose the baxter will first turn to
        original.pose.position.x = 0.505
        original.pose.position.y = 0.487
        original.pose.position.z = 0.515
        
        original.pose.orientation.x = 0.505
        original.pose.orientation.y = 0.487
        original.pose.orientation.z = 0.515
        original.pose.orientation.w = -0.493

        left_arm.set_pose_target(original)

        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()

        left_plan = left_arm.plan()

        raw_input('Press <Enter> to move the left arm to the original pose: ')
        left_arm.execute(left_plan)

        rospy.sleep(2)
        # First goal pose to pick up ----------------------------------------------
        goal_1 = PoseStamped()
        goal_1.header.frame_id = "base"

        while not rospy.is_shutdown():
            try:
                marker = rospy.wait_for_message('ar_pose_marker',AlvarMarkers)
                #rospy.sleep(1)
                #print(marker)
                ar_name = marker.markers[0].id
                arTagName = 'ar_marker_' + str(ar_name)
                print(arTagName)
                break
            except:
                continue
        #arTagName = 'ar_marker_' + str(ar_name)
        # Set up your subscriber and define its callback
        image_topic = "/cameras/left_hand_camera/image"
        my_image_msg = rospy.wait_for_message(image_topic, Image)

        print("Received an image!")
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(my_image_msg, "bgr8")
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image'+str(ar_name)+'.png', cv2_img)

        # Carry out the digit recognition alg
        # zipcode = digitRecog('camera_image'+str(ar_name)+'.png')

        # Display the original image on Baxter's screen
        display_pub = rospy.Publisher('/robot/xdisplay',Image, latch=True)
        display_pub.publish(my_image_msg)
        rospy.sleep(1)


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
        # print('final')
        # print(g_final)

        #x, y, and z position
        x_pos = "%.3f" % (g_final[0,3])
        y_pos = "%.3f" % (g_final[1,3])
        z_pos = "%.3f" % (g_final[2,3])

        # print(x_pos)
        # print(y_pos)
        # print(z_pos)


        goal_1.pose.position.x = float(x_pos)
        goal_1.pose.position.y = float(y_pos) - 0.2
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
        # print(mail_range)

        #Check using IR to see of the mail is close enough
        incre = 0.01
        while mail_range > 0.06 and incre < 0.05:
            print('Too far from the mail. Trying again...')
            rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
            rospy.sleep(1)
            print(mail_range)
            goal_1.pose.position.y = float(y_pos) - 0.1 + incre
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
        # print(mail_range)
        #Close the Gripper
        raw_input('Press <Enter> to close the gripper')

        left_gripper.close(timeout=60)
        print('Gripper closed')        
       
        #TODO- generate the recognized image and publish it
        # Check http://sdk.rethinkrobotics.com/wiki/Display_Image_-_Code_Walkthrough
        # display_pub.sleep()
        img = cv2.imread('withRecog'+str(ar_name)+'.png')
        msg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        rec_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        rec_pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(1)

        #Second goal pose -----------------------------------------------------
        rospy.sleep(2.0)
        goal_2 = PoseStamped()
        goal_2.header.frame_id = "base"


        if ar_name in (1, 3, 12):
            #x, y, and z position
            goal_2.pose.position.x = 0.7
            goal_2.pose.position.y = 0.6
            goal_2.pose.position.z = -0.18
            pile_1.insert(0,ar_name)
        elif ar_name in (0,9,15,16):
            #x, y, and z position
            goal_2.pose.position.x = 0.7
            goal_2.pose.position.y = 0.3
            goal_2.pose.position.z = -0.18
            pile_2.insert(0,ar_name)
        else:
            #x, y, and z position
            goal_2.pose.position.x = 0.7
            goal_2.pose.position.y = 0
            goal_2.pose.position.z = -0.18
            pile_3.insert(0,ar_name)
        
        print('pile_1')
        print(pile_1)

        print('pile_2')
        print(pile_2)

        print('pile_3')
        print(pile_3)

        #Orientation as a quaternion
        goal_2.pose.orientation.x = 0.0
        goal_2.pose.orientation.y = -1.0
        goal_2.pose.orientation.z = 0.0
        goal_2.pose.orientation.w = 0.0

        
        #Execute the plan
        #raw_input('Press <Enter> to move the left arm to the zipcode region: ')
        
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
        #reg_pub.sleep()

        #Continue or stop
        input=int(raw_input('Press 1 to start/continue moving, or 0 to stop moving  '))
    return (pile_1,pile_2,pile_3)

def pile_up(arMailsList1):
    print(arMailsList1)
    global left_arm

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

    start_region_1 = PoseStamped()
    start_region_1.header.frame_id = "base"

    #the pose the baxter will first turn to
    start_region_1.pose.position.x = 0.68
    start_region_1.pose.position.y = 0
    start_region_1.pose.position.z = -0.15
    
    start_region_1.pose.orientation.x = 0
    start_region_1.pose.orientation.y = -1
    start_region_1.pose.orientation.z = 0
    start_region_1.pose.orientation.w = 0


    inter_region_1 = PoseStamped()
    inter_region_1.header.frame_id = "base"

    #the pose the baxter will first turn to
    inter_region_1.pose.position.x = 0.7
    inter_region_1.pose.position.y = 0
    inter_region_1.pose.position.z = 0.2
    
    inter_region_1.pose.orientation.x = 0
    inter_region_1.pose.orientation.y = -1
    inter_region_1.pose.orientation.z = 0
    inter_region_1.pose.orientation.w = 0


    temp_region_1 = PoseStamped()
    temp_region_1.header.frame_id = "base"

    #the pose the baxter will first turn to
    temp_region_1.pose.position.x = 0.5
    temp_region_1.pose.position.y = 0
    temp_region_1.pose.position.z = -0.15
    
    temp_region_1.pose.orientation.x = 0
    temp_region_1.pose.orientation.y = -1
    temp_region_1.pose.orientation.z = 0
    temp_region_1.pose.orientation.w = 0

    destination_1 = PoseStamped()
    destination_1.header.frame_id = "base"

    #the pose the baxter will first turn to
    destination_1.pose.position.x = 0.3
    destination_1.pose.position.y = 0
    destination_1.pose.position.z = -0.15
    
    destination_1.pose.orientation.x = 0
    destination_1.pose.orientation.y = -1
    destination_1.pose.orientation.z = 0
    destination_1.pose.orientation.w = 0

    if (1 in arMailsList1) or (3 in arMailsList1) or (12 in arMailsList1):
        start_region_1.pose.position.y = 0.6
        inter_region_1.pose.position.y = 0.6
        temp_region_1.pose.position.y = 0.6
        destination_1.pose.position.y = 0.6
    elif (0 in arMailsList1) or (9 in arMailsList1) or (15 in arMailsList1) or (16 in arMailsList1):
        start_region_1.pose.position.y = 0.3
        inter_region_1.pose.position.y = 0.3
        temp_region_1.pose.position.y = 0.3
        destination_1.pose.position.y = 0.3
  


    #initialize armailslist1
    arMailsList2 = []

    for i in range(len(arMailsList1)):
        if len(arMailsList1)!=0 and len(arMailsList2)!=0: 
            compare=[max(arMailsList1), max(arMailsList2)]
            max_flag=max(compare)
            flag=[o for o, p in enumerate(compare) if p == max_flag]
            flag=flag[0]  
        elif len(arMailsList1)==0:
            flag=1 
        elif len(arMailsList2)==0: 
            flag=0

        if flag==0:
            goto1=start_region_1
            goto2=temp_region_1
            List=arMailsList1
        else:
            goto1=temp_region_1
            goto2=start_region_1
            List=arMailsList2

        max1=max(List)
        index=[k for k, j in enumerate(List) if j == max1]
        index= index[0]

        print('initial index')
        print(index)

        print('flag')
        print(flag)

        print('max1')
        print(max1)
        # Move all the mails in front of the biggest mail,and move the biggest mail to the right region
        while index!=0:
            #Move to original pile -----------------------------------------------------
            # raw_input('Press <Enter> to move the left arm to the original_pileup_position')
            # move_left_to(start_region_1)


            # rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
            # rospy.sleep(1)
            # print(mail_range)

            #Move to starting pile  -----------------------------------------------------
            #Check using IR to see of the mail is close enough and Execute the plan
            pick_from_pile(goto1)

            #Close the Gripper
            raw_input('Press <Enter> to close the gripper')
            left_gripper.close(timeout=60)
            print('Gripper closed')
            

            #Move to temperory pile -----------------------------------------------------
            move_left_to(inter_region_1)
            #Set the goal state to the pose you just defined
            move_left_to(inter_region_1)
            rospy.sleep(1)
            print('Move to the drop-off location:')
            move_left_to(temp_region_1)
            #Release the Gripper
            raw_input('Press <Enter> to open the gripper')
            left_gripper.open(block=True)
            print('Gripper opened')

            if flag==0:
                arMailsList2.insert(0,arMailsList1[0])
                arMailsList1.remove(arMailsList1[0])
            else:
                arMailsList1.insert(0,arMailsList2[0])
                arMailsList2.remove(arMailsList2[0])

            index=index-1

            print('arMailsList1')
            print(arMailsList1)
            print('arMailsList2')
            print(arMailsList2)

        #Move to original pile -----------------------------------------------------
        print('To destination')
        raw_input('Press <Enter> to move the left arm to the original pileup position')
        pick_from_pile(goto1)

        if flag==0:
            arMailsList1.remove(max1)
        else:
            arMailsList2.remove(max1)

        print('arMailsList1')
        print(arMailsList1)
        print('arMailsList2')
        print(arMailsList2)

        #Close the Gripper
        raw_input('Press <Enter> to close the gripper')
        left_gripper.close(timeout=60)
        print('Gripper closed')

        #Check using IR to see of the mail is close enough and Execute the plan to move biggest mail to destination
        move_left_to(inter_region_1)
        rospy.sleep(1)
        move_left_to(destination_1)

        #Release the Gripper
        raw_input('Press <Enter> to open the gripper')
        left_gripper.open(block=True)
        print('Gripper opened')

if __name__ == '__main__':
    #rospy.init_node('catch1')
    pile_1,pile_2,pile_3=hand_over()
    #pile_1 = [3, 12, 1]
    #pile_2 = [0,16, 15, 9]
    #pile_3=[ 10,2]

    next=int(raw_input('Press 1 to start/continue piling up, or 0 to stop  '))
    if next:
        pile_up(pile_1)
    next=int(raw_input('Press 1 to start/continue piling up, or 0 to stop  '))
    if next:
        pile_up(pile_2)
    next=int(raw_input('Press 1 to start/continue piling up, or 0 to stop  '))
    if next:
        pile_up(pile_3)
