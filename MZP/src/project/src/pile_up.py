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

listener = None

def quat_to_rbt(pos, rot):
    omega, theta = eqf.quaternion_to_exp(np.array([rot[0], rot[1], rot[2], rot[3]]))
    rbt = eqf.create_rbt(omega, theta, np.array([pos[0], pos[1], pos[2]]))
    return rbt

def callback(message):
    global mail_range
    mail_range = message.range

def callbackAR(message):
    global ar_name
    try:
        # TODO read multiple ar tags and return their numbers
        ar_name = message.markers[0].id
    except:
        a=1

def move_left_to(goal):
    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(goal)
    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()
    #Plan a path
    left_plan = left_arm.plan()
    #Execute the plan
    left_arm.execute(left_plan)

def pick_from_pile(goal,z_pos):
    rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
    rospy.sleep(1)
    print(mail_range)

    #Move to starting pile  -----------------------------------------------------
    #Check using IR to see of the mail is close enough
    incre = 0.01
    while mail_range > 0.49 and incre < 0.03:
        print('Too far from the mail. Trying again...')
        rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
        rospy.sleep(1)
        print(mail_range)
        goal.pose.position.z = float(z_pos) + 0.06 - incre
        incre += 0.01
        #Set the goal state to the pose you just defined
        left_arm.set_pose_target(goal)
        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()
        #Plan a path
        pick_mail_plan = left_arm.plan()
         
        #Execute the plan
        raw_input('Press <Enter> to move the left arm to the pile 1 (path constraints are never enforced during this motion): ')
        left_arm.execute(pick_mail_plan)
        print(mail_range)

def pick_from_hand(goal,y_pos):
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
        left_arm.set_pose_target(goal)

        #Set the start state for the left arm
        left_arm.set_start_state_to_current_state()
        #Plan a path
        left_plan = left_arm.plan()
     
        #Execute the plan
        raw_input('Press <Enter> to move the left arm to get the mail: ')
        left_arm.execute(left_plan)
    print(mail_range)

# TODO: Hard code in the sizes of the ar markers
# TODO: Modify goal poses (three possible ones for dropping the mail, remeber to check for moveit performance first!)
# TODO: Algorithm code up



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
    ######################################
    #          Hand over                 #
    ######################################
    input = 1

    arMailsList1=[]
    arMailsList2=[]
    arMailsList3=[]

    while input:
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
    		    break
    		except:
    		    continue

	    arTagName = 'ar_marker_' + str(ar_name)
	    

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
        
	    pick_from_hand(goal_1,y_pos)
	    
	    #Close the Gripper
	    raw_input('Press <Enter> to close the gripper')

	    left_gripper.close(timeout=60)
	    print('Gripper closed')

	    #Second goal pose -----------------------------------------------------
	    rospy.sleep(2.0)
	    goal_2 = PoseStamped()
	    goal_2.header.frame_id = "base"

        

	    if ar_name in (0, 9):
    		  #x, y, and z position
    		goal_2.pose.position.x = 0.7
    		goal_2.pose.position.y = 0.6
            goal_2.pose.position.z = -0.1
            print(ar_name)
                #arMailsList1.append(ar_name)
                #arMailsList1.reverse()       
	    elif ar_name in (1, 15, 14, 6):
    		#x, y, and z position
    		goal_2.pose.position.x = 0.7
    		goal_2.pose.position.y = 0.3
    		goal_2.pose.position.z = -0.1
            arMailsList2.append(ar_name)
            arMailsList2.reverse() 
	    else:
    		#x, y, and z position
    		goal_2.pose.position.x = 0.7
    		goal_2.pose.position.y = 0
    		goal_2.pose.position.z = -0.1
            arMailsList3.append(ar_name)
            arMailsList3.reverse() 

	    print('arMailsList1'+arMailsList1)
        print('arMailsList2'+arMailsList2)
        print('arMailsList3'+arMailsList3)

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
        input=raw_input('Press 1 to start/contine moving, or 0 to stop moving')


    ######################################
    #            Pile up                 #
    ######################################
    # List 1

    # TODO Assign Position
    original_pileup_position = PoseStamped()
    original_pileup_position.header.frame_id = "base"

    #the pose the baxter will first turn to
    original_pileup_position.pose.position.x = 0.7
    original_pileup_position.pose.position.y = 0.6
    original_pileup_position.pose.position.z = 0.2
    
    original_pileup_position.pose.orientation.x = 0
    original_pileup_position.pose.orientation.y = -1
    original_pileup_position.pose.orientation.z = 0
    original_pileup_position.pose.orientation.w = 0

    start_region_1 = original_pileup_position
    start_region_1.pose.position.z=-0.2

    temp_region_1 = start_region_1
    temp_region_1.pose.position.x = 0.6

    destination_1 = start_region_1
    temp_region_1.pose.position.x = 0.5

    for i in range(len(arMailsList1)):
        switch=['start','temp']
        if i%2==0:
            goto_1,goto_2=start_region_1,temp_region_1
        else:
            goto_1,goto_2=temp_region_1,start_region_1

        max1=max(arMailsList1)
        index=max(enumerate(arMailsList1))

        # Move all the mails in front of the biggest mail,and move the biggest mail to the right region
        while index!=0:
            #Move to original pile -----------------------------------------------------
            raw_input('Press <Enter> to move the left arm to the original_pileup_position')
            move_left_to(original_pileup_position)


            rospy.Subscriber('/robot/range/left_hand_range/state',Range, callback)
            rospy.sleep(1)
            print(mail_range)

            #Move to starting pile  -----------------------------------------------------
            #Check using IR to see of the mail is close enough and Execute the plan
            pick_from_pile(goto_1,z_pos)
            

            #Move to temperory pile -----------------------------------------------------
            #Set the goal state to the pose you just defined
            raw_input('Press <Enter> to move the temperory region: ')
            move_left_to(goto_2)
                        
            #Close the Gripper
            raw_input('Press <Enter> to close the gripper')
            left_gripper.close(timeout=60)
            print('Gripper closed')

            index=index-1

        #Move to original pile -----------------------------------------------------
        raw_input('Press <Enter> to move the left arm to the original_pileup_position')
        move_left_to(original_pileup_position)    


        #Check using IR to see of the mail is close enough and Execute the plan to move biggest mail to destination
        pick_from_pile(destination_1,z_pos)

        #Close the Gripper
        raw_input('Press <Enter> to close the gripper')
        left_gripper.close(timeout=60)
        print('Gripper closed')

        arMailsList1.remove(max1)

if __name__ == '__main__':
    #rospy.init_node('catch1')
    main()
