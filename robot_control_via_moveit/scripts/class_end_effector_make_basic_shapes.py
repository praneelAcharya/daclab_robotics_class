#!/usr/bin/env python

from tokenize import group
import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
import geometry_msgs.msg




def ee_fellow_line(group):

    start_pose = group.get_current_pose()
    goal_pose  = geometry_msgs.msg.Pose()

    goal_pose.orientation.x = start_pose.pose.orientation.x
    goal_pose.orientation.y = start_pose.pose.orientation.y
    goal_pose.orientation.z = start_pose.pose.orientation.z
    goal_pose.orientation.w = start_pose.pose.orientation.w


    # Make the first length movement

    dx = input("Enter the length (delx) in x-direction you want to move:")
    print("Is this what you entered --> ", dx, " <-- if correct press enter else Cntrl-c")
    raw_input()

    if (dx >= 0.05) and (dx <= 0.2):
        goal_pose.position.x = start_pose.pose.position.x + 0.2
        goal_pose.position.y = 0
        goal_pose.position.z = start_pose.pose.position.z

        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        goal_pose.position.x = start_pose.pose.position.x
        goal_pose.position.y = start_pose.pose.position.y
        goal_pose.position.z = start_pose.pose.position.z

        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

    else:
        print("Sorry the value is outside the range 0.05 - 0.2")

    return None


def ee_fellow_traingle(group):

    start_pose = group.get_current_pose()
    goal_pose  = geometry_msgs.msg.Pose()

    goal_pose.orientation.x = start_pose.pose.orientation.x
    goal_pose.orientation.y = start_pose.pose.orientation.y
    goal_pose.orientation.z = start_pose.pose.orientation.z
    goal_pose.orientation.w = start_pose.pose.orientation.w

    side_length = input("Enter the triangle side length you want to move:")
    print("Is this what you entered --> ", side_length, " <-- if correct press enter else Cntrl-c")
    raw_input()

    if (side_length >= 0.05) and (side_length <= 0.2):
        # Make the first length movement sin (45 deg) = 0.707
        goal_pose.position.x = start_pose.pose.position.x + 0.707*side_length
        goal_pose.position.y = start_pose.pose.position.y - 0.707*side_length
        goal_pose.position.z = start_pose.pose.position.z

        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        goal_pose.position.x = start_pose.pose.position.x
        goal_pose.position.y = start_pose.pose.position.y - 2*0.707*side_length
        goal_pose.position.z = start_pose.pose.position.z

        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        goal_pose.position.x = start_pose.pose.position.x
        goal_pose.position.y = start_pose.pose.position.y
        goal_pose.position.z = start_pose.pose.position.z

        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

    else:
        print("Sorry the value is outside the range 0.05 - 0.2")

    return None


def ee_fellow_rectangle(group):

    start_pose = group.get_current_pose()
    goal_pose  = geometry_msgs.msg.Pose()

    goal_pose.orientation.x = start_pose.pose.orientation.x
    goal_pose.orientation.y = start_pose.pose.orientation.y
    goal_pose.orientation.z = start_pose.pose.orientation.z
    goal_pose.orientation.w = start_pose.pose.orientation.w

    square_length = input("Enter the square side:")
    print("Is this what you entered", square_length, "if correct press enter else Cntrl-c")
    raw_input()

    if (square_length >= 0.05) and (square_length <= 0.2):

        # Make the first length movement
        goal_pose.position.x = start_pose.pose.position.x + square_length
        goal_pose.position.y = 0
        goal_pose.position.z = start_pose.pose.position.z

        # Pass this as the target pose
        group.set_pose_target(goal_pose)
        # Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # Clear targets after planning
        group.clear_pose_targets()
        rospy.sleep(1)

        # Make the next move in y-direction
        goal_pose.position.y = square_length
        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        # Make the next move in x-direction
        goal_pose.position.x = start_pose.pose.position.x
        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        # Make the next move in y-direction
        goal_pose.position.y = start_pose.pose.position.y
        group.set_pose_target(goal_pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

    else:
        print("Sorry the value is outside the range 0.05 - 0.2")

    return None

    


if __name__ == '__main__':

    rospy.init_node('move_to_different_locations')
    rospy.wait_for_message('move_group/status', GoalStatusArray)

    # This is our commander
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')

    
    commander.go()
    # Sleep for 3 seconds
    rospy.sleep(3.)

    # We are about to ask end effector to draw a line:
    ee_fellow_line(commander)


    ee_fellow_traingle(commander)

    ee_fellow_rectangle(commander)


    # # Print the current pose:
    # current_pose = commander.get_current_pose()
    # print( current_pose )
    # goal_pose = geometry_msgs.msg.Pose()

    # goal_pose.orientation.x = current_pose.pose.orientation.x
    # goal_pose.orientation.y = current_pose.pose.orientation.y
    # goal_pose.orientation.z = current_pose.pose.orientation.z
    # goal_pose.orientation.w = current_pose.pose.orientation.w

    # goal_pose.position.x = current_pose.pose.position.x + 0.2
    # goal_pose.position.y = 0
    # goal_pose.position.z = 0.6


    # # Pass this as the target pose
    # commander.set_pose_target(goal_pose)
    # # Now, we call the planner to compute the plan and execute it.
    # plan = commander.go(wait=True)
    # # Calling `stop()` ensures that there is no residual movement
    # commander.stop()
    # # Clear targets after planning
    # commander.clear_pose_targets()




