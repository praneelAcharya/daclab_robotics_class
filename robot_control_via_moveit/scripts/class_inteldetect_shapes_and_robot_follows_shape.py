#!/usr/bin/env python

from tokenize import group
import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
import geometry_msgs.msg


# Additional packages:
import pyrealsense2 as rs
import cv2
import numpy as np




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

    


    # Initialize the camera
pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# 640 is the image width and 480 is the image height
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)



# We only take N frames:
# Our goal is to do a samping and do a average method to minimize the error
Ncamera_frames = 10
detected_marker_corners = []
ALL_image_frame = []
ALL_depth_frame = []
ALL_tvecs = []


motion_triangle = 0
motion_line = 0
motion_square = 0


if __name__ == '__main__':

    # Detect a shape
    for countframes in range(0, Ncamera_frames):

      # Wait for a coherent pair of frames: depth and color
      frames = pipeline.wait_for_frames()
      color_frame = frames.get_color_frame()
      depth_frame = frames.get_depth_frame()

      # Convert images to numpy arrays
      color_image = np.asanyarray(color_frame.get_data())


      gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
      _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_OTSU)
      # _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

      # For each frame show to the user what is been seen
      # cv2.imshow('RealSense', color_image)
      # cv2.waitKey(2)

      cv2.imshow('RealSense', thresh)
      cv2.waitKey(3)

      contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
      
      (_,allcontours,_) = contours

      for contour in allcontours:

        # cnt_length = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)

        if len(approx) == 3:
          print("Its a traingle")
          motion_triangle = 1
        elif len(approx) == 4:

          rx,ry,rw,rh = cv2.boundingRect(approx)
        #   aspectRatio = float(rw/rh)
          print( aspectRatio )

          if (aspectRatio > 1.8):
            print("Its a line")
            motion_line   = 1
          
          if (aspectRatio >= 0.5) or (aspectRatio <= 1.5):
            print("This is a square")
            motion_square = 1
        
    # ---------- > END Working with camera --------------< :
    # At this point we expect that atleast a marker has been detected 
    cv2.destroyAllWindows()
    pipeline.stop()




    rospy.init_node('detect_shape_and_move')
    rospy.wait_for_message('move_group/status', GoalStatusArray)

    # This is our commander
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')

    commander.go()
    # Sleep for 3 seconds
    rospy.sleep(3.)

    # We are about to ask end effector to draw a line:
    if motion_line == 1:
        ee_fellow_line(commander)
    
    if motion_triangle == 1:
        ee_fellow_traingle(commander)

    if motion_square == 1:    
        ee_fellow_rectangle(commander)



