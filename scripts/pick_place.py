#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
from geometry_msgs.msg import Pose, Twist

latest_models = None

def model_states_cb(msg):
    global latest_models
    latest_models = msg

def find_model_by_substr(substrings):
    # returns model name or None
    if latest_models is None:
        return None
    for name in latest_models.name:
        low = name.lower()
        for s in substrings:
            if s in low:
                return name
    return None

def get_pose_by_name(model_name):
    if latest_models is None:
        return None
    try:
        i = latest_models.name.index(model_name)
        return latest_models.pose[i]
    except ValueError:
        return None

def dist(a, b):
    return math.sqrt((a.position.x-b.position.x)**2 + (a.position.y-b.position.y)**2 + (a.position.z-b.position.z)**2)

if __name__ == '__main__':
    rospy.init_node('stockroom_pick_place', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_cb)
    rospy.loginfo("Waiting for /gazebo/model_states...")
    # wait for first message
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and latest_models is None:
        rate.sleep()

    if len(sys.argv) < 3:
        print("Usage:")
        print("  rosrun stockroom_sim pick_place.py pick <item_model_name>")
        print("  rosrun stockroom_sim pick_place.py drop <item_model_name> <x> <y> <z>")
        sys.exit(1)

    cmd = sys.argv[1].lower()
    item_name = sys.argv[2]

    # find robot model name heuristically
    robot_name = find_model_by_substr(['waffle','burger','turtlebot','robot'])
    if not robot_name:
        rospy.logerr("Could not find robot model in /gazebo/model_states. Available models: %s", latest_models.name)
        sys.exit(1)
    rospy.loginfo("Using robot model name: %s", robot_name)

    if cmd == 'pick':
        item_pose = get_pose_by_name(item_name)
        if not item_pose:
            rospy.logerr("Item '%s' not found in gazebo model states.", item_name)
            sys.exit(1)
        robot_pose = get_pose_by_name(robot_name)
        if not robot_pose:
            rospy.logerr("Robot pose not found.")
            sys.exit(1)
        d = dist(robot_pose, item_pose)
        rospy.loginfo("Distance robot <-> item = %.3f m", d)
        if d > 0.8:
            rospy.logerr("Robot is too far from item. Move robot within 0.8 m and try again.")
            sys.exit(1)

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            new_state = ModelState()
            new_state.model_name = item_name
            # place item slightly in front of the robot and above ground
            # use robot_pose as base
            new_state.pose.position.x = robot_pose.position.x + 0.2
            new_state.pose.position.y = robot_pose.position.y
            new_state.pose.position.z = robot_pose.position.z + 0.2
            new_state.pose.orientation = robot_pose.orientation
            new_state.twist = Twist()
            new_state.reference_frame = 'world'
            set_state(new_state)
            rospy.loginfo("Picked item '%s' (attached to robot frame)", item_name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            sys.exit(1)

    elif cmd == 'drop':
        if len(sys.argv) < 6:
            print("Usage for drop: rosrun stockroom_sim pick_place.py drop <item_model_name> <x> <y> <z>")
            sys.exit(1)
        tx = float(sys.argv[3])
        ty = float(sys.argv[4])
        tz = float(sys.argv[5])
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            new_state = ModelState()
            new_state.model_name = item_name
            new_state.pose.position.x = tx
            new_state.pose.position.y = ty
            new_state.pose.position.z = tz
            new_state.pose.orientation.x = 0.0
            new_state.pose.orientation.y = 0.0
            new_state.pose.orientation.z = 0.0
            new_state.pose.orientation.w = 1.0
            new_state.twist = Twist()
            new_state.reference_frame = 'world'
            set_state(new_state)
            rospy.loginfo("Dropped item '%s' at (%.2f, %.2f, %.2f)", item_name, tx, ty, tz)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            sys.exit(1)
    else:
        rospy.logerr("Unknown command: %s", cmd)
        sys.exit(1)
