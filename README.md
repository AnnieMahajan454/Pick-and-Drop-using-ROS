# Pick and Drop Simulation

## 1) Install dependencies (one-time)

Open a terminal and copy/paste:

#update
sudo apt update
sudo apt -y upgrade

#install ROS turtlebot3 + gazebo related packages
sudo apt install -y ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-nav-core ros-noetic-move-base

#(optional but recommended) teleop package
sudo apt install -y ros-noetic-turtlebot3-teleop


Add TurtleBot3 model env var permanently:

echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc

## 2) Create a catkin workspace and a package for the stockroom sim
#create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

#create package (python node + dependencies)
catkin_create_pkg stockroom_sim rospy std_msgs geometry_msgs gazebo_msgs

#go back and build
cd ~/catkin_ws
catkin_make

#source workspace
source devel/setup.bash

## 3) Create a simple cube SDF (the “item” to be moved)

Create a models folder and an SDF file:

mkdir -p ~/catkin_ws/src/stockroom_sim/models
cat > ~/catkin_ws/src/stockroom_sim/models/box.sdf <<'EOF'
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='box_template'>
    <pose>0 0 0.15 0 0 0</pose>
    <static>false</static>
    <link name='link'>
      <inertial>
        <mass>0.5</mass>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
EOF

## 4) Add the pick/drop Python node

Create the node file and make it executable:

mkdir -p ~/catkin_ws/src/stockroom_sim/scripts
cat > ~/catkin_ws/src/stockroom_sim/scripts/pick_place.py <<'PY'
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
PY

#make executable
chmod +x ~/catkin_ws/src/stockroom_sim/scripts/pick_place.py


Then rebuild your workspace so ROS knows about the node:

cd ~/catkin_ws
catkin_make
source devel/setup.bash

## 5) Start Gazebo with TurtleBot3 world, and spawn several boxes to build a "stockroom"

Open Terminal A:

#source (do this every new terminal or add to .bashrc)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

#launch TurtleBot3 world (this starts roscore + Gazebo + robot)
roslaunch turtlebot3_gazebo turtlebot3_world.launch

Wait until Gazebo shows the world and robot.

Open Terminal B (source environment there too):

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

#spawn copies of the box at several positions to form shelves / stock
#adjust positions (x y z). We create 6 boxes.
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box1 -x 1.0 -y 0.5 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box2 -x 1.0 -y 0.0 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box3 -x 1.0 -y -0.5 -z 0.15

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box4 -x 2.0 -y 0.5 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box5 -x 2.0 -y 0.0 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box6 -x 2.0 -y -0.5 -z 0.15

You should now see six cubes in Gazebo arranged like shelves/stock.

## 6) Teleoperate the robot to approach an item

Open Terminal C and run teleop:

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

#keyboard teleop (press keys shown in terminal to move)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Drive the robot so it is close (within ~0.8 m) to box1 (or whichever box you want to pick).

## 7) Pick the item (attach it to the robot)

Once the robot is near box1, open Terminal D and run:

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

#'pick' attaches the box to the robot (moves box pose to robot)
rosrun stockroom_sim pick_place.py pick box1

You should see the item jump to a pose slightly in front of the robot (simulating a pick). Move the robot around — the item will appear to follow (because its pose was placed relative to world near the robot).

## 8) Drop the item at a target location

When you want to drop, run (example dropping near x=0.5, y= -1.2, z=0.15):

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun stockroom_sim pick_place.py drop box1 0.5 -1.2 0.15

The item will instantly teleport to that world coordinate (i.e., dropped on the shelf/area).


<img width="1913" height="998" alt="image" src="https://github.com/user-attachments/assets/fabba28f-ecb2-44d4-9a8c-2be1a80979c5" />
<img width="967" height="635" alt="image" src="https://github.com/user-attachments/assets/9362ebf4-a25a-4555-a894-3379cd6f2482" />
<img width="1142" height="970" alt="image" src="https://github.com/user-attachments/assets/1037a931-353c-4b52-9dbf-cc66f09ee803" />
