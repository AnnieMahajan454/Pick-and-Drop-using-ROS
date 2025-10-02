# Pick and Drop Simulation

A ROS-based simulation for picking and dropping boxes using TurtleBot3 in Gazebo.

---
## Install Dependencies


<summary>Installation steps</summary>

### Update System
```bash
sudo apt update
sudo apt -y upgrade
```
Install ROS TurtleBot3 & Gazebo Packages
```
sudo apt install -y ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-nav-core ros-noetic-move-base
```
(Optional) Teleop Package
```
sudo apt install -y ros-noetic-turtlebot3-teleop
#Set TurtleBot3 Model Permanently

echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

Create Catkin Workspace

<summary>Click to expand workspace setup</summary>

#Create workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
#Create package with Python node + dependencies
```
catkin_create_pkg stockroom_sim rospy std_msgs geometry_msgs gazebo_msgs
```

#Build workspace
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
#Create Cube Model
<summary>Click to expand SDF creation steps</summary>

```
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
```
Add Pick/Drop Node
<summary>Click to expand Python node</summary>

```
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
    robot_name = find_model_by_substr(['waffle','burger','turtlebot','robot'])
    if not robot_name:
        rospy.logerr("Robot model not found in /gazebo/model_states")
        sys.exit(1)
    rospy.loginfo("Using robot model: %s", robot_name)

    if cmd == 'pick':
        item_pose = get_pose_by_name(item_name)
        robot_pose = get_pose_by_name(robot_name)
        if not item_pose or not robot_pose:
            rospy.logerr("Item or robot pose not found")
            sys.exit(1)
        d = dist(robot_pose, item_pose)
        if d > 0.8:
            rospy.logerr("Robot too far from item (>0.8 m)")
            sys.exit(1)
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            new_state = ModelState()
            new_state.model_name = item_name
            new_state.pose.position.x = robot_pose.position.x + 0.2
            new_state.pose.position.y = robot_pose.position.y
            new_state.pose.position.z = robot_pose.position.z + 0.2
            new_state.pose.orientation = robot_pose.orientation
            new_state.twist = Twist()
            new_state.reference_frame = 'world'
            set_state(new_state)
            rospy.loginfo("Picked item '%s'", item_name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            sys.exit(1)

    elif cmd == 'drop':
        if len(sys.argv) < 6:
            print("Usage: rosrun stockroom_sim pick_place.py drop <item_model_name> <x> <y> <z>")
            sys.exit(1)
        tx, ty, tz = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            new_state = ModelState()
            new_state.model_name = item_name
            new_state.pose.position.x = tx
            new_state.pose.position.y = ty
            new_state.pose.position.z = tz
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
```

```
chmod +x ~/catkin_ws/src/stockroom_sim/scripts/pick_place.py
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Start Gazebo and Spawn Boxes
<summary>Click to expand Gazebo setup</summary>
Terminal A: Launch TurtleBot3 world

```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Terminal B: Spawn boxes
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box1 -x 1.0 -y 0.5 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box2 -x 1.0 -y 0.0 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box3 -x 1.0 -y -0.5 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box4 -x 2.0 -y 0.5 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box5 -x 2.0 -y 0.0 -z 0.15
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/stockroom_sim/models/box.sdf -sdf -model box6 -x 2.0 -y -0.5 -z 0.15
```
Teleoperate the Robot
<summary>Click to expand teleop instructions</summary>

```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Drive the robot close (~0.8 m) to the box you want to pick.


Pick an Item
<summary>Click to expand pick instructions</summary>

```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun stockroom_sim pick_place.py pick box1
```

Drop an Item
<summary>Click to expand drop instructions</summary>

```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun stockroom_sim pick_place.py drop box1 0.5 -1.2 0.15
```
---
## Screenshots

<img width="1913" height="998" alt="image" src="https://github.com/user-attachments/assets/fabba28f-ecb2-44d4-9a8c-2be1a80979c5" />
<img width="967" height="635" alt="image" src="https://github.com/user-attachments/assets/9362ebf4-a25a-4555-a894-3379cd6f2482" />
<img width="1142" height="970" alt="image" src="https://github.com/user-attachments/assets/1037a931-353c-4b52-9dbf-cc66f09ee803" />
