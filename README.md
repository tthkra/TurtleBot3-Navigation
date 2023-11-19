# TurtleBot3-Navigation
navigation through an environment with TurtleBot3.

in order to let the robot navigate through a given environment, you need first to map that environment and save it using SLAM method. if you haven't done that, you can follow this repo: https://github.com/tthkra/TurtleBot3.

## Navigation with TurtleBot3
navigation is moving the robot from one location to another determined location within an environment. when simulating the navigation of a robot, you can achieve that in several ways, like using visulaization program to directly interact with the robot's environment and control the robot as well, or, using python script to programatically control and define the robot's navigation goals. i will use both ways to specify navigation goals for the robot.

### Navigation with RViz
to start the navigation, launch Gazebo first if you want to see the simulated world for the robot
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
launch RViz to start the navigation
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
<img width="582" alt="Screenshot 2023-11-17 225305" src="https://github.com/tthkra/TurtleBot3-Navigation/assets/142266646/fccd566e-fa68-40df-afbf-7c30f2efeffb"> <br>
> note: if you encounter an error message like "Failed to create the dwa_local_planner/DWAPlannerROS planner," install the dwa_local_planner package using the command: `sudo apt-get install ros-melodic-dwa-local-planner
` . then rerun the navigation command. <br>

from the above list click on "2D Pose Estimate", and then on the map try to click on the same location of the robot as its location on gazebo to set the initial pose of the robot. <br>
<img width="569" alt="Screenshot 2023-11-17 233420" src="https://github.com/tthkra/TurtleBot3-Navigation/assets/142266646/954da14a-c7cf-4b27-be74-ede15721da4d"> <br>
click on "2D Nav Goal" to set a goal location for the robot. then choose any point on the map you want the robot to navigate to. once you click one, the robot will start moving to that location
<img width="568" alt="Screenshot 2023-11-17 233836" src="https://github.com/tthkra/TurtleBot3-Navigation/assets/142266646/20330272-08e0-42ec-8ef6-2861b6ef6eff">
<img width="568" alt="Screenshot 2023-11-17 233927" src="https://github.com/tthkra/TurtleBot3-Navigation/assets/142266646/783a1d5e-2f8b-4a76-9eb9-2f69650c8ff1"> 

### Navigation with Python Script
to control the robot's navigation goals through a ROS node, first create a new package that will contain the python script 
```
cd ~/catkin_ws/src
catkin_create_pkg navigation_publisher rospy
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
create a python script file inside the src directory of the newly created package and specify the navigation goals coordinates. you can use this script 
```
#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
import math

def publish_waypoints():
    """
    Publish the ROS message containing the waypoints
    """
    rospy.init_node('waypoint_publisher', anonymous=True)

    waypoint_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        waypoint_publisher.publish(pose)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
```
go to your script directory and change the permission of the script. (change "script" to your script file name)
```
cd src/navigation_publisher/src
chmod +x script.py
```
finally, run the node
```
source ~/catkin_ws/devel/setup.bash
rosrun navigation_publisher script.py
```
once you run the node, the script will start publishing to /move_base_simple/goal topic, which is the one resposible for sending the navigation goals. you can see the robot start moving to the specefied location
<img width="595" alt="Screenshot 2023-11-19 220708" src="https://github.com/tthkra/TurtleBot3-Navigation/assets/142266646/b7dd35b0-2078-40c3-aaf0-598435f2c6c4">
<img width="594" alt="Screenshot 2023-11-19 220759" src="https://github.com/tthkra/TurtleBot3-Navigation/assets/142266646/d7de6154-60dc-47a1-92dd-64c1d645378b">



