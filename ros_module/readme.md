## Welcome to the LEAP Hand Python SDK

### On Ubuntu
- Install [ROS 1 Noetic](http://wiki.ros.org/ROS/Installation) normally first.
#### If you need an environment (venv is slightly less buggy than conda with ROS)
- `python -m venv test_env`
- `source test_env/bin/activate`
#### Install hardware drivers
- `pip install dynamixel_sdk numpy`
#### Create a [catkin_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) named catkin_ws using instructions below.  (If you do not have one from your current project)
- `mkdir -p ~/catkin_ws/src`
- Copy the ros_module folder from this Github into src
- `cd ~/catkin_ws/`
- `pip install empy catkin_pkg pyyaml rospkg` 
- `catkin_make`
#### Source Bashrc
- echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
- source ~/.bashrc
#### To Launch
- First see our example.launch for an example launch script
- cd ~/catkin_ws/src/ros_module
- `chmod +x leaphand_node.py`
- `roslaunch example.launch`

Please see ros_example.py and leaphand_node.py for further details.  It should be easy to read.  :)
