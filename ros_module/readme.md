## Welcome to the LEAP Hand Python SDK

### On Ubuntu
- Install [ROS 1 Noetic](http://wiki.ros.org/ROS/Installation) normally first on Ubuntu 20.04.
#### If you need an environment (venv is slightly less buggy than conda with ROS)
- `python -m venv test_env`
- `source test_env/bin/activate`
#### Add this to a [catkin_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) named catkin_ws using instructions below.  
- `mkdir -p ~/catkin_ws/src` (If you do not have a workspace from your current project)
- Next, copy the ros_module folder from this Github into the newly created src folder.  Then:
- `cd ~/catkin_ws/`
- `pip install empy catkin_pkg pyyaml rospkg` 
- `catkin_make`
#### Setup Bashrc
- `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
- `source ~/.bashrc`
#### First time preparation
- `pip install dynamixel_sdk numpy`
-  cd ~/catkin_ws/src/ros_module
- `chmod +x leaphand_node.py`
#### To Launch
- Connect 5v power to the hand (the dynamixels should light up during boot up.)
- Connect the Micro USB cable to the hand (Do not use too many USB extensions)
- Find the USB port using [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
- `roslaunch example.launch`
- This is an example Launch script for just LEAP Hand.  The Hand should come to life.  You can query the services to receive data and publish to command the hand.
Please see ros_example.py for example code that uses this module and leaphand_node.py, the actual ros module, for further details.  It should be easy to read.  :)
