## Welcome to the LEAP Hand Python SDK

#### On Ubuntu
- Install [ROS 1 Noetic](http://wiki.ros.org/ROS/Installation) normally first.
- `python -m venv test_env`
- `source test_env/bin/activate`
- `pip install dynamixel_sdk numpy`
- Copy this LEAP ros_module folder to your project and run `catkin_make` in the root of it.
- `catkin_make` is required to generate the [ROS services](http://wiki.ros.org/Services).

Please see ros_example.py and leaphand_node.py for further details.  It should be easy to read.  :)
