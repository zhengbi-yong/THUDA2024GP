## Welcome to the LEAP Hand SDK

#### Hardware Setup
- Connect 5v power to the hand (the dynamixels should light up during boot up.)
- Connect the Micro USB cable to the hand (Do not use too many USB extensions)
- Open [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/) and find the correct port using the options button and put that in main.py or ros_example.py.
- On Ubuntu you can find the hand by ID using `/dev/serial/by-id` The ID will stay persistent on reboots.
- We offically support Python and ROS, but other languages are supported by [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).

#### Functionality
- Leap Node allows you to command joint angles in different scalings.
- You can read position, velocity and current from the hand.  
- Do not query reads too often, going past 90hz for one set of angles or 30hz for all three will slow down the USB communication.
- The default controller follows the PID control, up to the current limit cap. 
- Other controllers including velocity control or current control are supported as per the [motor manual](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/)
- For Lite, keep the current limit around 350ma.
- For Full, you can raise the current limit up to 550ma.
- If facing a jittery hand, adjust the PID values down.
- If the hand is too weak, adjust the PID values up.

#### Troubleshooting
- If your motor is 90/180/270 Degrees off, the horn is mounted incorrectly on the motor.  Remount it.
- If no motors show up, check that your serial port permissions are correct.
- If some motors are missing, make sure they are IDed corrrectly and are connected to the U2D2.

#### Coming Soon:
- MANO to LEAP joint angle mapping.

#### Support:
- Please contact me at kshaw2@andrew.cmu.edu for any issues.
- By using this code you agree to our CC BY-NC-SA Attribution-NonCommercial-ShareAlike license which allows you to use and build upon our work non-commercially.
- LEAP Hand is provided as-is and without warranty.