#Turtlebot_Burger
#To connect ESP32 bluetooth to PC.
#-connect the usb and then run the following command,
#-sudo rfcomm bind rfcomm0 "Address of Device on Bluetooth"
#Run the following commands to launch the Project:
#Run the commands in order,
#-roscore
#-rosrun rosserial_python serial_node.py /dev/ttyACM0
#-rosrun turtlebot3_bringup laptop.py
#-rosrun rosserial_python serial_node.py _port:=/dev/rfcomm0 _baud:=115200 __name:=com1
#Then for GAZEBO:
#-export TURTLEBOT3_MODEL=burger
#-roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
#For testing run the command,
#-rostopic echo /test
#For testing (cmd_vel of coming from encoders to Gazebo)
#-rostopic echo /cmd_vel
