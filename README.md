#Turtlebot_Burger

#To connect ESP32 bluetooth to PC.

#-connect the usb and then run the following command,

#-sudo rfcomm bind rfcomm0 "Address of Device on Bluetooth"
At My Laptop: sudo rfcomm connect rfcomm0 EC:94:CB:6B:15:3A

#Run the following commands to launch the Project: #Run the commands in order,

#-roscore

#-rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 __name:=com2

#-rosrun turtlebot3_laptop_code laptop.py

#-rosrun rosserial_python serial_node.py _port:=/dev/rfcomm0 _baud:=115200 __name:=com1

#Then for GAZEBO:

#-export TURTLEBOT3_MODEL=burger

#-roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

#For testing (cmd_vel of coming from encoders to Gazebo)

#-rostopic echo /cmd_vel


(OR RUN THE LAUNCH FILE DIRECT)


#TO LAUNCH THE PROJECT, RUN THE FOLLOWING COMMAND, 

-roslaunch turtlebot3_project_launch project.launch (The Launch file will launch everything)



#TO LAUNCH THE OBJECT AVOIDANCE PROJECT, RUN THE FOLLOWING,

(1) change the url in the object_detection.py according to your ip camera.

(2) the "yolo8" model is found inside the "turtlebot3_laptop_code" package. So edit the path to model in the object_detection.py.

(3) run the following command,
#-roslaunch turtlebot3_project_launch obstacle_avoidance.launch



