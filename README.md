#Turtlebot_Burger

#To connect ESP32 bluetooth to PC.

#-connect the usb and then run the following command,


#-sudo rfcomm bind rfcomm0 "Address of Device on Bluetooth"

# At My Laptop: sudo rfcomm connect rfcomm0 EC:94:CB:6B:15:3A


#TO LAUNCH THE PROJECT, RUN THE FOLLOWING COMMAND,

-roslaunch turtlebot3_project_launch project.launch
(The Launch file will launch everything)


#For testing (cmd_vel of coming from encoders to Gazebo)

#-rostopic echo /cmd_vel

