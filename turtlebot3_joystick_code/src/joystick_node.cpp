#include <JOYSTICK.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>

//Initialize data
int x = 0; 
int y = 0;

//Initialize Node and publisher
ros::NodeHandle nh; 

geometry_msgs::Twist joystick_msg; 

ros::Publisher joystick_pub("/joystick", &joystick_msg);

void setup() 
{

  Serial.begin(57600);
  nh.initNode(); 
  nh.advertise(joystick_pub);

}
void loop() {

  //Read data
  x = get_x_pos()-508;
  y = get_y_pos()-510;
  // map x and y values to be x(-5,5) and y(-77,77)
  if(x>0)
  {
    x=map(x,0,515,0,5);
  }
  if(x<0)
  {
    x=map(x,0,-508,0,-5);
  }
  if(y>0)
  {
    y=map(y,0,513,0,77);
  }
  if(y<0)
  {
    y=map(y,0,-510,0,-77);
  }

  //Publish data
  joystick_msg.linear.x = x;
  joystick_msg.angular.z = y;

  joystick_pub.publish(&joystick_msg);

  nh.spinOnce();

  delay(10);
}
