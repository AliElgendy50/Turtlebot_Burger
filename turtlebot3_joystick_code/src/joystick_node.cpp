#include <JOYSTICK.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>

int x = 0; 
int y = 0;
int max_speed = 4;

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

  x = get_x_pos();
  y = get_y_pos();

  x = map(x, 0, 1023, -max_speed, max_speed) +1; 
  y = map(y, 0, 1023, max_speed, -max_speed) -1;

  joystick_msg.linear.x = x;
  joystick_msg.angular.z = y;

  joystick_pub.publish(&joystick_msg);

  nh.spinOnce();

  delay(10);
}
