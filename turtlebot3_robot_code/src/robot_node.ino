/**********************Include the necessary libraries********************************/
#include "DifferentialDriveRobot.h"
#include <BluetoothSerial.h>
#include "Encoder.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>
#include <std_msgs/UInt16.h>


/*********************************Define the servo data*********************************/
#define Servo_pin 27
Servo myservo;
int angle = 0;
char flag=0;

/*********************************Define Ultrasonic data*********************************/
#define TRIGGER_PIN1 2
#define ECHO_PIN1 35

Ultrasonic ultrasonic1(TRIGGER_PIN1, ECHO_PIN1);

/*********************************Robot Details***************************************/

// Define the robot's wheel radius and wheel base
const float wheelRadius=0.04;
const float wheelBase=0.13;

/**********************************Define variables for time tracking*****************/
unsigned long previousMillis = 0; 
const long interval = 1000; 
const long interval2 = 50;
unsigned long currentMillis = 0;

/******************************Define a class for Bluetooth hardware******************/
BluetoothSerial SerialBT;
class BluetoothHardware {
public:
  BluetoothHardware() : connected(false) {}

  void init() 
  {
    // Start the Bluetooth connection
    SerialBT.begin("ESP32_BT");
  }

   int read() 
   {
     // Read a byte from the Bluetooth connection
     if (SerialBT.available())
       return SerialBT.read();
     else
       return -1;
   }


  void write(uint8_t* data, int length) 
  {
  // Write data to the Bluetooth connection
     for(int i=0; i<length; i++)
       SerialBT.write(data[i]);
   }


  unsigned long time() {
    // Return the current time in milliseconds
    return millis();
  }

  bool connected;
};

//Create an instance for mpu
Adafruit_MPU6050 mpu;
// Create a ROS node handle
ros::NodeHandle_<BluetoothHardware> nh;

/************************************************************************************/
// Create a pointer to a DifferentialDriveRobot object
DifferentialDriveRobot *my_robot;
/// Robot callback function
void robot_callback(const geometry_msgs::Twist& msg) 
{

  my_robot->move(msg.linear.x, msg.angular.z);
  
}


// Create a ROS subscriber
ros::Subscriber <geometry_msgs::Twist> sub("/turtle1/cmd_vel", &robot_callback);


// Create a geometry_msgs::Twist object and a ROS publisher
geometry_msgs::Twist cmd_msg;
ros::Publisher cmd_pub("cmd_vel", &cmd_msg);

// Publish IMU data
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Publish Odometry data
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

// Publish Ultrasonic data
std_msgs::UInt16 distance_msg1;
ros::Publisher distance_pub1("distance1", &distance_msg1);


/********************************** Function to handle encoder A************************************/

void handleEncoder_A() 
{
  int currentState_A = digitalRead(ENCODER_LEFT_MOTOR_A);
  int currentState_B = digitalRead(ENCODER_LEFT_MOTOR_B);
  if ((lastEncoderState_A == LOW) && (currentState_A == HIGH)) 
  {
    if (currentState_B == LOW)
    {
      encoderCount_A++;
    } 
    else 
    {
      encoderCount_A--;
    }
  }
  lastEncoderState_A = currentState_A;
}


/********************************** Function to handle encoder B************************************/

void handleEncoder_B() 
{
  int currentState_A = digitalRead(ENCODER_RIGHT_MOTOR_A);
  int currentState_B = digitalRead(ENCODER_RIGHT_MOTOR_B);
  if ((lastEncoderState_B == LOW) && (currentState_B == HIGH)) 
  {
    if (currentState_A == LOW)
    {
      encoderCount_B++;
    } 
    else 
    {
      encoderCount_B--;
    }
  }
  lastEncoderState_B = currentState_B;
}

/**********************************Set Up Function***********************************/

void setup()
{
  // Start the Bluetooth connection
  SerialBT.begin("ESP32_BT"); //Bluetooth device name
  myservo.attach(Servo_pin);
  myservo.write(0);
  // // Initialize the robot
  my_robot = new DifferentialDriveRobot(
    new DCMotor(IN1, IN2, ENA),
    new DCMotor(IN3, IN4,ENB));

  // Update the robot's parameters
  my_robot->updateParameters(0.2, 1);

  // // Try to initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  // set accelerometer range to +-2G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  // Set encoder pins as inputs
  pinMode(ENCODER_LEFT_MOTOR_A, INPUT);
  pinMode(ENCODER_RIGHT_MOTOR_A, INPUT);
  pinMode(ENCODER_LEFT_MOTOR_B, INPUT);
  pinMode(ENCODER_RIGHT_MOTOR_B, INPUT);


  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_MOTOR_A), handleEncoder_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_MOTOR_B), handleEncoder_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_MOTOR_A), handleEncoder_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_MOTOR_B), handleEncoder_B, CHANGE);


  // Initialize the ROS node, advertise the publisher, and subscribe to the subscriber
  nh.initNode();
  nh.advertise(cmd_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(distance_pub1);
  nh.subscribe(sub);

}

/**********************************Main Function**********************************/
void loop() 
{ 

  // Update the current time 
  currentMillis = millis();

  //get mpu readings#!/usr/bin/env python3
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Servo control for scanning
 

 if(currentMillis - previousMillis >= interval2)
 {
  unsigned int distance = ultrasonic1.read();
  distance_msg1.data = distance;
  distance_pub1.publish(&distance_msg1);
  if(flag==0)
  {
    angle+=1;
    myservo.write(angle);
    if(angle>=180)
    {
      flag=1;
    }
  }
  else if(flag==1)
  {
    angle-=1;
    myservo.write(angle);
    if(angle==0)
    {
      flag=0;
    }
  }
  // previousMillis = currentMillis;
 }

 
 // If the interval has passed
 if(currentMillis - previousMillis >= interval)
 {
  if(int(a.acceleration.x*10)>-10 && int(a.acceleration.x*10)<10 && int(a.acceleration.y*10)>-10 && int(a.acceleration.y*10)<10)
  {
    cmd_msg.linear.x = 0;
  }
  else{
    // Calculate the angular velocities of the encoders
    encoder_angular_A=(encoderCount_A*2*PI)/encoder_cpr;
    encoder_angular_B=(encoderCount_B*2*PI)/encoder_cpr;

    // Convert the angular velocities to a linear velocity and an angular velocity
    encoder_linearVelocity = (wheelRadius / 2) * (encoder_angular_A + encoder_angular_B);
    // Update the cmd_msg object and publish it
    cmd_msg.linear.x=encoder_linearVelocity*3;


  }
   
    //publishe angular velocity from imu
    cmd_msg.angular.z = g.gyro.z-0.05+0.019;

    //publish message
    cmd_pub.publish(&cmd_msg);
    
    // Reset the encoder counts
    encoderCount_A=0;
    encoderCount_B=0;

    // Read the ultrasonic sensor data

   
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation_covariance[0] = -1; // Indicates that orientation data is not provided
    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z;
    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;
    imu_pub.publish(&imu_msg);

    
    // Publish simplified Odometry data
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = encoder_linearVelocity;
    odom_msg.twist.twist.angular.z = g.gyro.z;
    odom_pub.publish(&odom_msg);

    // Update the previous time
    previousMillis = currentMillis; 
    
 }

  // Spin the ROS node once and delay for 1 ms
  nh.spinOnce(); 
  delay(1);
}


