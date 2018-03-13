#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Char.h>

SoftwareSerial portOne(2, 3); //RX= pin 2  TX= pin 3

ros::NodeHandle  nh;

std_msgs::Char bluetooth_msg;
ros::Publisher bluetooth_pub("eos_mobile_app", &bluetooth_msg);

void setup() {
  nh.initNode();
  nh.advertise(bluetooth_pub);
  portOne.begin(9600);
}

void loop() {
  if (portOne.available() > 0)
  {
    while (portOne.available() > 0) {
      byte byteRead = portOne.read();
      bluetooth_msg.data = (char)byteRead;
      bluetooth_pub.publish(&bluetooth_msg);      
    }
  }
  nh.spinOnce();
  delay(100);
}
