#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

std_msgs::Int32 fsr_msg;
ros::Publisher fsr_pub("fsr", &fsr_msg);
int fsrAnalogPin = 6;
int fsrReading; 

void setup()
{
  nh.initNode();
  nh.advertise(fsr_pub);
}

void loop()
{
  fsr_msg.data = analogRead(fsrAnalogPin);
  fsr_pub.publish( &fsr_msg );
  nh.spinOnce();
  delay(100);
}
