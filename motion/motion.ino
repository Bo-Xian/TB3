#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(1000000);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(3,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //int sensorValue = digitalRead(3);
  //Serial.println(sensorValue);

  digitalWrite(9,HIGH);
  digitalWrite(8,LOW);
  analogWrite(3, 100);
}
