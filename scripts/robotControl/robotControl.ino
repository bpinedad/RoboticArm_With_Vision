#include <Servo.h>  
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

//Robot servos
Servo servo1; 
Servo servo2;
Servo servo3;
Servo servo4;  

const int minValue = 75;
const int maxValue = 600;
const int delayValue = 10;
int pos1=80, pos2=60, pos3=130, pos4=0;  // define the variable of 4 servo angle and assign the initial value( that is the boot posture angle value)
int hand[4] = {pos1,pos2,pos3,pos4}; //0 Close, 1 Open
ros::NodeHandle nh;

void messageCb( const std_msgs::Int32MultiArray& val){
  for (int i=0; i<4; i++){
    hand[i] = val.data[i];
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("hand_status", &messageCb );

void setup() 
{
  // startup posture
  servo1.write(pos1);  
  delay(1000);
  servo2.write(pos2);  
  servo3.write(pos3);  
  servo4.write(pos4); 
  delay(1000);

  Serial.begin(9600);   // set the baud rate to 9600

  //Ros subscriber init
  nh.initNode();
  nh.subscribe(sub);
}

void loop() 
{
  //Set Pins
  servo1.attach(A1);  // set the control pin of servo 1 to A1
  servo2.attach(A0);  // set the control pin of servo 2 to A0
  servo3.attach(6);   //set the control pin of servo 3 to D6
  servo4.attach(9);   // set the control pin of servo 4 to D9
  
  nh.spinOnce();

  //Write Servo values
  servo1.write(hand[0]);
  servo2.write(hand[1]);
  servo3.write(hand[2]);
  servo4.write(hand[3]);
  
  delay(delayValue);
}