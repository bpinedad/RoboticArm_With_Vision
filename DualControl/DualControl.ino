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
  //Servo 2 Up - DOwn
  //Servo 3 Reach
  //Servo 4 Open - Close

}
/*
//claw
void zhuazi()
{
  Serial.println(x2);
    //claw
  if(x2<minValue) // if push the left joystick to the right
  {
      pos4=pos4-2;  //current angle of servo 4 subtracts 2（change the value you subtract, thus change the closed speed of claw）
      //Serial.println(pos4);
      myservo4.write(pos4);  // servo 4 operates the action, claw is gradually closed.
      delay(delayValue);
      if(pos4<2)  // if pos4 value subtracts to 2, the claw in 37 degrees we have tested is closed.
      {            //（should change the value based on the fact）
        pos4=2;   // stop subtraction when reduce to 2
      }
   }
  if(x2>maxValue) //// if push the left joystick to the left 
  {
      pos4=pos4+8; // current angle of servo 4 plus 8（change the value you plus, thus change the open speed of claw）
      //Serial.println(pos4);
      myservo4.write(pos4); // servo 4 operates the motion, the claw gradually opens.
      delay(delayValue);
      if(pos4>108)  // limit the largest angle when open the claw  
      {
        pos4=108;
      }
  }
}

 // turn
void zhuandong()
{
  if(x1<minValue)  // if push the right joystick to the right
  {
    pos1=pos1-1;  //pos1 subtracts 1
    myservo1.write(pos1);  // servo 1 operates the motion, the arm turns right.
    delay(delayValue);
    if(pos1<1)   // limit the angle when turn right
    {
      pos1=1;
    }
  }
  if(x1>maxValue)  // if push the right joystick to the let
  {
    pos1=pos1+1;  //pos1 plus 1
    myservo1.write(pos1);  // arm turns left 
    delay(delayValue);
    if(pos1>180)  // limit the angle when turn left 
    {
      pos1=180;
    }
  }
}

//upper arm
void xiaobi()
{
    if(y1>maxValue) // if push the right joystick upward
  {
    pos2=pos2-1;
    myservo2.write(pos2); // the upper arm will lift
    delay(delayValue);
    if(pos2<0)  // limit the lifting angle
    {
      pos2=0;
    }
  }
  if(y1<minValue)  // if push the right joystick downward
  {
    pos2=pos2+1;  
    myservo2.write(pos2);  // upper arm will go down  
    delay(delayValue);
    if(pos2>180)  // limit the angle when go down
    {
      pos2=180;
    }
  }
}

// lower arm
void dabi()
{
    if(y2<minValue)  // if push the left joystick upward 
  {
    pos3=pos3+1;
    myservo3.write(pos3);  // lower arm will stretch out 
    delay(delayValue);
    if(pos3>180)   // limit the stretched angle 
    {
      pos3=180;
    }
  }
  if(y2>maxValue)  // if push the left joystick downward
  {
    pos3=pos3-1;
    myservo3.write(pos3);  // lower arm will draw back
    delay(delayValue);
    if(pos3<35)  // limit the retracted angle 
    {
      pos3=35;
    }
  }
}
*/
