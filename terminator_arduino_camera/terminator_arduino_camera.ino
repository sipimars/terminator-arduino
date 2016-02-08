
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <Servo.h> 
 
#define PAN_PIN  4
#define TILT_PIN 5

Servo panServo;  
Servo tiltServo;  
int panPosition = 90; 
int tiltPosition = 105;

#define PAN_LIMIT_LEFT -70
#define PAN_LIMIT_RIGHT 60
#define TILT_LIMIT_UP 40
#define TILT_LIMIT_DOWN -70
#define PAN_CENTER 90
#define TILT_CENTER 105

ros::NodeHandle  nh;

int dir[] = {12,13};
int pwm[] = {3, 11};
int brake[] = {9, 8};
int cur[] = {0,1};
int DIR_FWD[] = {LOW,HIGH};
int DIR_REV[] = {HIGH,LOW};
int loopCount = 0;

void stopMotors(void) 
{
    for(int i=0;i<2;++i) {
      digitalWrite(brake[i], HIGH);
      analogWrite(pwm[i], 0);
    }
}

void motorCb( const geometry_msgs::Point& cmd_msg){
  if( (cmd_msg.x == 0) && (cmd_msg.y == 0) ) {
    stopMotors();
    return;
  }
  loopCount = cmd_msg.z;
  int efforts[2];
  efforts[0] = cmd_msg.x;
  efforts[1] = cmd_msg.y;
  for(int i=0;i<2;++i) {
    digitalWrite(brake[i], LOW);
    if(efforts[i] < 0) {
      digitalWrite(dir[i], DIR_REV[i]);
      analogWrite(pwm[i], -efforts[i]);
    } else {
      digitalWrite(dir[i], DIR_FWD[i]);
      analogWrite(pwm[i], efforts[i]);
    }
  }
  return;
}

void cameraCb( const geometry_msgs::Point& cmd_msg){
  int pan = cmd_msg.x;
  int tilt = cmd_msg.y;
  
  if( pan < PAN_LIMIT_LEFT) {
    pan = PAN_LIMIT_LEFT;
  } else if(pan > PAN_LIMIT_RIGHT) {
    pan = PAN_LIMIT_RIGHT;
  }
  if( tilt < TILT_LIMIT_DOWN) {
    tilt = TILT_LIMIT_DOWN;
  } else if(tilt > TILT_LIMIT_UP) {
    tilt = TILT_LIMIT_UP;
  }
  panServo.write(pan + PAN_CENTER);
  tiltServo.write(tilt + TILT_CENTER);
}             
                          
ros::Subscriber<geometry_msgs::Point> subMotor("motor_cmd", motorCb );
ros::Subscriber<geometry_msgs::Point> subCamera("cam_pt_cmd", cameraCb );

void setup() 
{ 
  for(int i=0;i<2;++i) {
    pinMode(dir[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);
    analogWrite(pwm[i], 0);
    pinMode(brake[i], OUTPUT);
  }
  nh.initNode();
  nh.subscribe(subMotor);
  nh.subscribe(subCamera);
  panServo.attach(PAN_PIN);  // attaches the servo on pin 9 to the servo object 
  tiltServo.attach(TILT_PIN);  // attaches the servo on pin 9 to the servo object 
  panServo.write(panPosition);
  tiltServo.write(tiltPosition);
} 
void loop() 
{
  nh.spinOnce();
  delay(50);
  if(loopCount >0) {
    loopCount--;
  } else {
    stopMotors();
  }
} 
