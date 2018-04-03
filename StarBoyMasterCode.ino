#include <Servo.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_Conveyor;
Servo servo_Push_Stick;
    
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//Assigning port pins
const int ci_pushButton_2 = 12;
const int ci_pushButton_3 = 13;

const int ci_Ultrasonic_Front_Ping = 2;
const int ci_Ultrasonic_Front_Data = 3;

const int ci_Ultrasonic_Right2_Ping = 4;
const int ci_Ultrasonic_Right2_Data = 5;

const int ci_Ultrasonic_Right1_Ping = 6;
const int ci_Ultrasonic_Right1_Data = 7;

const int ci_Left_Motor = 8;
const int ci_Right_Motor = 9;
const int ci_Conveyor = 10;
const int ci_Push_Stick = 11;

const int ci_I2C_SDA = A4;   //I2C data = white
const int ci_I2C_SCL = A5;   //I2C clock = yellow


//variables
int phase = 0;
double PingFreturn;
double PingR1return;
double PingR2return; 

unsigned long leftMotorSpeed;
unsigned long rightMotorSpeed;

unsigned long ultraSonicDistanceF;
unsigned long ultraSonicDistanceR2;
unsigned long ultraSonicDistanceR1;

unsigned int button2Counter = 0;
unsigned int button3Counter = 0;

#define DEBUG_ULTRASONIC

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);

  //Button Set-Up
  pinMode(ci_pushButton_2, INPUT);
  digitalWrite(ci_pushButton_2, HIGH);
  pinMode(ci_pushButton_3, INPUT);
  digitalWrite(ci_pushButton_3, HIGH);
  
  //Ultrasonic Set-up
  pinMode(ci_Ultrasonic_Front_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Front_Data, INPUT);

  pinMode(ci_Ultrasonic_Right2_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Right2_Data, INPUT);

  pinMode(ci_Ultrasonic_Right1_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Right1_Data, INPUT);

  //Motor Set-Up
  
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  
  pinMode(ci_Conveyor, OUTPUT);
  servo_Conveyor.attach(ci_Conveyor);
  
  pinMode(ci_Push_Stick, OUTPUT);
  servo_Push_Stick.attach(ci_Push_Stick);

  //Encoder Set up with the Daisy Chain connection
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);
  }

double PingF()
{
  int echoTime;
  digitalWrite(ci_Ultrasonic_Front_Ping, HIGH);
  delayMicroseconds(10);  
  digitalWrite(ci_Ultrasonic_Front_Ping, LOW);
  echoTime = pulseIn(ci_Ultrasonic_Front_Data, HIGH, 10000);
  ultraSonicDistanceF = (echoTime/58);
  return (ultraSonicDistanceF);
}  

 double PingR1()
{
  int echoTime;
  digitalWrite(ci_Ultrasonic_Right1_Ping, HIGH);
  delayMicroseconds(10);  
  digitalWrite(ci_Ultrasonic_Right1_Ping, LOW);
  echoTime = pulseIn(ci_Ultrasonic_Right1_Data, HIGH, 10000);
  ultraSonicDistanceR1 = (echoTime/58);
  return (ultraSonicDistanceR1);
}
 double PingR2()
{
  int echoTime;
  digitalWrite(ci_Ultrasonic_Right2_Ping, HIGH);
  delayMicroseconds(10);  
  digitalWrite(ci_Ultrasonic_Right2_Ping, LOW);
  echoTime = pulseIn(ci_Ultrasonic_Right2_Data, HIGH, 10000);
  ultraSonicDistanceR2 = (echoTime/58);
  return (ultraSonicDistanceR2);
}

void loop() {
////Initializing the button clicks
//  if (digitalRead(ci_pushButton_2) == LOW)
//  {
//    button2Counter = 1;
//  }
//
//  if (digitalRead(ci_pushButton_3) == LOW)
//  {
//    button3Counter = 1;
//  }

if (phase ==0)
{
  Serial.println(PingR1());
  //Serial.println(PingF());
  servo_RightMotor.writeMicroseconds(1700);
  servo_LeftMotor.writeMicroseconds(1700);
  PingFreturn= PingF();
  
  if (PingFreturn<=25 && PingFreturn>1)
  {
        phase++;
  }
}

if (phase==1)
{
      servo_RightMotor.writeMicroseconds(1700);
      servo_LeftMotor.writeMicroseconds(1500);
      PingR1return = PingR1();
      if((PingR1return>=5 && PingR1return<=8))
      {
        phase++;
      } 
}
if(phase==2)
{
      servo_RightMotor.writeMicroseconds(1650);
      servo_LeftMotor.writeMicroseconds(1500);
      PingR1return = PingR1();
      PingR2return = PingR2();
      if(((PingR1return - PingR2return)<=3) && ((PingR1return - PingR2return)>=1))
      {
        phase++;
      } 
}
if (phase==3)
{
      servo_RightMotor.writeMicroseconds(1500);
      servo_LeftMotor.writeMicroseconds(1500);
}
}

