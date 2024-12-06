#include "SoftwareSerial.h"
#include "NewPing.h"
#include <Servo.h>
#include "SimpleTimer.h"


#define Trigger_Pin 12
#define Echo_Pin 13
#define Max_Ultra_distance 400
#define Left_tra 7 // pin for left IR sensor pin
#define Right_tra A1 //pin for right IR sensor pin
#define Center_tra 8 //pin for ceneter IR sensor pin
#define Black_Line  1
#define stop_func 101 

String BT_receive;

int min_speed = 60;
int med_speed = 110;
int max_speed = 170;
int Emax_speed = 255;
int speed_car = 60;


int ultra_distance;

int claw_read;
int base_read;
int arm_read;
int claw_flag;
int base_flag;
int arm_flag;

SoftwareSerial BT(0,1);
NewPing sonar(Trigger_Pin, Echo_Pin, Max_Ultra_distance);

Servo clawservo;
Servo baseservo;
Servo armservo;

SimpleTimer servo_timer(10);
SimpleTimer US_timer(300); //Ultrasonic timer

int last_direction = 0; // 0 straight 1 left 2 right

void forward(int speed)
{
  digitalWrite(2,HIGH);
  analogWrite(5,speed); 
  digitalWrite(4,LOW);
  analogWrite(6,speed);
}
void backward(int speed)
{
  digitalWrite(2,LOW);
  analogWrite(5,speed);
  digitalWrite(4,HIGH);
  analogWrite(6,speed);
}
void left_direction(int speed)
{
  digitalWrite(2,LOW);
  analogWrite(5,speed);
  digitalWrite(4,LOW);
  analogWrite(6,speed);
}
void right_direction(int speed)
{
  digitalWrite(2,HIGH);
  analogWrite(5,speed);
  digitalWrite(4,HIGH);
  analogWrite(6,speed); 
}
void stop()
{
  digitalWrite(2,HIGH);
  analogWrite(5, 0); 
  digitalWrite(4,LOW);
  analogWrite(6, 0);
}

int ultrasonic_distance()
{
  int check_distance = sonar.ping_cm();
  return (check_distance);
}

void line_tracking()
{
  int Left, Center, Right;
  bool Line_tracking_flag = true;
  while (Line_tracking_flag) 
  {
    Left = digitalRead(Left_tra);
    Center = digitalRead(Center_tra);
    Right = digitalRead(Right_tra);
    if (Left != Black_Line && (Center == Black_Line && Right != Black_Line)) 
    {
      forward(60);
      last_direction = 0;
    } 
    else if (Left == Black_Line && (Center == Black_Line && Right != Black_Line)) 
    {
      left_direction(90);
      last_direction = 1;
    } 
    else if (Left == Black_Line && (Center != Black_Line && Right != Black_Line)) 
    {
      left_direction(80);
      last_direction = 1;
    } 
    else if (Left != Black_Line && (Center != Black_Line && Right == Black_Line)) 
    {
      right_direction(80);
      last_direction = 2;
    } 
    else if (Left != Black_Line && (Center == Black_Line && Right == Black_Line)) 
    {
      right_direction(80);
      last_direction = 2;
    } 
    else if (Left == Black_Line && (Center == Black_Line && Right == Black_Line)) 
    {
      stop();
    }
    else if (Left != Black_Line && (Center != Black_Line && Right != Black_Line)) 
    {
      stop();
      if (last_direction == 1)
      {
       left_direction(80);
       delay(200);
      }
      else if (last_direction == 2)
      {
        right_direction(80);
        delay(200);
      }
      else if (last_direction == 0)
      {
        backward(70);
        delay(300);
      }
    }
    if (BT.read() == stop_func) 
    {
      Line_tracking_flag = false;
      stop();
    }
  }
}

void initialise_servo()
{
  clawservo.write(90);
  baseservo.write(90);
  armservo.write(0);
  delay(1000);
}

void claw_close()
{
  claw_flag = true;
  claw_read = clawservo.read();
  while(claw_flag)
  {
    claw_read = constrain(claw_read, 90, 180); //limit angle to between 90 and 180 repeatly
    if (claw_read <= 180 && claw_read >= 90)
    {
      if(servo_timer.check())
      {
        clawservo.write(claw_read);
        claw_read = claw_read + 1;
      }
    }
    if (BT.available())
    {
      if (BT.read() == stop_func)
      {
        claw_flag = false;
        break;
      }
    }
  }
}

void claw_open()
{
  claw_flag = true;
  claw_read = clawservo.read();
  while(claw_flag)
  {
    claw_read = constrain(claw_read, 90, 180); //limit angle to between 90 and 180
    if (claw_read <= 180 && claw_read >= 90)
    {
      if(servo_timer.check())
      {
        clawservo.write(claw_read);
        claw_read = claw_read - 1;

      }
    }
    if (BT.available())
    {
      if (BT.read() == stop_func)
      {
        claw_flag = false;
        break;
      }
    }
  }
}

void anticlockwise_base()
{
  base_flag = true;
  base_read = baseservo.read();
  while(base_flag)
  {
    base_read = constrain(base_read, 0, 180);
    if (base_read <= 180 && base_read >= 0)
    {
      if(servo_timer.check())
      {
        baseservo.write(base_read);
        base_read = base_read + 1;
      }
    }
    if (BT.available())
    {
      if (BT.read() == stop_func)
      {
        claw_flag = false;
        break;
      }
    }
  }
}

void clockwise_base()
{
  base_flag = true;
  base_read = baseservo.read();
  while(base_flag)
  {
    base_read = constrain(base_read, 0, 180);
    if (base_read <= 180 && base_read >= 0)
    {
      if(servo_timer.check())
      {
        baseservo.write(base_read);
        base_read = base_read - 1;
      }
    }
    if (BT.available())
    {
      if (BT.read() == stop_func)
      {
        claw_flag = false;
        break;
      }
    }
  }
}

void arm_down()
{
  arm_flag = true;
  arm_read = armservo.read();
  while(arm_flag)
  {
    arm_read = constrain(arm_read, 0, 100);
    if (arm_read <= 100 && arm_read >= 0)
    {
      if (servo_timer.check())
      {
        armservo.write(arm_read);
        arm_read = arm_read + 1;
      }
    }
    if (BT.available())
    {
      if (BT.read() == stop_func)
      {
        claw_flag = false;
        break;
      }
    }
  }
}

void arm_up()
{
  arm_flag = true;
  arm_read = armservo.read();
  while(arm_flag)
  {
    arm_read = constrain(arm_read, 0, 100);
    if (arm_read <= 100 && arm_read >= 0)
    {
      if (servo_timer.check())
      {
        armservo.write(arm_read);
        arm_read = arm_read - 1;
      }
    }
    if (BT.available())
    {
      if (BT.read() == stop_func)
      {
        claw_flag = false;
        break;
      }
    }
  }
}

void setup() {
  BT.begin(9600);
  pinMode(2,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(Left_tra, INPUT);
  pinMode(Center_tra, INPUT);
  pinMode(Right_tra, INPUT);
  clawservo.attach(10);
  baseservo.attach(11);  
  armservo.attach(9);
  initialise_servo();
}


void loop() {
  if(BT.available())
  {
    BT.setTimeout(20);
    BT_receive = BT.readString();
    BT_receive.trim();
    char bt_receive = BT_receive[0];
    switch (bt_receive)
    {
      case 'f':
        forward(speed_car);
        break;
      case 'b':
        backward(speed_car);
        break;
      case 'l':
        left_direction(speed_car);
        break;
      case 'r':
        right_direction(speed_car);
        break;
      case 's':
        stop();
        break;
      case 'X':
        speed_car = min_speed;
        break;
      case 'Y':
        speed_car = med_speed;
        break;
      case 'Z':
        speed_car = max_speed;
        break;
      case 'E':
        speed_car = Emax_speed;
        break;
      case 'A':
        line_tracking();
        break;
      case 'o':
        claw_open();
        break;
      case 'c':
        claw_close();
        break;
      case 'L':
        anticlockwise_base();
        break;
      case 'R':
        clockwise_base();
        break;
      case 'u':
        arm_up();
        break;
      case 'd':
        arm_down();
        break;
      case 'I':
        initialise_servo();
        break;
      default:
        break;
    }
  }
  if (US_timer.check())
  {
    ultra_distance = ultrasonic_distance();
    if (ultra_distance > 0 && ultra_distance < Max_Ultra_distance)
    {
      BT.print(ultra_distance);
      BT.print("cm");
    }
    else
    {
      BT.print("NULL");
    }
  }
}