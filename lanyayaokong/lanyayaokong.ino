//计划初始设置为自动模式，开始选择模式，通过模式切换指令（m）切换模式，通过速度切换指令（a，j）切换速度
//其中在自动模式下输入任意遥控模式指令（f,b,l,r,s,k）切换到遥控模式，通过遥控指令完成动作
//其中在输入s指令后再输入k指令，重新开始自动模式
//初始化设置----------------------------------------------
#include <MsTimer2.h>
#include "HardwareSerial.h"
const char Forward = 'f';
const char Backward = 'b';
const char Turn_left = 'l' ;
const char Turn_right = 'r' ;
const char Stop = 's';
const char Start = 'k';
#define SPEED_LEVEL_MAX 200  //遥控模式下电机最大速度，待改！！！
//#define SPEED_LEVEL_MIN 100   //遥控模式下电机最小速度，待改！！！

//电机接口定义
const int left_ENA = 9;
const int right_ENB = 10;

const int left_IN1_3 = 13;
const int left_IN2_4 = 8;
const int right_IN1_3 = 13;
const int right_IN2_4 = 8;

const int left_ENC_A = 2;
const int right_ENC_A = 3;

//超声接口定义
const int TrgPin = 11;
const int left_EcoPin = 4;
const int right_EcoPin = 7;
const int front_EcoPin = 12;
float front_dist;
float left_dist;
float right_dist;

//PID
int left_count = 0;
int right_count = 0;
float err = 0, derr = 0, sumErr = 0;
float Kp = 0.9 , Ki = 0.04, Kd = 0.1;//待调
int left_Pwm;
int right_Pwm;

float left_rpm;
float right_rpm;

float left_goal = 100;//待调
float right_goal = 100;//待调

bool mode = 0;//模式选择 'm=1',遥控模式  'm=0',自动模式
//---------------------------------------------------------

//PID算法
float PID(float goalRpm, float nowRpm){
  derr = goalRpm - nowRpm - err;
  err = goalRpm - nowRpm;
  sumErr += err;
  float PWM = Ki * sumErr + Kp * err + Kd * derr;
  return PWM;
}

//左侧轮中断计数
void CodeL(){
  left_count++;
}

//右侧轮中断计数
void CodeR(){
  right_count++;
}

//前超声测距
float front_UlDisMea(){
  digitalWrite(TrgPin, LOW);
  delayMicroseconds(8);
  digitalWrite(TrgPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrgPin, LOW);
  return pulseIn(front_EcoPin, HIGH) / 58.00;
}

//左超声测距
float left_UlDisMea(){
  digitalWrite(TrgPin, LOW);
  delayMicroseconds(8);
  digitalWrite(TrgPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrgPin, LOW);
  return pulseIn(left_EcoPin, HIGH) / 58.00;
}

//右超声测距
float right_UlDisMea(){
  digitalWrite(TrgPin, LOW);
  delayMicroseconds(8);
  digitalWrite(TrgPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrgPin, LOW);
  return pulseIn(left_EcoPin, HIGH) / 58.00;
}

//左侧轮中断更新速度
void left_ISR(){
  left_rpm = left_count * 15.0 / 13.0;
  left_count = 0;
  left_Pwm = PID(left_goal, left_rpm);
  analogWrite(left_ENA, left_Pwm);
}

//右侧轮中断更新速度
void right_ISR(){
  right_rpm = right_count * 15.0 / 13.0;
  right_count = 0;
  right_Pwm = PID(right_goal, right_rpm);
  analogWrite(right_ENB, right_Pwm);
}

//外部中断
void Isr(){
  left_ISR();
  right_ISR();
  Serial.print("LeftGoal:");
  Serial.print(left_goal);
  Serial.print(",");
  Serial.print("RightGoal:");
  Serial.print(right_goal);
  Serial.print(",");
  Serial.print("LeftRpm:");
  Serial.print(left_rpm);
  Serial.print(",");
  Serial.print("RightRpm:");
  Serial.println(right_rpm);
}

//恢复前行
void Proceed(){
  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, HIGH);
  left_Pwm = -left_Pwm;
  digitalWrite(right_IN1_3, HIGH);
  digitalWrite(right_IN2_4, LOW);
  right_Pwm = -right_Pwm;
}

//倒车
void Back(){
  digitalWrite(left_IN1_3, HIGH);
  digitalWrite(left_IN2_4, LOW);
  left_Pwm = -left_Pwm;
  digitalWrite(right_IN1_3, LOW);
  digitalWrite(right_IN2_4, HIGH);
  right_Pwm = -right_Pwm;
  while(front_dist < 5){}
}

//减速
void Deceleration(){
  while(front_dist < 5){
    front_dist = front_UlDisMea();
    left_dist = left_UlDisMea();
    right_dist = right_UlDisMea();
    if((right_goal - 20 <= 0) && (left_goal - 20 <= 0)){
      left_goal = 0;
      right_goal = 0;
      delay(500);
    }
    else if((right_goal >= 0) && (left_goal >= 0)){
      left_goal -= 20;
      right_goal -= 20;
      delay(500);
    }
  }
}

//判断是否同速
bool TheSameSpeed(){
  return (abs(left_rpm) < abs(right_rpm) + 5.0);
}

//判断是否右侧轮转速更高
bool RHigher(){
  return (abs(right_rpm) - abs(left_rpm) > 20.0);
}

//判断是否左侧轮转速更高
bool LHigher(){
  return (abs(left_rpm) - abs(right_rpm) > 20.0);
}

void setup() {

  // put your setup code here, to run once:
  pinMode(TrgPin, OUTPUT);
  pinMode(left_EcoPin, INPUT);
  pinMode(right_EcoPin, INPUT);
  pinMode(front_EcoPin, INPUT);
  
  pinMode(left_IN1_3, OUTPUT);
  pinMode(left_IN2_4, OUTPUT);
  pinMode(right_IN1_3, OUTPUT);
  pinMode(right_IN2_4, OUTPUT);
  
  pinMode(left_ENC_A, INPUT);
  pinMode(right_ENC_A, INPUT);

  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, HIGH);
  digitalWrite(right_IN1_3, HIGH);
  digitalWrite(right_IN2_4, LOW);
  
  MsTimer2::set(50, Isr);
  MsTimer2::start();
  attachInterrupt(0, CodeL, FALLING);
  attachInterrupt(1, CodeR, FALLING);
  Serial.begin(9600);
  //Serial.println("all ready");
  //Serial.println("Please chose mode.");
  //Serial.println("The default mode is Remote Control mode.");
}

void loop() {
  while(Serial.available()>0)
  {
    char Cmd = Serial.read();
    delay(1);
    if(mode==1)
    {
      //Serial.print("The Remote Control mode: ");
      //Serial.println(Cmd);
      RemoteControlMode(Cmd);//遥控模式
    }
    else
    {
      //Serial.print("The Auto mode: ");
      //Serial.println(Cmd);
      AutoMode(Cmd);//自动模式
    }
  }
  
  front_dist = front_UlDisMea();
  left_dist = left_UlDisMea();
  right_dist = right_UlDisMea();
  
  //遇到障碍物
  if(front_dist < 5){
    //判定正在直行，前方有车，减速
    if(TheSameSpeed()){
    //待调
      Deceleration();
      left_goal = 100;
      right_goal = 100;
    }
    //判定正在右转，但是转弯幅度不合适
    else if(LHigher()){
      Back();
      left_goal = 100;
      right_goal = 100;
      Proceed();
    }
    //判定正在左转，但是转弯幅度不合适
    else if(RHigher()){
      Back();
      left_goal = 100;
      right_goal = 100;
      Proceed();
    }
  }
}

void RemoteControlMode(char Cmd)//遥控模式
{

    if(Cmd =='f' || Cmd == 'b' || Cmd == 'l' || Cmd == 'r')
    {
      motorRun(Cmd, left_goal, right_goal);
    }
    else{
      switch(Cmd)
      {
        case 'm':   //输入'm'切换模式
        //Serial.println("NOW, change to the Autonomic mode.");
        mode=0;
        break;

        case 'a':    //输入'a'增加速度
        if((left_goal < SPEED_LEVEL_MAX) && (right_goal < SPEED_LEVEL_MAX))
        {
          right_goal += 20;
          left_goal += 20;//增幅待改！！！
          //Serial.print("The current speed: ");
        }
        else if((left_goal == SPEED_LEVEL_MAX) && (left_goal == SPEED_LEVEL_MAX))
        {
          //Serial.print("Already the maximum speed! ");
          //Serial.print("The current speed: ");
        }
        else
        {
          //Serial.println("Something wrong(From the speed.)");
        }
        break;
        
        case 'j':
        if(abs(right_goal) > 0 && abs(left_goal) > 0)
        {
          left_goal -= 20;//增幅待改！！！
          right_goal -= 20;
          //Serial.print("The current speed: ");
        }
        else if(right_goal == 0 && left_goal == 0/*SPEED_LEVEL == SPEED_LEVEL_MIN*/)
        {
          //Serial.println("Already the minimum speed! ");
          //Serial.print("The current speed: ");
        }
        else
        {
          //Serial.println("Something wrong(From the speed).");
        }
        break;

        case 's':
        motorRun(Cmd, left_goal, right_goal);
        //Serial.println("Stop run.");
        break;

        case 'k':
        if(left_goal == 0 && right_goal == 0)
        {
          //Serial.println("Start run.");
          mode = 0;
          break;
        }
        //else Serial.println("Already start!");

        default:
          //Serial.println("Not the right command!");
          ;
      }
    }
}

void AutoMode(char Cmd){
  right_goal = 100;
  left_goal = 100;//初始速度待改！！！
  switch(Cmd)
  {
    case 'm':
      mode = 1;
      //Serial.println("Now, change to the Remote Control mode.");
    break;
    
    case 's':
      motorRun(Cmd, left_goal, right_goal);
      //Serial.println("Stop run.");
    break;

    case 'k':
      if(left_Pwm == 0 && right_Pwm == 0)
      {
        //Serial.println("Start run.");
        mode = 0;
        break;
      }
      //else Serial.println("Already start!");

    case 'f':
      mode = 1;
      //Serial.println("Now, change to the Remote Control mode.");
      motorRun('f', left_goal, right_goal);
      break;

    case 'b':
      mode = 1;
      //Serial.println("Now, change to the Remote Control mode.");
      motorRun('b', left_goal, right_goal);
      break;

    case 'l':
      mode = 1;
      //Serial.println("Now, change to the Remote Control mode.");
      motorRun('l', left_goal, right_goal);
      break;

    case 'r':
      mode = 1;
      //Serial.println("Now, change to the Remote Control mode.");
      motorRun('r', left_goal, right_goal);
      break;

    default:
    ;
      //Serial.println("Not the right command!");
  }
}


void motorRun(char Cmd, uint8_t left_goal, uint8_t right_goal){
  switch(Cmd){
    case Forward:
    left_goal = left_goal;
    right_goal = right_goal;
    Proceed();
    break;
    
    case Backward:
    left_goal = left_goal;
    right_goal = right_goal;
    Back();
    break;

    case Turn_left:
    left_goal=0;
    right_goal=100;//待调
    Proceed();
    delay(100);//待调
    motorStop();
    break;
    
    
    case Turn_right:
    left_goal=100;right_goal=0;//待调
    Proceed();
    delay(100);//待调
    motorStop();
    break;

    case Stop:
    motorStop();
    break;

    default:
    ;
    //Serial.print("Wrong command!");
    //Serial.println("Please input true command again.");
  }
}

void motorStop()
{
  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, LOW);
  digitalWrite(right_IN1_3, LOW);
  digitalWrite(right_IN2_4, LOW);
}
