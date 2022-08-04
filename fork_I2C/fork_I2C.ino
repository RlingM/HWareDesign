#include <Wire.h>
#include <TimerOne.h>

//遥控
const char Forward = 'f';
const char Backward = 'b';
const char Turn_left = 'l' ;
const char Turn_right = 'r' ;
const char Stop = 's';
const char Start = 'k';
#define SPEED_LEVEL_MAX 200  //遥控模式下电机最大速度，待改！！！
//#define SPEED_LEVEL_MIN 100   //遥控模式下电机最小速度，待改！！！

//电机接口定义
const int left_ENAandB = 6;
const int right_ENAandB = 5;

const int left_IN1_3 = 9;
const int left_IN2_4 = 4;
const int right_IN1_3 = 10;
const int right_IN2_4 = A0;

const int left_ENC_A = 2;
const int right_ENC_A = 3;

//通信
float order[3];

//PID
int left_count = 0;
int right_count = 0;
float left_err = 0, left_derr = 0, left_sumErr = 0;
float right_err = 0, right_derr = 0, right_sumErr = 0;
float Kp = 0.92 , Ki = 0.20, Kd = 0.01;
int left_Pwm;
int right_Pwm;

float left_rpm;
float right_rpm;

const float iniSpeed = 80;
float left_goal = iniSpeed;
float right_goal = iniSpeed;

//模式选择 'm = 1',遥控模式  'm = 0',自动模式
bool mode = 0;

//Control
float CA = -4.8 * pow(10, -4);
float Ca = -4.9 * pow(10, -5);
float Cb = CA * Ca * Ca / 4;
float deltaV;
float pre_dist;
//---------------------------------------------------------

//PID算法--左
float left_PID(float goalRpm, float nowRpm){
  left_derr = goalRpm - nowRpm - left_err;
  left_err = goalRpm - nowRpm;
  left_sumErr += left_err;
  float PWM = Ki * left_sumErr + Kp * left_err + Kd * left_derr;
  return PWM;
}

//PID算法--右
float right_PID(float goalRpm, float nowRpm){
  right_derr = goalRpm - nowRpm - right_err;
  right_err = goalRpm - nowRpm;
  right_sumErr += right_err;
  float PWM = Ki * right_sumErr + Kp * right_err + Kd * right_derr;
  return PWM;
}

//减速
void Deceleration(){
  float x[3];
  do{
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
    Wire.read(&x, sizeof(x));
    Serial.println(x[1]);
  }while((x[1] <= 25) && ((int)x[0] == 0));
}

//直线控制
float Control(float deltaDist){
  Cb = CA * Ca * Ca / 4;
  deltaV = Ca * (deltaDist - pre_dist) / 5 + Cb * (deltaDist - 15) / 100;
  pre_dist = deltaDist;
  return deltaV;
}

//左侧轮中断计数
void CodeL(){
  left_count++;
}

//右侧轮中断计数
void CodeR(){
  right_count++;
}

//左侧轮中断更新速度
void left_ISR(){
  left_rpm = left_count * 15.0 / 13.0;
  left_count = 0;
  left_Pwm = left_PID(left_goal, left_rpm);
  analogWrite(left_ENAandB, left_Pwm);
}

//右侧轮中断更新速度
void right_ISR(){
  right_rpm = right_count * 15.0 / 13.0;
  right_count = 0;
  right_Pwm = right_PID(right_goal, right_rpm);
  analogWrite(right_ENAandB, right_Pwm);
}

//外部中断
void Isr(){
  float tempX[3];
  Wire.read(&tempX, sizeof(tempX));
  float temp = Control(tempX[1]);
  left_goal += temp / 2;
  right_goal -= temp / 2;
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
  digitalWrite(left_IN1_3, HIGH);
  digitalWrite(left_IN2_4, LOW);
  left_Pwm = -left_Pwm;
  digitalWrite(right_IN1_3, HIGH);
  digitalWrite(right_IN2_4, LOW);
  right_Pwm = -right_Pwm;
}

void motorChange(float x[])
{
  switch((int)x[0]){
    case 0:
    //首先尝试原地左转一点能否继续前进，即绕过去
      right_goal = -right_goal;
      digitalWrite(right_IN1_3, LOW);
      digitalWrite(right_IN2_4, HIGH);
      right_Pwm = -right_Pwm;
      delay(2000);

      //2秒尝试失败后，开始减速
      right_goal = -right_goal;
      digitalWrite(right_IN1_3, HIGH);
      digitalWrite(right_IN2_4, LOW);
      right_Pwm = -right_Pwm;
      Deceleration();

      //减速成功后，恢复原速
      left_goal = iniSpeed;
      right_goal = iniSpeed;
      break;

    default:
      break;
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
        mode = 0;
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
  right_goal = 80;
  left_goal = 80;
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
      //Serial.println("Not the right command!");
      ;
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
      //Back();
      break;

    case Turn_left:
      left_goal = -60;
      right_goal = 60;
      Proceed();
      delay(100);//待调
      motorStop();
      break;
    
    
    case Turn_right:
      left_goal = 60;
      right_goal = -60;//待调
      Proceed();
      delay(100);//待调
      motorStop();
      break;

    case Stop:
      motorStop();
      break;

    default:
      //Serial.print("Wrong command!");
      //Serial.println("Please input true command again.");
      ;
  }
}

void motorStop()
{
  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, LOW);
  digitalWrite(right_IN1_3, LOW);
  digitalWrite(right_IN2_4, LOW);
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

void setup(){
  pinMode(left_IN1_3, OUTPUT);
  pinMode(left_IN2_4, OUTPUT);
  pinMode(right_IN1_3, OUTPUT);
  pinMode(right_IN2_4, OUTPUT);
  
  pinMode(left_ENC_A, INPUT);
  pinMode(right_ENC_A, INPUT);

  digitalWrite(left_IN1_3, HIGH);
  digitalWrite(left_IN2_4, LOW);
  digitalWrite(right_IN1_3, HIGH);
  digitalWrite(right_IN2_4, LOW);
  
  Timer1.initialize(50000);
  Timer1.attachInterrupt(Isr);
  attachInterrupt(0, CodeL, FALLING);
  attachInterrupt(1, CodeR, FALLING);
  //Serial.println("All ready");
  //Serial.println("Please choose mode.");
  //Serial.println("The default mode is Remote Control mode.");
  Serial.begin(9600);
  Wire.begin(44);
  Wire.onReceive(receiveEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Wire.available()){
    Wire.read(&order, sizeof(order));
    motorChange(order);
  }
  /*while(Serial.available() > 0){
    char Cmd = Serial.read();
    delay(1);
    if(mode == 1)
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
  }*/
  delay(2);
}
