#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#include <TimerOne.h>

//遥控
const char Forward = 'F';
const char Backward = 'B';
const char TurnlLft = 'L' ;
const char TurnRight = 'R' ;
const char Stop = 'S';
const char ModeChange = 'C';
const char SpeedUp = 'U';
const char SpeedDown = 'D';
const char Start = 'I';

//电机接口定义
const int left_ENAandB = 6;
const int right_ENAandB = 5;

const int left_IN1_3 = 10;
const int left_IN2_4 = 14;
const int right_IN1_3 = 9;
const int right_IN2_4 = 4;

const int left_ENC_A = 2;
const int right_ENC_A = 3;

//通信
RF24 radio(7,8);
const byte address[6] = "00001";
float orderForF[3];
float orderForR[3];
bool NeedControl = true;
bool notStop = true;

//直线控制
float err = 0, derr = 0, dderr = 0;
float Skp = 1.0, Ski = 0, Skd = 0;
float goal_dist = 15;

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

const float iniSpeed = 150;
const float MaxSpeed = 200;
float left_goal = iniSpeed;
float right_goal = iniSpeed;

//模式选择，mode = true为遥控，mode = false为自动
char Cmd;
bool mode = true;

//读取1/3引脚的电平，高则返回1，低则返回0
int staFor1_3;

/*//Control
float CA = -0.048;
float Ca = -41.7;
float Cb = CA * Ca * Ca / 4.0;   //Cb = -20.9
float deltaV;
float pre_dist;*/
//---------------------------------------------------------

//互换左侧电机高低电平
void LchangeStatus(){
  left_goal = -left_goal;
  staFor1_3 = digitalRead(left_IN1_3);
  if(staFor1_3 == 1){
    digitalWrite(left_IN1_3, LOW);
    digitalWrite(left_IN2_4, HIGH);
  }
  else{
    digitalWrite(left_IN1_3, HIGH);
    digitalWrite(left_IN2_4, LOW);
  }
}

//互换右侧电机高低电平
void RchangeStatus(){
  right_goal = -right_goal;
  staFor1_3 = digitalRead(right_IN1_3);
  if(staFor1_3 == 1){
    digitalWrite(right_IN1_3, LOW);
    digitalWrite(right_IN2_4, HIGH);
  }
  else{
    digitalWrite(right_IN1_3, HIGH);
    digitalWrite(right_IN2_4, LOW);
  }
}

//直线控制
float Straight(float goalDist, float nowDist){
  dderr = goalDist - nowDist - err - derr;
  derr = goalDist - nowDist - err;
  err = goalDist - nowDist;
  float dere = Ski * err + Skp * derr + Skd * dderr;
  return dere;
}

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
    }
    else if((right_goal >= 0) && (left_goal >= 0)){
      left_goal -= 20;
      right_goal -= 20;
      delay(200);
    }
    
    if(left_goal < 0){
      left_goal = 0;
    }
    if(right_goal < 0){
      right_goal = 0;
    }
    
    do{
      radio.read(&x, sizeof(x));
    }while((int)x[0] != 0);
  }while(x[1] <= 30);
}

/*//直线控制
float Control(float deltaDist){
  Cb = CA * Ca * Ca / 4.0;
  deltaV = Ca * (deltaDist - pre_dist) * 3 / 50 + Cb * (deltaDist - 15) * 3 / 5.0;
  pre_dist = deltaDist;
  return deltaV;
}*/

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
  left_rpm = left_count * 30.0 / 13.0;
  left_count = 0;
  left_Pwm = left_PID(left_goal, left_rpm);
  if(left_Pwm < 0){
    left_Pwm = 0;
  }
  analogWrite(left_ENAandB, left_Pwm);
}

//右侧轮中断更新速度
void right_ISR(){
  right_rpm = right_count * 30.0 / 13.0;
  right_count = 0;
  right_Pwm = right_PID(right_goal, right_rpm);
  if(right_Pwm < 0){
    right_Pwm = 0;
  }
  analogWrite(right_ENAandB, right_Pwm);
}

//外部中断
void Isr(){
  if(NeedControl){    
    //float temp = Control(orderForR[1]);
    float temp = Straight(goal_dist, orderForR[1]);
    left_goal -= temp / 2;
    right_goal += temp / 2;
    
    if(left_goal > MaxSpeed){
      left_goal = MaxSpeed;
      digitalWrite(left_IN1_3, HIGH);
      digitalWrite(left_IN2_4, LOW);
    }
    if(right_goal > MaxSpeed){
      right_goal = MaxSpeed;
      digitalWrite(right_IN1_3, HIGH);
      digitalWrite(right_IN2_4, LOW);
    }
    if(left_goal < 0){
      LchangeStatus();
      if(left_goal > MaxSpeed){
        left_goal = MaxSpeed;
      }
    }
    if(right_goal < 0){
      RchangeStatus();
      if(right_goal > MaxSpeed){
        right_goal = MaxSpeed;
      }
    }
  }
  
  left_ISR();
  right_ISR();
}

//恢复前行
void Proceed(){
  left_goal = iniSpeed;
  right_goal = iniSpeed;
  digitalWrite(left_IN1_3, HIGH);
  digitalWrite(left_IN2_4, LOW);
  digitalWrite(right_IN1_3, HIGH);
  digitalWrite(right_IN2_4, LOW);
}

//后退
void Back(){
  left_goal = iniSpeed;
  right_goal = iniSpeed;
  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, HIGH);
  digitalWrite(right_IN1_3, LOW);
  digitalWrite(right_IN2_4, HIGH);
}

//处理前方道路情况
void SolutionF(float x){
  //首先尝试原地左转一点能否继续前进，即绕过去
  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, HIGH);
  delay(1500);
  
  float temp[3];
  radio.read(&temp, sizeof(temp));
  while(temp[0] != 0){
    radio.read(&temp, sizeof(temp));
  }

  digitalWrite(left_IN1_3, HIGH);
  digitalWrite(left_IN2_4, LOW);
  
  //2秒尝试失败后，开始减速，尝试成功，则直接结束
  if(temp[1] < 30){
    Deceleration();

    //减速成功后，恢复原速
    left_goal = iniSpeed;
    right_goal = iniSpeed;
  }
}

//遥控模式
void RemoteControlMode(char cmd){
  switch(cmd){
    //电机启动
    case 'I':
      Begin();
      break;
      
    //恢复前进
    case 'F':
      Proceed();
      break;
      
    //后退
    case 'B':
      Back();
      break;
      
    //左转
    case 'L':
      digitalWrite(right_IN1_3, HIGH);
      digitalWrite(right_IN2_4, LOW);
      digitalWrite(left_IN1_3, LOW);
      digitalWrite(left_IN2_4, HIGH);
      left_goal = 60;
      right_goal = 60;
      break;

    //右转
    case 'R':
      digitalWrite(right_IN1_3, LOW);
      digitalWrite(right_IN2_4, HIGH);
      digitalWrite(left_IN1_3, HIGH);
      digitalWrite(left_IN2_4, LOW);
      left_goal = 60;
      right_goal = 60;
      break;
      
    //停车
    case 'S':
      motorStop();
      break;

    //切换到自动模式 
    case 'C':
      //Serial.print("To Auto mode");
      mode = false;
      break;
      
    //逐步加速
    case 'U':
      if(left_goal > MaxSpeed - 10){
        left_goal = MaxSpeed;
        //Serial.print("LMax Speed");
      }
      else if(left_goal < MaxSpeed){
        left_goal += 10;
      }
      
      if(right_goal > MaxSpeed - 10){
        right_goal = MaxSpeed;
        //Serial.print("RMax Speed");
      }
      else if(right_goal < MaxSpeed){
        right_goal += 10;
      }
      break;

    //逐步减速
    case 'D':
      if(right_goal < 10){
        right_goal = 0;
        //Serial.print("RStop");
      }
      else if(right_goal > 0){
        right_goal -= 10;
      }
      
      if(left_goal < 10){
        left_goal = 0;
        //Serial.print("LStop");
      }
      else if(left_goal > 0){
        left_goal -= 10;
      }
      break;

    default:
      //Serial.print("Wrong");
      ;
  }
}

//自动模式
void AutoMode(char cmd){
  switch(cmd){
    //电机启动
    case 'I':
      Begin();
      notStop = true;
      break;

    //切换到遥控  
    case 'C':
      mode = true;
      //Serial.print("To Remote mode");
      break;

    //停车
    case 'S':
      motorStop();
      notStop = false;
      //Serial.print("Stop");
      break;

    //恢复前进
    case 'F':
      Proceed();
      break;

    case 'B':
      Back();
      break;

    //左转
    case 'L':
      digitalWrite(right_IN1_3, HIGH);
      digitalWrite(right_IN2_4, LOW);
      digitalWrite(left_IN1_3, LOW);
      digitalWrite(left_IN2_4, HIGH);
      left_goal = 80;
      right_goal = 80;
      break;

    //右转
    case 'R':
      digitalWrite(right_IN1_3, LOW);
      digitalWrite(right_IN2_4, HIGH);
      digitalWrite(left_IN1_3, HIGH);
      digitalWrite(left_IN2_4, LOW);
      left_goal = 80;
      right_goal = 80;
      
    default:
      //Serial.print("Wrong");
      ;
  }
}

//电机开始转动
void Begin(){
  digitalWrite(left_IN1_3, HIGH);
  digitalWrite(left_IN2_4, LOW);
  digitalWrite(right_IN1_3, HIGH);
  digitalWrite(right_IN2_4, LOW);
}

//电机停止转动
void motorStop(){
  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, LOW);
  digitalWrite(right_IN1_3, LOW);
  digitalWrite(right_IN2_4, LOW);
}

void setup(){
  SPI.begin();
  Serial.begin(9600);
  radio.begin();
  pinMode(left_IN1_3, OUTPUT);
  pinMode(left_IN2_4, OUTPUT);
  pinMode(right_IN1_3, OUTPUT);
  pinMode(right_IN2_4, OUTPUT);
  
  pinMode(left_ENC_A, INPUT);
  pinMode(right_ENC_A, INPUT);

  pinMode(left_ENAandB, OUTPUT);
  pinMode(right_ENAandB, OUTPUT);
  
  digitalWrite(left_IN1_3, LOW);
  digitalWrite(left_IN2_4, LOW);
  digitalWrite(right_IN1_3, LOW);
  digitalWrite(right_IN2_4, LOW);
  
  Timer1.initialize(100000);
  Timer1.attachInterrupt(Isr);
  attachInterrupt(0, CodeL, FALLING);
  attachInterrupt(1, CodeR, FALLING);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN); 
  radio.startListening();
  while(Serial.read() >= 0){};
}

void loop() {
  if(millis() % 100 == 0){
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
  
  if(NeedControl){
    if(notStop){
      if(radio.available()){
        delay(10);
        float temp[3];
        radio.read(&temp, sizeof(temp));
        if(temp[0] == 0){
          orderForF[0] = temp[0];
          orderForF[1] = temp[1];
          orderForF[2] = temp[2];
          if(orderForF[1] < 30){
            SolutionF(orderForF[1]);
          }
        }
        if(temp[0] == 1){
          orderForR[0] = temp[0];
          orderForR[1] = temp[1];
          orderForR[2] = temp[2];
        }
      }
    }
    else{
      radio.flush_tx();
      radio.flush_rx();
    }
  }
  
  if(Serial.available()){
    delay(10);
    Cmd = Serial.read();
    radio.flush_tx();
    radio.flush_rx();
    //遥控模式
    if(mode){
      //Serial.print("Remote mode");
      radio.stopListening();
      NeedControl = false;
      RemoteControlMode(Cmd);
    }
    
    //自动模式
    else{
      //Serial.print("Auto mode");
      radio.startListening();
      NeedControl = true;
      AutoMode(Cmd);
    }
  }
  delay(2);
}
