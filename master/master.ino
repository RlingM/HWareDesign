#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

const int front_TrgPin = 10;
const int front_EcoPin = 4;
const int left_TrgPin = 6;
const int left_EcoPin = 2;
const int right_TrgPin = 5;
const int right_EcoPin = 3;
float front_dist[2];
float left_dist[2];
float right_dist[2];

RF24 radio(7, 8);
const byte address[6] = "00001";
int info[2];

//前超声测距
float front_UlDisMea(){
  digitalWrite(front_TrgPin, LOW);
  delayMicroseconds(8);
  digitalWrite(front_TrgPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(front_TrgPin, LOW);
  return pulseIn(front_EcoPin, HIGH) / 58.00;
}

//左超声测距
float left_UlDisMea(){
  digitalWrite(left_TrgPin, LOW);
  delayMicroseconds(8);
  digitalWrite(left_TrgPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(left_TrgPin, LOW);
  return pulseIn(left_EcoPin, HIGH) / 58.00;
}

//右超声测距
float right_UlDisMea(){
  digitalWrite(right_TrgPin, LOW);
  delayMicroseconds(8);
  digitalWrite(right_TrgPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(right_TrgPin, LOW);
  return pulseIn(right_EcoPin, HIGH) / 58.00;
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
  info[1] = '\0';
  front_dist[1] = '\0';
  left_dist[1] = '\0';
  right_dist[1] = '\0';
  SPI.begin();
  Serial.begin(115200);
  radio.begin();
  radio.setChannel(90);
  radio.openWritingPipe(address);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  pinMode(front_TrgPin, OUTPUT);
  pinMode(front_EcoPin, INPUT);
  pinMode(left_TrgPin, OUTPUT);
  pinMode(left_EcoPin, INPUT);
  pinMode(right_TrgPin, OUTPUT);
  pinMode(right_EcoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  front_dist[0] = front_UlDisMea();
  left_dist[0] = left_UlDisMea();
  right_dist[0] = right_UlDisMea();

  //判断正在直行且前方有车或障碍物
  if(front_dist[0] <= 10){
    if(TheSameSpeed()){
      info[0] = 1;
      radio.write(&info, sizeof(info));
      
    }
  }

  //判断正在直行但角度有些不对，应右转少许
  if((left_dist[0] <= 10) && TheSameSpeed()){
    info[0] = 4;
    radio.write(&info, sizeof(info));
    
  }
  //判断正在直行但角度有些不对，应左转少许
  if((right_dist[0] <= 10) && TheSameSpeed()){
    info[0] = 5;
    radio.write(&info, sizeof(info));
  }


  //判断正在左转但是转弯幅度不对
  if((left_dist[0] <= 10) && RHigher()){
    info[0] = 2;
    radio.write(&info, sizeof(info));
    
  }

  //判断正在右转但是转弯幅度不对
  if((right_dist[0] <= 10) && LHigher()){
    info[0] = 3;
    radio.write(&info, sizeof(info)); 
    
  }

  //判断可以恢复原车速
  if(front_dist[0] <= 10){
    info[0] = 6;
    radio.write(&info, sizeof(info)); 
  }
  delay(20);
}
