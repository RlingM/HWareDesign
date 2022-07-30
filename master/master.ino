#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

//超声接口定义
const int front_TrgPin = 10;
const int front_EcoPin = 4;
const int left_TrgPin = 6;
const int left_EcoPin = 2;
const int right_TrgPin = 5;
const int right_EcoPin = 3;

RF24 radio(7, 8);
const byte address[6] = "00001";
float info[3];

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

void ISR(){
  
}

void setup() {
  // put your setup code here, to run once:
  info[2] = '\0';
  SPI.begin();
  Serial.begin(115200);
  radio.begin();
  radio.setChannel(90);
  radio.openWritingPipe(address);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
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
  
  //判断小车前方有车或障碍物
  if(front_UlDisMea() <= 10){
    info[0] = 0;
    info[1] = front_UlDisMea();
    radio.write(&info, sizeof(info));
      
  }
  
  //判断小车偏离中心，这里隐含规定了中心值为15
  else if((right_UlDisMea() < 14) || (right_UlDisMea() > 16)){
    info[0] = 1;
    info[1] = right_UlDisMea() - 15;
    radio.write(&info, sizeof(info)); 
    
  }

  delay(20);
}
