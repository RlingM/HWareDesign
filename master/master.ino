#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

//超声接口定义
const int front_TrgPin = 10;
const int front_EcoPin = 4;
const int left_TrgPin = 6;
const int left_EcoPin = 18;
const int right_TrgPin = 5;
const int right_EcoPin = 19;

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

void setup() {
  // put your setup code here, to run once:
  info[2] = '\0';
  SPI.begin();
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
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

  info[0] = 0;
  info[1] = front_UlDisMea();
  radio.write(&info, sizeof(info));
  info[0] = 1;
  info[1] = right_UlDisMea();
  radio.write(&info, sizeof(info));
    
  //后续可以考虑利用左超声做第三个模式，作用同第二个模式
  delay(4);
}
