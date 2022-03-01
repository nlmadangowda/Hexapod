/**
 * @file main.cpp
 * @author nlmadangowda (madannl17@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "M5Atom.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_PWMServoDriver.h"

#define MAX_TOLL    10
#define TRIG_PIN    19
#define ECHO_PIN    22
// This is the 'minimum' pulse length count (out of 4096)
#define SERVOMIN    150 

// This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAX    600 

// This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMIN       600 

// This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define USMAX       2400 

// Analog servos run at ~50 Hz updates
#define SERVO_FREQ  50 


#define J0_INIT 90
#define J1_INIT 65
#define J2_INIT 115


#define GET_BIT_STATUS(_value_,_pos_)   ((_value_ & (1<<_pos_)) >> _pos_)

enum _hand_dir_{
  DIR_FRONT = 0,
  DIR_BACK,
  DIR_LEFT,
  DIR_RIGHT
};

typedef struct _joints_{
  uint8_t pin;
  uint8_t pos;
  int8_t error;
}Joint;

typedef struct _legs_{
  uint8_t driver;
  Joint   joints[3];
}Leg;

Leg legs[6];

uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xCC, 0x85, 0x08};
const uint32_t g_disp_dir[]={0xFFFFFFFF,0x004255c4,0x00475484,0x00417c44,0x00447D04,0x01151151};
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
float duration_us, distance_cm;


/**
 * @brief 
 * 
 * @param p_dir 
 */
void SendSigToPod(uint8_t p_dir);

/**
 * @brief 
 * 
 * @param mac_addr 
 * @param status 
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

/**
 * @brief 
 * 
 * @param p_dir 
 */
void PrintPattren(uint8_t p_dir);

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void ESPNow_Init(){
 if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void UltraSonicInit(){
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void InitServoLegs(uint8_t p_number,uint8_t p_joint, uint8_t p_pin, uint8_t p_pos, uint8_t p_driver, int8_t p_error){
  p_number-=1;
  legs[p_number].driver = p_driver;
  legs[p_number].joints[p_joint].pin = p_pin;
  legs[p_number].joints[p_joint].pos = p_pos;
  legs[p_number].joints[p_joint].error = p_error;
}

void MvLeg(Leg p_s_leg){
  int val = 0;
  for (int i = 0 ; i < 3; i ++) {
  val = map((p_s_leg.joints[i].pos+(p_s_leg.joints[i].error)),0,180,SERVOMIN,SERVOMAX);
    if(p_s_leg.driver==0){
      pwm.setPWM(p_s_leg.joints[i].pin, 0, val);
    }else if(p_s_leg.driver==1){
      pwm1.setPWM(p_s_leg.joints[i].pin, 0, val);
    }else{
      /*NOP*/
    }
    delay(10);
  } 
}


void InitLegs(){
  InitServoLegs(1,0,0,J0_INIT,0,0);
  InitServoLegs(1,1,1,J1_INIT,0,0);
  InitServoLegs(1,2,2,J2_INIT,0,0);

  InitServoLegs(2,0,4,J0_INIT,0,0);
  InitServoLegs(2,1,5,J1_INIT,0,0);
  InitServoLegs(2,2,6,J2_INIT,0,0);

  InitServoLegs(3,0,8,J0_INIT,0,0);
  InitServoLegs(3,1,9,J1_INIT,0,0);
  InitServoLegs(3,2,10,J2_INIT,0,0);

  InitServoLegs(4,0,0,J0_INIT,1,0);
  InitServoLegs(4,1,1,J1_INIT,1,0);
  InitServoLegs(4,2,2,J2_INIT,1,5);

  InitServoLegs(5,0,4,J0_INIT,1,0);
  InitServoLegs(5,1,5,J1_INIT,1,0);
  InitServoLegs(5,2,6,J2_INIT,1,0);

  InitServoLegs(6,0,8,J0_INIT,1,0);
  InitServoLegs(6,1,9,J1_INIT,1,5);
  InitServoLegs(6,2,10,J2_INIT,1,0);

  for (int i = 0 ; i < 6; i ++) {
    MvLeg(legs[i]);
  }
}

void ServoInit(){
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);
  
  InitLegs();  
  delay(2000);
}
void setup(){
  M5.begin(true, true, true);
  // M5.IMU.Init();
  // WiFi.mode(WIFI_STA);
  // ESPNow_Init();
  // UltraSonicInit();
  ServoInit();
}

void PrintPattren(uint8_t p_dir){
  for(int i = 0 ; i < 25; i++){
    if(GET_BIT_STATUS(g_disp_dir[p_dir],i)){
      M5.dis.drawpix(i,0xf00000);
    }else{
      M5.dis.drawpix(i,0x00000);
    }
  }
}

void UltraSoincReading(){
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);
    // calculate the distance
    distance_cm = 0.017 * duration_us;
    Serial.print("distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
  
}


void SetLegPos(uint8_t p_number, uint8_t p_j0,uint8_t p_j1,uint8_t p_j2){
  p_number-=1;
  legs[p_number].joints[0].pos = p_j0; 
  legs[p_number].joints[1].pos = p_j1; 
  legs[p_number].joints[2].pos = p_j2; 
}

void MovFW(){

  SetLegPos(1,120,50,115);
  SetLegPos(3,120,50,115);
  SetLegPos(5,60,50,115);
  for (int i = 0 ; i < 6; i ++) {
    MvLeg(legs[i]);
  }
  delay(500); 
  SetLegPos(2,60,65,115);
  SetLegPos(4,120,65,115);
  SetLegPos(6,120,65,115);
  for (int i = 0 ; i < 6; i ++) {
    MvLeg(legs[i]);
  }
  delay(500); 
  SetLegPos(1,60,65,115);
  SetLegPos(3,60,65,115);
  SetLegPos(5,120,65,115);
  for (int i = 0 ; i < 6; i ++) {
    MvLeg(legs[i]);
  }
  delay(500); 
  SetLegPos(2,120,50,115);
  SetLegPos(4,60,50,115);
  SetLegPos(6,60,50,115);
  
  for (int i = 0 ; i < 6; i ++) {
    MvLeg(legs[i]);
    // delay(10);
  }
  delay(500); 
}

void loop(){
    MovFW();
}