
#include "M5Atom.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_PWMServoDriver.h"
/************************************************************************************************************/

#define MAX_TOLL    10
#define TRIG_PIN    19
#define ECHO_PIN    22
#define SERVOMIN    150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX    600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN       600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX       2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ  50 // Analog servos run at ~50 Hz updates
#define GET_BIT_STATUS(_value_,_pos_)   ((_value_ & (1<<_pos_)) >> _pos_)
/************************************************************************************************************/

enum _hand_dir_{
  DIR_FRONT = 0,
  DIR_BACK,
  DIR_LEFT,
  DIR_RIGHT
};
typedef struct _leg_ Leg;
struct _leg_{
    uint8_t number;
    uint8_t joint;
    uint8_t addrs;
    uint8_t pin;
    uint8_t pos;
};
/************************************************************************************************************/
Leg legs[18];
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
float duration_us, distance_cm;

uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xCC, 0x85, 0x08};
const uint32_t g_disp_dir[]={0xFFFFFFFF,0x004255c4,0x00475484,0x00417c44,0x00447D04,0x01151151};

int g_move[][6]={{135,45,135,135,45,135},{45,135,45,45,135,45}}; // movement angle for first joint from the body
int leg_1_pos[3]={90,90,90};
int leg_1_move[2][3]={{135,20,90},{45,50,130}};
int leg_2_move[2][3]={{135,20,90},{45,50,90}};
/************************************************************************************************************/

void SendSigToPod(uint8_t p_dir);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void PrintPattren(uint8_t p_dir);
/************************************************************************************************************/

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

void InitLeg(Leg *p_leg, uint8_t p_number,uint8_t p_joint,uint8_t p_addrs,uint8_t p_pin,uint8_t p_pos){
    p_leg->number = p_number;
    p_leg->joint = p_joint;
    p_leg->addrs = p_addrs;
    p_leg->pin = p_pin;
    p_leg->pos = p_pos;
}

void ServoInit(){
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  int init_pos_val = 90;
  init_pos_val = map(init_pos_val,0,180,SERVOMIN,SERVOMAX);
  Serial.println(init_pos_val); //375

  InitLeg(&legs[0],1,1,0,0,90);
  InitLeg(&legs[1],1,2,0,1,90);
  InitLeg(&legs[2],1,3,0,2,90);
  
  int val = 0;
  for (int i = 0 ; i < 3; i ++) {
  val = map(legs[i].pos,0,180,SERVOMIN,SERVOMAX);
    pwm.setPWM(legs[i].pin, 0, val);
  }
  delay(1000);
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


void MovFW(){
  int val = 0;
  for (int i = 0 ; i < 3; i++) {
    val = map(leg_1_move[0][i],0,180,SERVOMIN,SERVOMAX);
    pwm.setPWM(i,0, val);
    delay(50);
  }
  delay(500);
  for (int i = 2 ; i >= 0; i--) {
    val = map(leg_1_move[1][i],0,180,SERVOMIN,SERVOMAX);
    pwm.setPWM(i,0, val);
    delay(50);
  }
  delay(500); 
}
void loop(){
//    MovFW();
}
