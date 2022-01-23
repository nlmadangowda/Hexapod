
#include "M5Atom.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ACROBOTIC_SSD1306.h>

#define MAX_TOLL 10
#define TRIG_PIN 19
#define ECHO_PIN 22

enum _hand_dir_{
  DIR_FRONT = 0,
  DIR_BACK,
  DIR_LEFT,
  DIR_RIGHT
};

typedef struct _payload_ Payload;
#ifndef STRUCT_PACK
#define STRUCT_PACK
#pragma pack(1)
struct _payload_{
  uint8_t s_dir;
};
#undef STRUCT_PACK
#endif //STRUCT_PACK

double pitch ;
double roll;
uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xCC, 0x85, 0x08};


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup(){
  M5.begin(true, true, true);
  M5.IMU.Init();
  WiFi.mode(WIFI_STA);
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
  oled.init();                      // Initialze SSD1306 OLED display
  oled.setFont(font5x7);            // Set font type (default 8x8)
  oled.clearDisplay();              // Clear screen
  oled.setTextXY(0,0);              // Set cursor position, start of line 0
  oled.putString("Madan");
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
}

void PrintPattren(uint8_t p_dir){
  const uint8_t g_pattren[5][25]={
                        {2,6,7,8,12,17,22},
                        {2,7,12,16,17,18,22},
                        {6,10,11,12,13,14,16},
                        {8,10,11,12,13,14,18}
  };
  
  M5.dis.clear();
  for(int i = 0 ; i < 7; i++){
    M5.dis.drawpix(g_pattren[p_dir][i],0xf00000);
  }
}
void DetectMovement(double p_pitch, double p_roll){
  if(p_roll > (1*MAX_TOLL)){
    SendSigToPod(DIR_FRONT);
  }else if(p_roll <-(1*MAX_TOLL)){
    SendSigToPod(DIR_BACK);
  }else if( p_pitch >(1*MAX_TOLL)){
    SendSigToPod(DIR_LEFT);
  }else if( p_pitch <-(1*MAX_TOLL)){
    SendSigToPod(DIR_RIGHT);
  }else{
    Serial.println("Halt");
  }
}

void SendSigToPod(uint8_t p_dir){
  static uint8_t l_last_sent_data = 0;
  if(p_dir != l_last_sent_data){
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &p_dir, sizeof(p_dir));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      PrintPattren(p_dir);
      l_last_sent_data = p_dir;
    }
    else {
      Serial.println("Error sending the data");
    }
  }else{
    Serial.println("No Change in data to send");
  }
}
float duration_us, distance_cm;

void loop(){
    delay(200);
   M5.IMU.getAttitude(&pitch, &roll);
   DetectMovement(pitch,roll);
    // generate 10-microsecond pulse to TRIG pin
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
    delay(500);
}
