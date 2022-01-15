
#include "M5Atom.h"
#include <esp_now.h>
#include <WiFi.h>

#define MAX_TOLL 10

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

void loop(){
    delay(200);
    M5.IMU.getAttitude(&pitch, &roll);
    DetectMovement(pitch,roll);
}
