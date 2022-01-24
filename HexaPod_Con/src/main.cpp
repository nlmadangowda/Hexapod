#include "M5Atom.h"
#include <esp_now.h>
#include <WiFi.h>

#define GET_BIT_STATUS(_value_,_pos_)   ((_value_ & (1<<_pos_)) >> _pos_)
#define MAX_TOLL 10

enum _hand_dir_{
  DIR_HALT = 0,
  DIR_FRONT = 1,
  DIR_BACK = 2,
  DIR_LEFT = 3,
  DIR_RIGHT = 4,
  DIR_NO_DIR = 5,
};


const uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xCC, 0x85, 0x08};
const uint32_t g_disp_dir[]={0xFFFFFFFF,0x004255c4,0x00475484,0x00417c44,0x00447D04,0x01151151};
uint32_t g_disp_invrs = 0;
double pitch ;
double roll;

void SendSigToPod(uint8_t p_dir);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void PrintPattren(uint8_t p_dir);
void DetectMovement(double p_pitch, double p_roll);


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Packet Send Status: Success" : "Packet Send Status: Fail");
}

void OnDataReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.print("\r\n Last Packet Send Status:\t");
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
  esp_now_register_recv_cb(OnDataReceive);

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
  for(int i = 0 ; i < 25; i++){
    if(GET_BIT_STATUS(g_disp_dir[p_dir],i)){
      M5.dis.drawpix(i,0xf00000);
    }else{
      M5.dis.drawpix(i,0x00000);
    }
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
    SendSigToPod(DIR_HALT);
  }
}

void loop(){
  delay(200);
  M5.IMU.getAttitude(&pitch, &roll);
  DetectMovement(pitch,roll);
}