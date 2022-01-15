#include "M5Atom.h"
#include <esp_now.h>
#include <WiFi.h>


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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Bytes received: ");
  Serial.println(len);
  PrintPattren(*incomingData);
}


void setup() {
   M5.begin(true, true, true);
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

 void loop(){}
