/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp8266-nodemcu/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Create a struct_message called myData
uint8_t myData [50] = {0};

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

  digitalWrite(9, HIGH);
  digitalWrite(2, !digitalRead(2));
  memcpy(myData, incomingData, sizeof(myData));

  Serial.print((char*)myData);
  digitalWrite(9, LOW);
  /*
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(myData.x);
  Serial.print("y: ");
  Serial.println(myData.y);
  Serial.println();
  */
  
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {

  /*
  char macStr[18];
  Serial.print("Packet to:");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status: ");

  
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }

  */
  digitalWrite(5, !digitalRead(5));
}



void setup() {
  // Establecer el pin del LED en modo salida
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);





  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  //esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  //esp_now_add_peer(broadcastAddress2, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  //esp_now_add_peer(broadcastAddress3, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);




}
 
void loop() {
  if ((millis() - lastTime) > timerDelay) {

    /*
    // Set values to send
    test.x = random(1, 50);
    test.y = random(1, 50);
    */

   
    // Send message via ESP-NOW
    esp_now_send(0, (uint8_t *) &test, sizeof(test));

    lastTime = millis();
    
  }
}