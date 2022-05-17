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
uint8_t myData [256] = {0};

byte receivedBytes[256];
byte numReceived = 0;
static byte ndx = 0;
byte rb;



unsigned long lastTime = 0;  
unsigned long timerDelay = 500;  // send readings timer

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

  digitalWrite(2, !digitalRead(2));
  memcpy(myData, incomingData, len);
  myData[len] = '\0';
  Serial.print((char*)myData);
  //digitalWrite(0, HIGH);
  digitalWrite(4, HIGH);
  delay(100);
  //digitalWrite(0, LOW);
  digitalWrite(4, LOW);
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

  
  digitalWrite(2, !digitalRead(2));
}



void setup() {
  // Establecer el pin del LED en modo salida
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT); //Pin WD como salida
  //pinMode(0, OUTPUT); 
  pinMode(15, OUTPUT);  //Pin Enable 485
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW); //Pin WD en Bajo
  //digitalWrite(0, LOW);
  digitalWrite(15, HIGH); //Pin Enable 485 en BAJO
  // Init Serial Monitor
  Serial.begin(9600);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);


    // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);






}
 
void loop() {
  // if ((millis() - lastTime) > timerDelay) {

  //   /*
  //   // Set values to send
  //   test.x = random(1, 50);
  //   test.y = random(1, 50);
  //   */

   
  //   // Send message via ESP-NOW
  //   esp_now_send(0, (uint8_t *) &test, sizeof(test));

  //   lastTime = millis();
    
  // }

if ((millis() - lastTime) > timerDelay) {

  //Serial.println("Testing\r\n");
  
  while (Serial.available() > 0) {
      rb = Serial.read();
      receivedBytes[ndx] = rb;
      ndx++;
  }

  if(ndx != 0){

    receivedBytes[ndx] = '\0'; // terminate the string
    numReceived = ndx;  // save the number for use when printing
    esp_now_send(0, receivedBytes, numReceived);
    ndx = 0;
    if (Serial.available() > 0){
      Serial.read();
      }
    
  }


   lastTime = millis();
    
}


}
