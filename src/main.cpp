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
#include "uart_register.h"

#define TX_TEST

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Create a struct_message called myData
uint8_t myData [256] = {0};
char myData2 [] = "mensaje de prueba";
char myData3[40];
uint8_t i = 0;

byte receivedBytes[512];
byte numReceived = 0;
static byte ndx = 0;
static byte cnt = 0;
byte rb;
bool flag = 0;


// Structure example to send data
// Must match the receiver structure
typedef struct test_struct {
    int x;
    int y;
} test_struct;

// Create a struct_message called test to store variables to be sent
test_struct test = {};

unsigned long lastTime = 0;
unsigned long lastTimePollBuffer = 0;
unsigned long lastTimeWatchDog = 0;

unsigned long timerTransmit = 3000;
unsigned long timerPollBuffer = 2;  // send readings timer
unsigned long timerWatchDogRefresh = 200;


// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

  digitalWrite(2, !digitalRead(2));
  // memcpy(myData, incomingData, len);
  // myData[len] = '\0';
  // Serial.print("Bytes received: ");
  // Serial.print((char*)myData);


  digitalWrite(15, HIGH); //Pin Enable TX RS485 en BAJO
  Serial.write(incomingData,uint16_t(len));
  digitalWrite(15, LOW); //Pin Enable RX RS485
  

  //digitalWrite(0, HIGH);
  digitalWrite(4, HIGH);
  lastTimeWatchDog = millis();
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
  
  digitalWrite(2, !digitalRead(2));
}

  void mycallback(){
  // Serial.println("Working");
  flag = 1;

}

void serialEvent(){
  
}

void uart0_rx_intr_handler(void *para){

  uint8_t Rcv_Char;
  uint8_t uart_no= UART0;
  uint8_t fifo_len = 0;
  uint8_t buf_idx = 0;
  uint32_t uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)); //get uart intr status
  while (uart_intr_status != 0x0) {
    if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST)){ //if it is caused by a frm_err interrupt
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
      Serial.println("caused by a frm_err interrupt");
    } else if (UART_RXFIFO_FULL_INT_ST == (uart_intr_status & UART_RXFIFO_FULL_INT_ST)) { //if it is caused by a fifo_full interrupt
      Serial.println("caused by a fifo_full interrupt");
      fifo_len = (READ_PERI_REG(UART_STATUS(uart_no)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; //read rx fifo length
      char r[fifo_len];
      buf_idx = 0;
      while (buf_idx < fifo_len){
        r[buf_idx] = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
        buf_idx++;
      }
      r[fifo_len] = '\0';
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR); //clear full interrupt state
    } else if (UART_RXFIFO_TOUT_INT_ST == (uart_intr_status & UART_RXFIFO_TOUT_INT_ST)) { //if it is caused by a time_out interrupt
      // Serial.println("caused by a time_out interrupt");
      mycallback();
      fifo_len = (READ_PERI_REG(UART_STATUS(uart_no)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; //read rx fifo length
      char r[fifo_len];
      
      buf_idx = 0;
      while (buf_idx < fifo_len) {
        r[buf_idx] = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
        buf_idx++;
      }
      r[fifo_len] = '\0';

      esp_now_send(0, (uint8_t*)r, buf_idx);
      // Serial.write((uint8_t*)r, buf_idx);
      // Serial.flush();
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_TOUT_INT_CLR); //clear full interrupt state
    } else if (UART_TXFIFO_EMPTY_INT_ST == (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST)){ //if it is caused by a tx_empty interrupt
      Serial.println("caused by a tx_empty interrupt");
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
      CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_TXFIFO_EMPTY_INT_ENA);
    } else {

    }

    uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)); //update interrupt status

  }

}


static void install_uart_tout(){
  ETS_UART_INTR_DISABLE(); //Disable UART Interrupt
  ETS_UART_INTR_ATTACH(uart0_rx_intr_handler, NULL); //Attach handler function to uart0_rx_intr_handler

  SET_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_TOUT_INT_ENA);

  WRITE_PERI_REG(UART_CONF1(0), UART_RX_TOUT_EN | 
    ((0x2 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S)); //Enable UART RX Timeout function and set the timeout period as the time transmitting 2 bits

  WRITE_PERI_REG(UART_INT_CLR(0), 0xffff); //Clear UART Interrupts flags
  SET_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_TOUT_INT_ENA); //Enable UART RX Timeout interrupt
  CLEAR_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_FULL_INT_ENA); //Disable UART RX Full interrupt

  ETS_UART_INTR_ENABLE();
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
  digitalWrite(15, LOW); //Pin Enable 485 en ALTO / para recibir
  // Init Serial Monitor
  Serial.begin(9600);
  // Serial.setRxBufferSize(256);
  // install_uart_tout();
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  WiFi.setOutputPower(20.5);
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

  if (digitalRead(4) == HIGH){

    if ((millis() - lastTimeWatchDog) > timerWatchDogRefresh){

      digitalWrite(4, LOW);

      

    }



  } 

#ifdef TX_TEST

  if ((millis() - lastTime) > timerTransmit) {

 
    
    
    // Set values to send
    test.x = random(1, 50);
    test.y = random(1, 50);


    sprintf(myData3,"%s %d %d %d",myData2, i, test.x, test.y);
    
   
    // Send message via ESP-NOW
    //esp_now_send(0, (uint8_t *) &test, sizeof(test)+1);
    esp_now_send(0, (uint8_t*)myData3, strlen(myData3));
    i++;

    lastTime = millis();
    
  }


#else

if ((millis() - lastTimePollBuffer) > timerPollBuffer) {

  //Serial.println("Testing\r\n");

  if (Serial.available() > 0){

    while (Serial.available() > 0) {
      rb = Serial.read();
      receivedBytes[ndx] = rb;
      ndx++;
    }

  }else{
    if(ndx != 0){
    receivedBytes[ndx] = '\0'; // terminate the string
    numReceived = ndx;  // save the number for use when printing
    if(numReceived > 1){
      esp_now_send(0, receivedBytes, numReceived);
      // Serial.write(receivedBytes, numReceived);
      // Serial.flush();
    }
    
    ndx = 0;
    
    /*
    if (Serial.available() > 0){
      Serial.read();
      }
    */
    // Serial.read();
    
    }


    
  }
  // Serial.read();


  
   lastTimePollBuffer = millis();
    
}

#endif

}





