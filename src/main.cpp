#include <Arduino.h>
#include "driver/twai.h"

// Default for ESP32 CAN
#define CAN_TX		5
#define CAN_RX		4

extern bool can_init();
extern bool can_write(twai_message_t* frame);
extern bool can_read(twai_message_t* frame);
extern uint32_t can_tx_queue();
extern uint32_t can_rx_queue();
extern void can_clear_tx();
extern void can_clear_rx();

twai_message_t rxFrame; //для приема фреймов

void sendObdFrame(uint8_t obdId);
void handle_rx_message(twai_message_t& message);

//Global Variables
unsigned long previousMillis = 0;
unsigned long interval = 5000;  //5 sec.

void setup() {

  Serial.begin(115200);

    //Старт CAN
    if(can_init()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }

  Serial.println("----------------Start Info-----------------");
  Serial.printf("Total heap:\t%d \r\n", ESP.getHeapSize());
  Serial.printf("Free heap:\t%d \r\n", ESP.getFreeHeap());
  Serial.println("-------------------------------------------");
 
}

void loop() {

//Передаем пакеты CAN..
unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >=interval) {
    if(can_tx_queue()<3){
      sendObdFrame(5); // Передача запроса по CAN шине.
    } else {
      Serial.println("CAN BUS DOWN..");
    }
    previousMillis = currentMillis;
  }

  // Принимаем пакеты CAN..
  if(can_read(&rxFrame)) {
          handle_rx_message(rxFrame); //печатаем содержимое пакета
  }
}

//Посылаем запрос по шине CAN для Service=1 (параметр=PID)
//каждая станция CAN должна иметь свой адрес на передачу !!!
void sendObdFrame(uint8_t obdId) {
	twai_message_t obdFrame = { 0 };  //структура twai_messae_t инициализируем нулями
	obdFrame.identifier = 0x7E8; //адрес-ответ ECU OBDII (на запрос адрес: 0x7DF)
	obdFrame.extd = 0; //формат 11-бит
	obdFrame.data_length_code = 8; //OBD2 frame - всегда 8 байт !

	obdFrame.data[0] = 2; //количество значимых байт во фрейме
	obdFrame.data[1] = 1;     //Service OBD2
	obdFrame.data[2] = obdId; //PID OBD2
	obdFrame.data[3] = 0xAA;    // Best to use 0xAA (0b10101010) instead of 0
	obdFrame.data[4] = 0xAA;    // CAN works better this way as it needs
	obdFrame.data[5] = 0xAA;    // to avoid bit-stuffing
	obdFrame.data[6] = 0xAA;
	obdFrame.data[7] = 0xAA;
    // Accepts both pointers and references 
    if(can_write(&obdFrame)){  // timeout defaults to 1 ms
      Serial.printf("--Frame sent: %03X tx_queue: %d\r\n",obdFrame.identifier,can_tx_queue());
    } else {
      Serial.printf("tx_queue: %d\r\n",can_tx_queue());
    }
}

//Распечатка содержимого принятого пакета CAN
void handle_rx_message(twai_message_t& message) {

  if (message.extd) {
    Serial.print("CAX: 0x");
  } else {
    Serial.print("CAN: 0x");
  }
  Serial.print(message.identifier, HEX);
  Serial.print(" (");
  Serial.print(message.identifier, DEC);
  Serial.print(")");
  Serial.print(" [");
  Serial.print(message.data_length_code, DEC);
  Serial.print("] <");
  for (int i = 0; i < message.data_length_code; i++) {
    if (i != 0) Serial.print(":");
    Serial.print(message.data[i], HEX);
  }
  Serial.println(">");
}