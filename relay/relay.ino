#include <avr/wdt.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include <ArduinoJson.h>

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
//Контакты от радиомодуля NRF24L01 подключаем к пинамнам -> Arduino
//SCK  -> 13
//MISO -> 12
//MOSI -> 11
//CSN  -> 10
//CE   -> 9
RF24 radio(9, 10);

//светодиод и оптопара реле подключены к этим пинам
#define Relay  6 // нога, к которой подключено реле
#define RelayOn LOW // полярность сигнала включения реде (HIGH/LOW)

static unsigned long time;

// адреса каналов приема и передачи
const uint64_t pipe01 = 0xF0F0F0F000LL;
const uint64_t pipe02 = 0xF0F0F0F0FFLL;
uint8_t msg[50];


void resetFunc() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void sendMessage(String param, String value) {
  uint8_t buf[50];
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root[param] = value;
  root["_time"] = millis();
  char output[50];
  root.printTo(output);
  strcpy((char*)buf, output);
  radio.stopListening();
  radio.write(buf, sizeof(buf));
  delay(10);
  radio.startListening();
}



void setup() {
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, HIGH);
  radio.begin();
  radio.setChannel(100); // канал (0-127)
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(15, 15); // Кол-во попыток и время между попытками
  radio.openReadingPipe(1, pipe01); // открываем трубу с индитификатором "pipe01"
  radio.openWritingPipe(pipe02);
  //radio.openReadingPipe(2,pipe02); // открываем трубу с индитификатором "pipe02"
  //radio.openReadingPipe(0, pipe01); // или открываем все трубы разом
  radio.startListening(); // включаем приемник, начинаем слушать трубу
  time = 0;
}

void loop() {

  // Если работаем больше 5 минут и от центра нет никаких сигналов, то ребут
  if (millis() - time > 60000)
  {
    resetFunc();
  }

  if (radio.available()) {
    radio.read(&msg, sizeof(msg));
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject((char*)msg);
    //если пришел пакет от Arduino №1 (111) вКлючается светодиод (горит)LEDpin1, HIGH, замыкается реле
    if (root["status"] == 77) {
      delay(10);
      time = millis();
      if (digitalRead(Relay) == !RelayOn) {
        digitalWrite(Relay, RelayOn);
      }
    }
    if (root["status"] == 11) {
      delay(10);
      time = millis();
      digitalWrite(Relay, !RelayOn);
    }

  } else {
    digitalRead(Relay) == RelayOn ? sendMessage("status", "55") : sendMessage("status", "33");
  }
  delay(10);
}
