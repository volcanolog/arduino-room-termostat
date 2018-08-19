#include <SPI.h>
#include "RF24.h"

int msg[1];

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
//Контакты от радиомодуля NRF24L01 подключаем к пинамнам -> Arduino

//SCK  -> 13
//MISO -> 12
//MOSI -> 11
//CSN  -> 10
//CE   -> 9

RF24 radio(9,10);

//светодиод и оптопара реле подключены к этим пинам
int LEDpin1 = 2;


// адреса каналов приема и передачи
const uint64_t pipes[2] = {
  0xF0F0F0F000LL, 0xF0F0F0F0FFLL};

void setup(){
  radio.begin();  
  radio.setDataRate(RF24_250KBPS);  // Скорость передачи
  radio.setChannel(100); // Номер канала от 0 до 127
  radio.setRetries(15,15); // Кол-во попыток и время между попытками
  radio.openWritingPipe(pipes[0]); // Открываем канал передачи
  radio.openReadingPipe(1, pipes[1]); // Открываем один из 6-ти каналов приема
  radio.startListening(); // Начинаем слушать эфир

  pinMode(LEDpin1, OUTPUT);

}

void loop(){
  if (radio.available()){
    bool done = false;    
    while (!done){
      done = radio.read(msg, 1);      
      //если пришел пакет от Arduino №1 (111) вКлючается светодиод (горит)LEDpin1, HIGH, замыкается реле
      if (msg[0] == 111){
        delay(10);
        digitalWrite(LEDpin1, HIGH);
      }
      if (msg[0] == 000){
        delay(10);
        digitalWrite(LEDpin1, LOW);
      }
    }
  }
}
