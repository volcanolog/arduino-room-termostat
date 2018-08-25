#include <avr/wdt.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <RTClib.h>
#include <DallasTemperature.h>
#include <EEPROMex.h> // EE
#include <TimerOne.h> // прерывания по таймеру1

//радио модуль
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
// адреса каналов приема и передачи
const uint64_t pipe01 = 0xF0F0F0F000LL;
const uint64_t pipe02 = 0xF0F0F0F0FFLL;
uint8_t radioStat = 0;
byte radioFails = 0;
static unsigned long radioTime = 0;
//Контакты от радиомодуля NRF24L01 подключаем к пинамнам -> Arduino
//SCK  -> 13
//MISO -> 12
//MOSI -> 11
//CSN  -> 10
//CE   -> 9
RF24 radio(9, 10);

#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
DeviceAddress DS18B20Address;

#define encoderA    3 // энкодер - поворот вправо (об землю)
#define encoderB    2 // энкодер - поворот влево (об землю)
#define encoderK    4 // энкодер - кнопка (об землю)
#define Relay  6 // нога, к которой подключено реле
#define RelayOn LOW // полярность сигнала включения реде (HIGH/LOW)
#define BeepPin     5 // пищалка
#define BeepToneNo  2000 // тон звука "No", герц
#define BeepToneYes 4000 // тон звука "Yes", герц
#define BeepToneNoDuration 200 // длительность звука "No", мс
#define BeepToneYesDuration 200 // длительность звука "Yes", мс

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

RTC_DS1307 RTC;

byte char_1[8] = {
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
};

byte block1[8] = {
  B00100,
  B01110,
  B11111,
  B00100,
  B00100,
  B00100,
  B00100,
};
byte block2[8] = {
  0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00,
};

#define serialenabled // раскомментировать для выдачи в порт отладочной инфы

#define TstatTimerMax 60 //минимальная пауза между включениями горелки, сек
unsigned int TstatTimer = 60; //таймер паузы между включениями/выключениями, начальная установка 20 сек для устаканивания системы после сброса

float DS18B20Temperature = 0; //сырая температура от датчика
float Temperature = 24; //вычисленная температура с коррекцией
float DS18B20TempTmp; //времянка
byte DS18B20iteration = 0; //счётчик измерений температуры для усреднения

float TstatTemp = 22; //температура термостатирования, может изменяться настройками
float TemperatureCorr = 0; //коррекция температуры, может изменяться настройками
float TimerTShift = 0;
float Hysteresis = 0.1; // гистерезис термостата, может изменяться настройками
float HysteresisOld;

int Hours = 0; // времянка часов RTC для отображения и установки
int Minutes = 0; // времянка минут RTC для отображения и установки
int EHours = 0; // времянка часов RTC для отображения и установки
int EMinutes = 0; // времянка минут RTC для отображения и установки
int Seconds = 0;
boolean PrintYesNo = false; // показывать ли после времени Yes/No (косвенно - указание на режим установка/отображение)
boolean SetH = false; // выделение часов при отображении
boolean SetM = false; // выделение минут при отображении
boolean SetEnd = false; // выделение минут при отображении
boolean SetYesNo = false; // выделение Yes/No при установке часов
DateTime _now = 0;

boolean blink500ms = false; // мигающий бит, инвертируется каждые 500мс
boolean plus1sec = false; // ежесекундно взводится

boolean BeepEnabled = false;

byte MenuTimeoutTimer;

// структура для суточных таймеров (8 байт)
struct buffer_template {
  byte SHours;
  byte SMinutes;
  byte EHours;
  byte EMinutes;
  float Temperature;
  boolean Enabled;
  boolean Activated;
};
buffer_template Timer[4] = {
  0, 0, 0, 0, 4, false, false
}; //объявление 4-х суточных таймеров и их начальные значения

float AlarmTemp = 20; // температура для замерзательного орала

// encoder vars
static boolean rotating = false;    // debounce management
boolean A_set = false;
boolean B_set = false;
boolean encoderR = false;
boolean encoderL = false;


// EEPROM
EEMEM float TstatTempEE; //EE температура термостатирования
EEMEM float TemperatureCorrEE; // EE коррекция температуры
EEMEM float HysteresisEE; // EE гистерезис
EEMEM boolean BeepEnabledEE; // EE признак разрешения звука
EEMEM float AlarmTempEE; // EE значение недопустимого снижения температуры
EEMEM buffer_template TimerEE[4]; // EE структуры для 4 суточных таймеров

void PrintRTC(char x, char y, bool startEnd = false);
void resetFunc() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

// ===== SETUP ========================================================================
void setup() {

#ifdef serialenabled
  Serial.begin(9600);
#endif
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, HIGH);
  lcd.init();
  lcd.backlight();
  lcd.createChar(1, block1);
  lcd.createChar(2, block2);
  pinMode(encoderA, INPUT);
  digitalWrite(encoderA, HIGH);
  pinMode(encoderB, INPUT);
  digitalWrite(encoderB, HIGH);
  pinMode(encoderK, INPUT);
  digitalWrite(encoderK, HIGH);
  attachInterrupt(0, doEncoderA, CHANGE);   // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE);  // encoder pin on interrupt 1 (pin 3)
  Timer1.initialize(500000); // Timer0 interrupt - set a timer of length 500000 microseconds
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  EEPROM.setMaxAllowedWrites(32767);
  if ((digitalRead(encoderK)) == 0)
  { // если первая запись однокристалки (зажата кнопка при включении питания)- записать начальные значения в EE
    lcd.setCursor(0, 0); //инфо на LCD
    lcd.print(F("Cold start..."));
    for (int i = 0; i < 4; i++)
    {
      EEPROM.updateBlock(int(&TimerEE[i]), Timer[i]);
    }
    EEPROM.updateFloat(int(&TstatTempEE), TstatTemp);
    EEPROM.updateByte(int(&BeepEnabledEE), BeepEnabled);
    EEPROM.updateFloat(int(&TemperatureCorrEE), TemperatureCorr);
    EEPROM.updateFloat(int(&HysteresisEE), Hysteresis);
    EEPROM.updateFloat(int(&AlarmTempEE), AlarmTemp);
    tone(BeepPin, 2000, 50);
    delay(50);
    tone(BeepPin, 3000, 50);
    delay(50);
    tone(BeepPin, 4000, 50);
    delay(1000);
  }
  lcd.clear();
  lcd.setCursor(0, 0); //инфо на LCD
  lcd.print(F("Read settings..."));
  BeepEnabled = EEPROM.readByte(int(&BeepEnabledEE));
  TstatTemp = EEPROM.readFloat(int(&TstatTempEE));
  TemperatureCorr = EEPROM.readFloat(int(&TemperatureCorrEE));
  Hysteresis = EEPROM.readFloat(int(&HysteresisEE));
  AlarmTemp = EEPROM.readFloat(int(&AlarmTempEE));
  for (int i = 0; i < 4; i++)
  {
    EEPROM.readBlock(int(&TimerEE[i]), Timer[i]);
  }

  DS18B20.begin();
  DS18B20.getAddress(DS18B20Address, 0);
  DS18B20.setResolution(DS18B20Address, 12);
  DS18B20.setWaitForConversion(false);
  DS18B20.requestTemperatures();

  tone(BeepPin, 4000, 50);
  delay(100);
  tone(BeepPin, 4000, 50);
  delay(1000);
  lcd.clear();
  RTC.begin();
  if (! RTC.isrunning()) {
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  // Инициализация радио модуля
  radio.begin();
  delay(2);
  radio.setChannel(100); // канал (0-127)
  radio.setRetries(15, 15); // Кол-во попыток и время между попытками
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, pipe02);
  radio.openWritingPipe(pipe01);
  radio.startListening();
  delay(1000);

}

uint8_t msg[50];

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


// ===== MAIN CYCLE ===================================================================
void loop() {

  // Пришел статус от реле или 5 секунд молчание от него
  if (radio.available() || (radioTime + 5000 < millis())) {
    radioTime = millis();
    radio.read(&msg, sizeof(msg));
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject((char*)msg);
    radioStat = 0;
    if (root["status"] == 33) {
      radioFails = 0;
      radioStat = 1;
    }
    if (root["status"] == 55) {
      radioFails = 0;
      radioStat = 2;
    }
    if (radioStat == 0) {
      radioFails++;
    }

  }

  // Принимаем решение перезагрузиться, возможно отвалилось радио.
  if (radioFails > 10) {
    resetFunc();
  }

  mainScreenUpdate();
  // термостатирование
  if ( TstatTimer == 0 )
  {
    if ( Temperature > ( TstatTemp + Hysteresis + TimerTShift ) ) // гистерезис
    {
      if ( digitalRead(Relay) == RelayOn ) // если горелка включена -
      {
        digitalWrite(Relay, !RelayOn); // выключить горелку
        TstatTimer = TstatTimerMax; // горелку держать выключённой не менее заданного в TstatTimerMax времени
      }
    }
    if (Temperature < (TstatTemp + Hysteresis + TimerTShift))
    {
      if ( digitalRead(Relay) == !RelayOn ) // если горелка выключена -
      {
        digitalWrite(Relay, RelayOn); // включить горелку
        TstatTimer = TstatTimerMax; // горелку держать включённой не менее заданного в TstatTimerMax времени
      }
    }
  }

  // если прошла 1 секунда - делаем ежесекундные дела
  if (plus1sec) {
    // Проверяем команду на горелку и отправляем радио сигнал
    uint8_t msg;
    if (digitalRead(Relay) == RelayOn) {
      msg = 77;
    } else {
      msg = 11;
    }
    sendMessage("status", String(msg));
    plus1sec = false; // сбрасываем до следующей секунды
    // измеряем температуру воздуха
    getCurrTemp();
    // обновляем часы
    _now = RTC.now();
    Hours = _now.hour();
    Minutes = _now.minute();
    Seconds = _now.second();
    // проверим таймеры
    checkTimers();
    // если уставку термостата поменяли вручную - запись её в EE, не чаще 1 раза в минуту (экономия ресурса EE)
    if ((EEPROM.readFloat(int(&TstatTempEE)) != TstatTemp) & (Seconds == 0)) {
      EEPROM.updateFloat(int(&TstatTempEE), TstatTemp);
    }
    if (Temperature < AlarmTemp) {
      tone(BeepPin, 4000, 5);
    }
  }

  // обработка поворота энкодера на лету (ручное изменение уставки температуры))
  rotating = true;  // reset the debouncer
  if ((encoderR) | (encoderL)) {
    if (encoderR) {
      TstatTemp += 0.1;
    }
    else
    {
      TstatTemp -= 0.1;
    }
    TstatTemp = constrain(TstatTemp, 7, 35);
    encoderR = false;
    encoderL = false;
  }

  // ================ по нажатию кнопки энкодера - меню настроек ====================
  if (digitalRead(encoderK) == 0) {
    MenuTimeoutTimer = 10; //таймер таймаута, секунд
    lcd.clear();
    lcd.setCursor(0, 0); //инфо на LCD
    lcd.print(F("< SETUP >"));
    if (BeepEnabled) {
      tone(BeepPin, 4000, 50);
    }
    delay(200);
    int menuitem = 0;

    do {
      rotating = true;  // reset the debouncer
      if ((encoderR) | (encoderL)) {
        MenuTimeoutTimer = 10; //таймер таймаута, секунд
        if (encoderR) {
          menuitem += 1;
        }
        else  {
          menuitem -= 1;
        }
        if ( menuitem > 9 ) {
          menuitem = 0;
        } // границы пунктов меню
        if ( menuitem < 0 ) {
          menuitem = 9;
        }
        encoderR = false;
        encoderL = false;
      }

      // индикация пункта меню (номер пункта - в menuitem)
      lcd.setCursor(0, 1); //инфо на LCD
      switch (menuitem)
      {
        case 0:
          lcd.print(F("0.BACK          "));
          break;
        case 1:
          lcd.print(F("1.TIMER1 SET    "));
          break;
        case 2:
          lcd.print(F("2.TIMER2 SET    "));
          break;
        case 3:
          lcd.print(F("3.TIMER3 SET    "));
          break;
        case 4:
          lcd.print(F("4.TIMER4 SET    "));
          break;
        case 5:
          lcd.print(F("5.CLOCK SET     "));
          break;
        case 6:
          lcd.print(F("6.HYSTERESIS SET"));
          break;
        case 7:
          lcd.print(F("7.T-CORRECT SET "));
          break;
        case 8:
          lcd.print(F("8.SOUND SET     "));
          break;
        case 9:
          lcd.print(F("9.T-ALARM SET   "));
          break;
      }
      if (MenuTimeoutTimer == 0) {
        menuitem = 0;
      }

    }
    while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
    // если нажата кнопка энкодера или таймаут - обработка пункта меню (номер пункта - в menuitem)
    if (BeepEnabled) {
      tone(BeepPin, 4000, 50);
    }

    switch (menuitem)
    {
      // ====== пункт 0 - выход
      case 0:
        if (BeepEnabled) {
          tone(BeepPin, BeepToneNo, BeepToneNoDuration);
        } //звук "NO"
        break; // case 0 out

      // ====== пункт 1 - установка Timer1
      case 1:
        TimerXSetup(0);
        break; // case 1 out

      // ====== пункт 2 - установка Timer2
      case 2:
        TimerXSetup(1);
        break; // case 2 out

      // ====== пункт 3 - установка Timer3
      case 3:
        TimerXSetup(2);
        break; // case 3 out

      // ====== пункт 4 - установка Timer4
      case 4:
        TimerXSetup(3);
        break; // case 4 out

      // ====== пункт 5 - установка RTC
      case 5:
        MenuTimeoutTimer = 10; //таймер таймаута, секунд
        lcd.clear();
        lcd.setCursor(0, 0); //инфо на LCD
        lcd.print(F("SETUP CLOCK"));
        delay(200);
        _now = RTC.now();
        Hours = _now.hour();
        Minutes = _now.minute();
        SetYesNo = false;
        PrintYesNo = true;
        SetTime(0, 1); // в позиции 0,1 - запрос ввода времени
        if (MenuTimeoutTimer != 0) {
          if (SetYesNo)
          {
            if (BeepEnabled) {
              tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
            }
            RTC.adjust(DateTime(_now.year(), _now.month(), _now.day(), Hours, Minutes, 0));
            RTC.begin();
          }
          else
          {
            if (BeepEnabled) {
              tone(BeepPin, BeepToneNo, BeepToneNoDuration); //звук "NO"
            }
          }
        }
        else {
          if (BeepEnabled) {
            tone(BeepPin, BeepToneNo, BeepToneNoDuration); //звук "NO"
          }
        }
        break; // case 5 out

      // ====== пункт 6 - установка гистерезиса
      case 6:
        MenuTimeoutTimer = 10; //таймер таймаута, секунд
        HysteresisOld = Hysteresis;
        lcd.clear();
        lcd.setCursor(0, 0); //инфо на LCD
        lcd.print(F("SETUP HYSTERESIS"));
        delay(200);
        do {
          lcd.setCursor(0, 1);
          if (blink500ms) {
            lcd.print("   ");
          }
          else {
            lcd.print(Hysteresis, 1);
            lcd.write(0x02); // значок градуса
          }
          rotating = true;  // reset the debouncer
          if (encoderR) {
            Hysteresis += 0.1;
            encoderR = false;
          }
          if (encoderL) {
            Hysteresis -= 0.1;
            encoderL = false;
          }
          Hysteresis = constrain(Hysteresis, 0.1, 2); // крайние значения
        }
        while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
        if (MenuTimeoutTimer != 0) {
          EEPROM.updateFloat(int(&HysteresisEE), Hysteresis); // запись в ЕЕПРОМ
          if (BeepEnabled) {
            tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
          }
        }
        else {
          Hysteresis = HysteresisOld;
          if (BeepEnabled) {
            tone(BeepPin, BeepToneNo, BeepToneNoDuration); //звук "NO"
          }
        }
        break; // case 6 out

      // ====== пункт 7 - установка коррекции температуры
      case 7:
        MenuTimeoutTimer = 10; //таймер таймаута, секунд
        lcd.clear();
        lcd.setCursor(0, 0); //инфо на LCD
        lcd.print(F("SETUP T-CORRECT "));
        delay(200);
        do {
          lcd.setCursor(0, 1);
          if (blink500ms) {
            lcd.print(F("    "));
          }
          else {
            if (TemperatureCorr >= 0) {
              lcd.print(F("+"));
            }

            lcd.print(TemperatureCorr, 1);
            lcd.write(0x02); // значок градуса
          }
          rotating = true;  // reset the debouncer
          if (encoderR) {
            TemperatureCorr += 0.1;
            encoderR = false;
          }
          if (encoderL) {
            TemperatureCorr -= 0.1;
            encoderL = false;
          }
          TemperatureCorr = constrain(TemperatureCorr, -8, 8); // крайние значения
        }
        while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));

        if (MenuTimeoutTimer != 0) {
          EEPROM.updateFloat(int(&TemperatureCorrEE), TemperatureCorr); // запись в ЕЕПРОМ
          if (BeepEnabled) {
            tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
          }
        }
        else {
          TemperatureCorr = EEPROM.readFloat(int(&TemperatureCorrEE));
          if (BeepEnabled) {
            tone(BeepPin, BeepToneNo, BeepToneNoDuration); //звук "NO"
          }
        }
        break; // case 7 out

      // ====== пункт 8 - вкл/выкл звука
      case 8:
        MenuTimeoutTimer = 10; //таймер таймаута, секунд
        lcd.clear();
        lcd.setCursor(0, 0); //инфо на LCD
        lcd.print(F("SOUND SET       "));
        delay(200);
        do {
          lcd.setCursor(0, 1);
          if (BeepEnabled) {
            lcd.print(F("BEEP ON         "));
          }
          else {
            lcd.print(F("BEEP OFF        "));
          }

          rotating = true;  // reset the debouncer
          if ((encoderR) | (encoderL)) {
            BeepEnabled = !BeepEnabled;
            encoderR = false;
            encoderL = false;
          }
        }
        while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));

        if (MenuTimeoutTimer != 0) {
          if (BeepEnabled) {
            tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
          }
          EEPROM.updateByte(int(&BeepEnabledEE), BeepEnabled);
        }
        if (MenuTimeoutTimer == 0) {
          BeepEnabled = EEPROM.readByte(int(&BeepEnabledEE));
        }
        break; // case 8 out

      // ====== пункт 9 - установка предупреждалки о холоде
      case 9:
        MenuTimeoutTimer = 10; //таймер таймаута, секунд
        lcd.clear();
        lcd.setCursor(0, 0); //инфо на LCD
        lcd.print(F("ALARM-TEMP SET  "));
        delay(200);
        do {
          lcd.setCursor(0, 1);
          if (blink500ms) {
            lcd.print(F("    "));
          }
          else {
            if (AlarmTemp >= 0) {
              lcd.print(F("+"));
            }

            lcd.print(AlarmTemp, 0);
            lcd.write(0x02); // значок градуса
          }
          rotating = true;  // reset the debouncer
          if (encoderR) {
            AlarmTemp += 1;
            encoderR = false;
          }
          if (encoderL) {
            AlarmTemp -= 1;
            encoderL = false;
          }
          AlarmTemp = constrain(AlarmTemp, 15, 30); // крайние значения
        }
        while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));

        if (MenuTimeoutTimer != 0) {
          EEPROM.updateFloat(int(&AlarmTempEE), AlarmTemp); // запись в ЕЕПРОМ
          if (BeepEnabled) {
            tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
          }
        }
        else {
          AlarmTemp = EEPROM.readFloat(int(&AlarmTempEE));
          if (BeepEnabled) {
            tone(BeepPin, BeepToneNo, BeepToneNoDuration); //звук "NO"
          }
        }
        break; // case 9 out

    }

    delay(200);
    lcd.clear();
  }
}

// ===== SUBROUTINES ==================================================================
void mainScreenUpdate() {
  lcd.setCursor(8, 0); //инфо на LCD
  if ((Temperature < AlarmTemp) & (blink500ms)) {
    lcd.print(F("*"));
  }
  else {
    lcd.print(F(" "));
  }

  switch (radioStat) {
    case 1: lcd.print("-"); break;
    case 2: lcd.print("+"); break;
    default: lcd.print("!");
      tone(BeepPin, 4000, 5);
  }

  lcd.print(F("t="));
  if (Temperature < 10) {
    lcd.print(F(" "));
  }
  lcd.print(Temperature, 1);
  lcd.write(0x02); // значок градуса

  // если таймер включен - надпись светится, если сработал - мигает, обрабатываем все 4 таймера
  lcd.setCursor(0, 1); //инфо на LCD
  for (int i = 0; i < 4; i++) {
    ((Timer[i].Enabled) & !((Timer[i].Activated) & (blink500ms))) ?  lcd.print(i + 1) : lcd.print(F(" "));
  }

  lcd.setCursor(6, 1); //инфо на LCD
  lcd.print(TstatTemp, 1);
  (TstatTemp + TimerTShift) > Temperature ? lcd.print("<") : lcd.print(">");
  lcd.print(TstatTemp + TimerTShift, 1);
  digitalRead(Relay) == RelayOn ? lcd.write(0x01) : lcd.write(0x02);
  // печатаем текущее время
  PrintYesNo = false;
  PrintRTC(0, 0);
}

// ========================================
void checkTimers()
{
  unsigned int toMinutes = Hours * 60 + Minutes;
  TimerTShift = 0;
  // проверка таймеров и изменение уставки термостата при совпадении
  for (int i = 0; i < 4; i++)
  {
    unsigned int timerStartToMinutes = Timer[i].SHours * 60 + Timer[i].SMinutes;
    unsigned int timerEndToMinutes = Timer[i].EHours * 60 + Timer[i].EMinutes;
    // время T совпадает с RTC
    if (
      (
        // Внутри одного дня
        ((timerStartToMinutes <= timerEndToMinutes) && (toMinutes > timerStartToMinutes && toMinutes < timerEndToMinutes)) ||
        // Через день
        ((timerStartToMinutes > timerEndToMinutes) && (toMinutes > timerStartToMinutes || toMinutes < timerEndToMinutes))
      ) && Timer[i].Enabled)
    {
      Timer[0].Activated = Timer[1].Activated = Timer[2].Activated = Timer[3].Activated = false;
      Timer[i].Activated = true;
      TimerTShift = Timer[i].Temperature;
      break;
    }
  }

}
// ========================================
void getCurrTemp() {
  DS18B20TempTmp = DS18B20.getTempCByIndex(0); // получить температуру от датчика
  DS18B20.requestTemperatures();  // запустить новое измерение
  if (DS18B20TempTmp != -127)
  {
    DS18B20Temperature += DS18B20TempTmp; // суммируем для усреднения
    DS18B20iteration ++;
    if (DS18B20iteration == 10)
    {
      DS18B20iteration = 0;
      Temperature = (DS18B20Temperature / 10) + TemperatureCorr; //усреднённая + коррекция
      DS18B20Temperature = 0;
    }
  }
}

// ========================================
void SetTime(char x, char y)
{
  // ========= set hours
  SetH = true;
  do {
    PrintRTC(x, y);
    rotating = true;  // reset the debouncer
    if (encoderR) {
      Hours == 23 ? Hours = 0 : Hours++;
      encoderR = false;
    }
    if (encoderL) {
      Hours == 0 ? Hours = 23 : Hours--;
      encoderL = false;
    }
  }
  while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
  if (BeepEnabled) {
    tone(BeepPin, 4000, 50); //звук "YES"
  }
  SetH = false;
  delay(200);
  // ========= set minutes
  SetM = true;
  do {
    PrintRTC(0, 1);
    rotating = true;  // reset the debouncer
    if (encoderR) {
      Minutes == 59 ? Minutes = 0 : Minutes++;
      encoderR = false;
    }
    if (encoderL) {
      Minutes  == 0 ? Minutes = 59 : Minutes--;
      encoderL = false;
    }
  }
  while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
  if (BeepEnabled) {
    tone(BeepPin, 4000, 50); //звук "YES"
  }
  if (PrintYesNo) {
    SetM = false;
    delay(200);
    // ========= set yes-no
    SetYesNo = false;
    do {
      PrintRTC(0, 1);
      rotating = true;  // reset the debouncer
      if ((encoderR) || (encoderL)) {
        SetYesNo = !SetYesNo;
        encoderR = false;
        encoderL = false;
      }
    }
    while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
    delay(200);
  }
}

// ========================================
void SetTimerTime(char x, char y)
{
  // ========= set hours
  SetH = true;
  do {
    PrintRTC(x, y, true);
    rotating = true;  // reset the debouncer
    if (encoderR) {
      Hours == 23 ? Hours = 0 : Hours++;
      encoderR = false;
    }
    if (encoderL) {
      Hours == 0 ? Hours = 23 : Hours--;
      encoderL = false;
    }
  }
  while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
  if (BeepEnabled) {
    tone(BeepPin, 4000, 50); //звук "YES"
  }
  SetH = false;
  delay(200);
  // ========= set minutes
  SetM = true;
  do {
    PrintRTC(0, 1, true);
    rotating = true;  // reset the debouncer
    if (encoderR) {
      Minutes == 59 ? Minutes = 0 : Minutes++;
      encoderR = false;
    }
    if (encoderL) {
      Minutes  == 0 ? Minutes = 59 : Minutes--;
      encoderL = false;
    }
  }
  while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
  SetM = false;
  delay(200);
  // ========= set hours
  SetH = true;
  SetEnd = true;
  do {
    PrintRTC(0, 1, true);
    rotating = true;  // reset the debouncer
    if (encoderR) {
      EHours == 23 ? EHours = 0 : EHours++;
      encoderR = false;
    }
    if (encoderL) {
      EHours == 0 ? EHours = 23 : EHours--;
      encoderL = false;
    }
  }
  while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
  if (BeepEnabled) {
    tone(BeepPin, 4000, 50); //звук "YES"
  }
  SetH = false;
  delay(200);
  // ========= set minutes
  SetM = true;
  do {
    PrintRTC(0, 1, true);
    rotating = true;  // reset the debouncer
    if (encoderR) {
      EMinutes == 59 ? EMinutes = 0 : EMinutes++;
      encoderR = false;
    }
    if (encoderL) {
      EMinutes  == 1 ? EMinutes = 0 : EMinutes--;
      encoderL = false;
    }
  }
  while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
  SetM = false;
  SetEnd = false;
  delay(200);


  if (BeepEnabled) {
    tone(BeepPin, 4000, 50); //звук "YES"
  }
  if (PrintYesNo) {
    delay(200);
    // ========= set yes-no
    SetYesNo = false;
    do {
      PrintRTC(0, 1, true);
      rotating = true;  // reset the debouncer
      if ((encoderR) || (encoderL)) {
        SetYesNo = !SetYesNo;
        encoderR = false;
        encoderL = false;
      }
    }
    while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));
    delay(200);
  }
}
// ========================================
void PrintRTC(char x, char y, bool startEnd = false)
{
  lcd.setCursor(x, y);
  if (SetH && blink500ms && !SetEnd ) {
    lcd.print(F("  "));
  }
  else {
    if (Hours < 10) {
      lcd.print(F("0"));
    }
    lcd.print(Hours);
  }

  // мигающее двоеточие, если не в режиме установки времени
  if (!(SetH || SetM || PrintYesNo || blink500ms))
  {
    lcd.print(F(" "));
  }
  else {
    lcd.print(F(":"));
  }

  if (SetM && blink500ms && !SetEnd) {
    lcd.print(F("  "));
  }
  else {
    if (Minutes < 10) {
      lcd.print(F("0"));
    }
    lcd.print(Minutes);
  }
  lcd.print(F(" "));

  if (startEnd) {
    if (SetH && SetEnd && blink500ms) {
      lcd.print(F("  "));
    }
    else {
      if (EHours < 10) {
        lcd.print(F("0"));
      }
      lcd.print(EHours);
    }

    // мигающее двоеточие, если не в режиме установки времени
    if (!(SetH || SetM || PrintYesNo || blink500ms))
    {
      lcd.print(F(" "));
    }
    else {
      lcd.print(F(":"));
    }

    if (SetM && SetEnd && blink500ms) {
      lcd.print(F("  "));
    }
    else {
      if (EMinutes < 10) {
        lcd.print(F("0"));
      }
      lcd.print(EMinutes);
    }
    lcd.print(F(" "));
  }

  if (PrintYesNo) {
    lcd.print(F("["));
    if (!(SetH || SetM || blink500ms))
    {
      lcd.print(F(" "));
    }
    else {
      if (SetYesNo)
      {
        lcd.print(F("Y"));
      }
      else {
        lcd.print(F("N"));
      }
    }
    lcd.print(F("]"));
  }

}

// ============================ Encoder interrupts =============================
// Interrupt on A changing state
void doEncoderA() {
  if ( rotating ) {
    delay (1) ;  // wait a little until the bouncing is done
  }
  // Test transition, did things really change?
  if ( digitalRead(encoderA) != A_set ) { // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set )
    {
      MenuTimeoutTimer = 10; //таймер таймаута, секунд
      if (BeepEnabled) {
        tone(BeepPin, 4000, 5);
      }
      encoderR = true;
      rotating = false;  // no more debouncing until loop() hits again
    }
  }
}
// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) {
    delay (1);
  }
  if ( digitalRead(encoderB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set ) {
      MenuTimeoutTimer = 10; //таймер таймаута, секунд
      if (BeepEnabled) {
        tone(BeepPin, 4000, 5);
      }
      encoderL = true;
      rotating = false;
    }
  }
}
// ============================ Timer0 interrupt =============================
// run every 500ms
void timerIsr()
{
  blink500ms = !blink500ms; // инверсия мерцающего бита
  if (blink500ms) {
    plus1sec = true; // ежесекундно взводится
    if (TstatTimer != 0) {
      TstatTimer --; // ежесекундный декремент этого таймера
    }
    if (MenuTimeoutTimer != 0) {
      MenuTimeoutTimer --; // ежесекундный декремент этого таймера
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void TimerXSetup(int X) {
  MenuTimeoutTimer = 10; //таймер таймаута, секунд
  lcd.clear();
  lcd.setCursor(0, 0); //инфо на LCD
  lcd.print(F("SETUP TIMER"));
  lcd.print(X + 1); // выводим номер таймера на LCD
  delay(200);
  Hours = Timer[X].SHours;
  Minutes = Timer[X].SMinutes;
  EHours = Timer[X].EHours;
  EMinutes = Timer[X].EMinutes;
  SetYesNo = Timer[X].Enabled;
  PrintYesNo = true;
  SetTimerTime(0, 1); // в позиции 0,1 - запрос ввода времени
  if (MenuTimeoutTimer != 0) {
    if (SetYesNo) // если при установке времени выбрано "Yes"
    {
      if (BeepEnabled) {
        tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
      }
      Timer[X].SHours = Hours;
      Timer[X].SMinutes = Minutes;
      Timer[X].EHours = EHours;
      Timer[X].EMinutes = EMinutes;
      Timer[X].Enabled = true;
      EEPROM.updateBlock(int(&TimerEE[X]), Timer[X]);

      MenuTimeoutTimer = 10; //таймер таймаута, секунд
      lcd.clear();
      lcd.setCursor(0, 0); //инфо на LCD
      lcd.print(F("Timer"));
      lcd.print(X + 1);
      lcd.print(F(" Temp. Set"));
      delay(200);
      do {
        lcd.setCursor(0, 1);
        if (blink500ms) {
          lcd.print(F("     "));
        }
        else {
          lcd.print(Timer[X].Temperature, 1);
          lcd.write(0x02); // значок градуса
        }
        rotating = true;  // reset the debouncer
        if (encoderR) {
          Timer[X].Temperature += 0.1;
          encoderR = false;
        }
        if (encoderL) {
          Timer[X].Temperature -= 0.1;
          encoderL = false;
        }
        Timer[X].Temperature = constrain(Timer[X].Temperature, -15, 15); // крайние значения
      }
      while ((digitalRead(encoderK) == 1) | (MenuTimeoutTimer == 0));

      if (MenuTimeoutTimer != 0) { // если после выбора температуры нажата кнопка энкодера
        EEPROM.updateBlock(int(&TimerEE[X]), Timer[X]);
        if (BeepEnabled) {
          tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
        }
      }
      else { // если не нажата - используем старую температуру
        EEPROM.readBlock(int(&TimerEE[X]), Timer[X]);
        if (BeepEnabled) {
          tone(BeepPin, BeepToneYes, BeepToneYesDuration); //звук "YES"
        }
      }

    }
    else // если при установке времени выбрано "No"
    {
      if (BeepEnabled) {
        tone(BeepPin, BeepToneNo, BeepToneNoDuration); //звук "NO"
      }
      EEPROM.readBlock(int(&TimerEE[X]), Timer[X]);
      Timer[X].Enabled = false;
      EEPROM.updateBlock(int(&TimerEE[X]), Timer[X]);
    }
  }
  else {
    if (BeepEnabled) {
      tone(BeepPin, BeepToneNo, BeepToneNoDuration); //звук "NO"
    }
  }
}
