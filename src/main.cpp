#include "esp32_e220900t22s_jp_lib.h"
#include <Arduino.h>
#include <FS.h>
// #include <SD.h>
// #include <SPI.h>
#define LED0 D0
#define LED1 D1
const int sleepSec = 30;
RTC_DATA_ATTR uint16_t bootCount = 254;
CLoRa lora;
struct LoRaConfigItem_t config;
struct RecvFrameE220900T22SJP_t data;

/** prototype declaration **/
void LoRaRecvTask(void *pvParameters);
void LoRaSendTask(void *pvParameters);
void ReadDataFromConsole(char *msg, int max_msg_len);

//WDT
#include "esp_system.h"


//DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(D2);
DallasTemperature sensors(&oneWire);

struct msgStruct{ 
  uint16_t bootcount;
  uint16_t myadress;
  float temp;
  uint16_t end = 0x0a0d;
} msg;



const int wdtTimeout = 30*1000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  // esp_restart();
}
float getTemp(){
  sensors.requestTemperatures(); 
  Serial.print("Temperature:");
  Serial.println(sensors.getTempCByIndex(0));
  return sensors.getTempCByIndex(0);
}
void deep_sleep(){
        SerialMon.printf("sleep \n");
        bootCount++;
        lora.SwitchToConfigurationMode();
        delay(3000);
        digitalWrite(LED0 ,LOW);
        digitalWrite(LED1 ,LOW);
        esp_sleep_enable_timer_wakeup(sleepSec * 1000 * 1000);
      	esp_deep_sleep_start();
}
void setup() {
  // put your setup code here, to run once:
  SerialMon.begin(9600);
  delay(2000); // SerialMon init wait
  SerialMon.println("start");
  pinMode( LED0 ,OUTPUT);
  pinMode( LED1 ,OUTPUT);
  sensors.begin();
  // LoRa設定値の読み込み
  
  if (lora.LoadConfigSetting(CONFIG_FILENAME, config)) {
    SerialMon.printf("Loading Configfile failed. The default value is set.\n");
  } else {
    SerialMon.printf("Loading Configfile succeeded.\n");
  }

  // E220-900T22S(JP)へのLoRa初期設定
  if (lora.InitLoRaModule(config)) {
    SerialMon.printf("init error\n");
    return;
  }

  // ノーマルモード(M0=0,M1=0)へ移行する
  lora.SwitchToNormalMode();

  // timer = timerBegin(0, 80, true);                  //timer 0, div 80
  // timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  // timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  // timerAlarmEnable(timer);                          //enable interrupt
  // マルチタスク
  xTaskCreateUniversal(LoRaRecvTask, "LoRaRecvTask", 8192, NULL, 1, NULL,
                       1);
  xTaskCreateUniversal(LoRaSendTask, "LoRaSendTask", 8192, NULL, 1, NULL,
                       1);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
  deep_sleep();
}

void LoRaRecvTask(void *pvParameters) {
  while (1) {
    if (lora.RecieveFrame(&data) == 0) {
      digitalWrite(LED1 ,HIGH);
      delay(500);
      digitalWrite(LED1 ,LOW);
      SerialMon.printf("from ATOM recv data:\n");
      for (int i = 0; i < data.recv_data_len; i++) {
        SerialMon.printf("%02x", data.recv_data[i]);
      }
      SerialMon.printf("\n");
      SerialMon.printf("hex dump:\n");
      for (int i = 0; i < data.recv_data_len; i++) {
        SerialMon.printf("%02x ", data.recv_data[i]);
      }
      SerialMon.printf("\n");
      SerialMon.printf("RSSI: %d dBm\n", data.rssi);
      SerialMon.printf("\n");

      SerialMon.flush();
      
      if ((data.recv_data[4]<<8 |data.recv_data[5]) == config.own_address){
        SerialMon.printf("response ok \n");
        delay(100);
        deep_sleep();
      }
    }

    delay(1);
  }
}

void LoRaSendTask(void *pvParameters) {
   char c=0x00;

  while (1) {
    // lora.SwitchToNormalMode();
    msg.bootcount = bootCount;
    msg.myadress = config.own_address;
    msg.temp = getTemp();


    // union ByteFloatUnion{
    //   uint8_t byteformat[4];
    //   float floatformat;
    // } temp;
  
    // temp.floatformat = getTemp();
    // delay(200);
    // char  msg[32] = {0};
    // // msg[0] = 0x00;
    // msg[0] = bootCount;
    // msg[1] = config.own_address>>8;
    // msg[2] = config.own_address&0xff;
    // msg[3] = 0x0A;
    // msg[4] = 0x0d;
    // msg[5] = temp.byteformat[0];
    // msg[6] = temp.byteformat[1];
    // msg[7] = temp.byteformat[2];
    // msg[8] = temp.byteformat[3];
    // msg[9] = 0x0A;
    // msg[10] = 0x0d;
// SerialMon.printf("%04x",config.own_address);
    SerialLoRa.flush();
    SerialMon.printf("I send data\n");
    digitalWrite(LED0 ,HIGH);
    if (lora.SendFrame(config, (uint8_t *)&msg, sizeof(msg)) == 0) {
      delay(500);
      SerialMon.printf("send succeeded.\n");
      digitalWrite(LED0 ,LOW);
      SerialMon.printf("\n");
    } else {
      SerialMon.printf("send failed.\n");
      SerialMon.printf("\n");
    }

    // SerialMon.flush();
    // lora.SwitchToConfigurationMode();
    
    delay(30000);
  }
}
