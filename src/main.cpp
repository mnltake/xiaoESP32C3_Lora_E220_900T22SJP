#include "esp32_e220900t22s_jp_lib.h"
#include <Arduino.h>
#include <FS.h>

#define OWN_ADDRESS 158
// #define SECOND_ADDRESS 
#define SW_LOW D0
#define SW_HIGH D1
#define SW_COM D2
#define SECOND_SW_LOW D3
#define SECOND_SW_HIGH D4
#define SECOND_SW_COM D5
uint64_t sleepSec = 60*60 - 5;//実行時間5ｓ
RTC_DATA_ATTR uint16_t bootCount = 0;
CLoRa lora;
struct LoRaConfigItem_t config = {
      OWN_ADDRESS, // own_address 0
      0b011, // baud_rate 9600 bps
      0b10000, // air_data_rate SF:9 BW:125
      0b11, // subpacket_size 200
      0b1, // rssi_ambient_noise_flag 有効
      0b0, // transmission_pause_flag 有効
      0b00, // transmitting_power 13 dBm
      0x00, // own_channel 0
      0b1, // rssi_byte_flag 有効
      0b0, // transmission_method_type トランスペアレント送信モード(default)
      0b0, // lbt_flag 有効
      0b011, // wor_cycle 2000 ms
      0x0000, // encryption_key 0
      0xFFFF, // target_address 0
      0x00}; // target_channel 0

struct RecvFrameE220900T22SJP_t data;

/** prototype declaration **/
void LoRaRecvTask(void *pvParameters);
void LoRaSendTask(void *pvParameters);
void ReadDataFromConsole(char *msg, int max_msg_len);

//WDT
#include "esp_system.h"


//DS18B20
// #include <OneWire.h>
// #include <DallasTemperature.h>
// OneWire oneWire(SENSOR_DQ);
// DallasTemperature sensors(&oneWire);

struct  __attribute__((packed, aligned(4))) msgStruct{ 
  uint16_t myadress ;
  uint16_t water ;
  uint16_t bootcount;
  float temp  ;
} msg;

const int wdtTimeout = 30*1000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

float getTemp(){
  // digitalWrite (SENSOR_3V3 ,HIGH);
  // delay(10);
  // sensors.requestTemperatures(); 
  // Serial.print("Temperature:");
  // Serial.println(sensors.getTempCByIndex(0));
  // return sensors.getTempCByIndex(0);
  return -127;
}

void IRAM_ATTR deep_sleep(){
        SerialMon.printf("sleep \n");
        lora.SwitchToConfigurationMode();
        if (bootCount == 0){
          delay(10000);
        } 
        if (bootCount < 10)
        {
          sleepSec = 25;
        }
        
        bootCount++;

        // digitalWrite(LED1 ,LOW);
        esp_sleep_enable_timer_wakeup(sleepSec * 1000 * 1000);
      	esp_deep_sleep_start();
}

void setup() {
  // put your setup code here, to run once:
  SerialMon.begin(9600);
  delay(500);
  // while(!SerialMon){}; // SerialMon init wait
  SerialMon.println("start");
  SerialMon.println(OWN_ADDRESS);
  pinMode( SW_LOW ,INPUT_PULLUP);
  pinMode( SW_HIGH ,INPUT_PULLUP);
  pinMode( SW_COM ,OUTPUT);
  digitalWrite ( SW_COM ,LOW);
  #ifdef SECOND_ADDRESS
    pinMode( SECOND_SW_LOW ,INPUT_PULLUP);
    pinMode( SECOND_SW_HIGH ,INPUT_PULLUP);
    pinMode( SECOND_SW_COM ,OUTPUT);
    digitalWrite ( SECOND_SW_COM ,LOW);
  #endif
  

  // sensors.begin();
  // LoRa設定値の読み込み
  
  // if (lora.LoadConfigSetting(CONFIG_FILENAME, config)) {
  //   SerialMon.printf("Loading Configfile failed. The default value is set.\n");
  // } else {
  //   SerialMon.printf("Loading Configfile succeeded.\n");
  // }

  // E220-900T22S(JP)へのLoRa初期設定
  if (lora.InitLoRaModule(config)) {
    SerialMon.printf("init error\n");
    return;
  }

  // ノーマルモード(M0=0,M1=0)へ移行する
  lora.SwitchToNormalMode();
  while(!digitalRead(LoRa_AUXPin)){}
  // timer = timerBegin(0, 80, true);                  //timer 0, div 80
  // timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  // timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  // timerAlarmEnable(timer);                          //enable interrupt
  // マルチタスク
  // xTaskCreateUniversal(LoRaRecvTask, "LoRaRecvTask", 8192, NULL, 1, NULL,
  //                      1);
  // xTaskCreateUniversal(LoRaSendTask, "LoRaSendTask", 8192, NULL, 1, NULL,
  //                      1);
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &deep_sleep, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
  timerWrite(timer, 0);
  msg.myadress = config.own_address;
  msg.temp = getTemp();
  // msg.temp = -127;
  msg.water = digitalRead( SW_LOW) * 49 + digitalRead( SW_HIGH) * 51; //ここに水位
  msg.bootcount = bootCount;
  SerialMon.printf("boot:%d \nWater:%d \nTemp:%f\n" ,msg.bootcount,msg.water,msg.temp);
  SerialLoRa.flush();
  SerialMon.printf("I send data\n");
  if (lora.SendFrame(config, (uint8_t *)&msg, sizeof(msg)) == 0) {
    delay(500);
    SerialMon.printf("send succeeded.\n");
    SerialMon.printf("\n");
  } else {
    SerialMon.printf("send failed.\n");
    SerialMon.printf("\n");
  }
  
  #ifdef SECOND_ADDRESS
    timerWrite(timer, 0);
    delay(5000);
    SerialMon.println(SECOND_ADDRESS);
    msg.myadress = SECOND_ADDRESS;
    msg.temp = getTemp();
    // msg.temp = -127;
    msg.water = digitalRead( SECOND_SW_LOW) * 49 + digitalRead( SECOND_SW_HIGH) * 51; //ここに水位
    msg.bootcount = bootCount;
    SerialMon.printf("boot:%d \nWater:%d \nTemp:%f\n" ,msg.bootcount,msg.water,msg.temp);
    SerialLoRa.flush();
    SerialMon.printf("I send data\n");
    if (lora.SendFrame(config, (uint8_t *)&msg, sizeof(msg)) == 0) {
      delay(500);
      SerialMon.printf("send succeeded.\n");
      SerialMon.printf("\n");
    } else {
      SerialMon.printf("send failed.\n");
      SerialMon.printf("\n");
    }
  #endif

  deep_sleep();
}

void loop() {

}

// void LoRaRecvTask(void *pvParameters) {
//   while (1) {
//     if (lora.RecieveFrame(&data) == 0) {
//       digitalWrite(LED1 ,HIGH);
//       delay(500);
//       digitalWrite(LED1 ,LOW);
//       SerialMon.printf("from ATOM recv data:\n");
//       for (int i = 0; i < data.recv_data_len; i++) {
//         SerialMon.printf("%02x", data.recv_data[i]);
//       }
//       SerialMon.printf("\n");
//       SerialMon.printf("hex dump:\n");
//       for (int i = 0; i < data.recv_data_len; i++) {
//         SerialMon.printf("%02x ", data.recv_data[i]);
//       }
//       SerialMon.printf("\n");
//       SerialMon.printf("RSSI: %d dBm\n", data.rssi);
//       SerialMon.printf("\n");

//       SerialMon.flush();
      
//       if ((data.recv_data[4]<<8 |data.recv_data[5]) == config.own_address){
//         SerialMon.printf("response ok \n");
//         delay(100);
//         deep_sleep();
//       }
//     }

//     delay(1);
//   }
// }

// void LoRaSendTask(void *pvParameters) {
//   while (1) {
//     // lora.SwitchToNormalMode();
//     msg.bootcount = bootCount;
//     msg.myadress = config.own_address;
//     msg.temp = getTemp();
//     msg.water = bootCount % 100;//ここに水位
//     SerialLoRa.flush();
//     SerialMon.printf("I send data\n");
//     digitalWrite(LED0 ,HIGH);
//     if (lora.SendFrame(config, (uint8_t *)&msg, sizeof(msg)) == 0) {
//       delay(500);
//       SerialMon.printf("send succeeded.\n");
//       digitalWrite(LED0 ,LOW);
//       SerialMon.printf("\n");
//     } else {
//       SerialMon.printf("send failed.\n");
//       SerialMon.printf("\n");
//     }

//     // SerialMon.flush();
//     // lora.SwitchToConfigurationMode();
    
//     delay(30000);
//     deep_sleep();
//   }
// }
