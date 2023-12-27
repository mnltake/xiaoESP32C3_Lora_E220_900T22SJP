#include <Arduino.h>
#include "Wire.h"
#include <EEPROM.h>
// Set serial for debug console (to the Serial Monitor)
#define SerialMon Serial
// Set serial for LoRa (to the module)
#define SerialLoRa Serial1

//I2C 
#define I2C_DEV_ADDR 0x55

// E220-900T22S(JP)へのピンアサイン
#define LoRa_ModeSettingPin_M0 D7
#define LoRa_ModeSettingPin_M1 D7
#define LoRa_Rx_ESP_TxPin D8
#define LoRa_Tx_ESP_RxPin D9
#define LoRa_AUXPin D10

// E220-900T22S(JP)のbaud rate
#define LoRa_BaudRate 9600

int16_t senserID = -1;
#define OWN_ADDRESS 131
// #define SECOND_ADDRESS 305

#define SW_LOW D0
#define SW_HIGH D1
#define SW_COM D2
// #define I2C_VCC D3
#define I2C_SDA D4
#define I2C_SCL D5
#define SECOND_SW_LOW D3
#define SECOND_SW_HIGH D6
// #define SECOND_SW_COM D5
// #define SENSOR_GND D5
// #define SENSOR_3V3 D6
// #define SENSOR_DQ D3
uint64_t sleepSec = 60*60 - 5;//実行時間5ｓ
RTC_DATA_ATTR uint16_t bootCount = 0;

//WDT
#include "esp_system.h"
const int wdtTimeout = 30*1000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

//DS18B20
// #include <OneWire.h>
// #include <DallasTemperature.h>
// OneWire oneWire(SENSOR_DQ);
// DallasTemperature sensors(&oneWire);

uint8_t conf[] ={0xc0, 0x00, 0x08, 
                OWN_ADDRESS >> 8, //ADDH
                OWN_ADDRESS & 0xff, //ADDL
                0b01110000, // baud_rate 9600 bps  SF:9 BW:125
                0b11100000, //subpacket_size 32, rssi_ambient_noise_flag on, transmitting_power 13 dBm
                0x09, //own_channel
                0b10000011, //RSSI on ,fix mode,wor_cycle 2000 ms
                0x00, //CRYPT
                0x00};
struct  __attribute__((packed, aligned(4))) msgStruct{ 
  char conf_0 = 0xFF;
  char conf_1 = 0xFF;
  char channel = 0x09;
  uint16_t myadress ;
  uint16_t water ;
  uint16_t bootcount;
  float temp  ;
} msg;


  /**
   * @brief ノーマルモード(M0=0,M1=0)へ移行する
   */
void SwitchToNormalMode(void){
  digitalWrite(LoRa_ModeSettingPin_M0, 0);
  digitalWrite(LoRa_ModeSettingPin_M1, 0);
  delay(100);
}


  /**
   * @brief コンフィグ/sleepモード(M0=1,M1=1)へ移行する
   */
void SwitchToConfigurationMode(void){
  digitalWrite(LoRa_ModeSettingPin_M0, 1);
  digitalWrite(LoRa_ModeSettingPin_M1, 1);
  delay(100);
}
void onReceive(int len){
  byte buf[2];
  int j =0;
  while(Wire.available()){
    buf[j]=Wire.read();
    SerialMon.printf("%02x ",buf[j]);
    j++;
  }
    EEPROM.write(0, buf[0]);  
    EEPROM.write(1, buf[1]);  
    EEPROM.commit();
  senserID = buf[0]<<8|buf[1];
  SerialMon.printf("change sensorID: %d\n",senserID);
  return ;
}

void getSensorID(){


    return ;
}
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
        SwitchToConfigurationMode();
        // if (bootCount == 0){
        //   delay(10000);
        // } 
        if (bootCount < 10)
        {
          sleepSec = 25;
        }
        bootCount++;
        esp_sleep_enable_timer_wakeup(sleepSec * 1000 * 1000);
        // gpio_hold_en(GPIO_NUM_20);
        // gpio_deep_sleep_hold_en();
      	esp_deep_sleep_start();
}

void setup() {
  // put your setup code here, to run once:
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &deep_sleep, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
  timerWrite(timer, 0);
  SerialMon.begin(9600);
  delay(500);
  // gpio_hold_dis(GPIO_NUM_20);
  // gpio_deep_sleep_hold_dis();
  // while(!SerialMon){}; // SerialMon init wait
  SerialMon.println("start");
  SerialMon.println(OWN_ADDRESS);
  pinMode( SW_LOW ,INPUT_PULLUP);
  pinMode( SW_HIGH ,INPUT_PULLUP);
  pinMode( SW_COM ,OUTPUT);
  digitalWrite ( SW_COM ,LOW);
  // pinMode( I2C_VCC ,OUTPUT);
  // digitalWrite ( I2C_VCC ,HIGH);
  // pinMode( SENSOR_3V3 ,OUTPUT);
  // pinMode( SENSOR_GND ,OUTPUT);
  // digitalWrite ( SENSOR_GND ,LOW);
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);
  #ifdef SECOND_ADDRESS
    pinMode( SECOND_SW_LOW ,INPUT_PULLUP);
    pinMode( SECOND_SW_HIGH ,INPUT_PULLUP);
    pinMode( SECOND_SW_COM ,OUTPUT);
    digitalWrite ( SECOND_SW_COM ,LOW);
  #endif
  
  // E220-900T22S(JP)へのLoRa初期設定
  SerialLoRa.end(); // end()を実行　←←追加
  delay(1000); // 1秒待つ　 ←←追加
  SerialLoRa.begin(LoRa_BaudRate, SERIAL_8N1, LoRa_Tx_ESP_RxPin,LoRa_Rx_ESP_TxPin);

  EEPROM.begin(2);
  if(!bootCount){
    Wire.begin((uint8_t)I2C_DEV_ADDR, I2C_SDA, I2C_SCL, 100000);
    Wire.onReceive(onReceive);
    delay(20000);

    senserID = (EEPROM.read(0) >> 8) | EEPROM.read(1); 
    SerialMon.printf("\n sensorID: %d\n",senserID);
    msg.myadress =  senserID;
    
    SwitchToConfigurationMode();
    while(!digitalRead(LoRa_AUXPin)){}
    SerialMon.printf("I send conf\r\n");
    for (size_t i = 0; i < sizeof(conf); i++)
    {
      
      SerialMon.printf(" %02x",conf[i]);
    }
    SerialLoRa.write((uint8_t *)&conf, sizeof(conf));

    delay(100);
    while(!digitalRead(LoRa_AUXPin)){}

  }
  SerialLoRa.flush();
  // ノーマルモード(M0=0,M1=0)へ移行する
  SwitchToNormalMode();
  while(!digitalRead(LoRa_AUXPin)){}
  senserID = (EEPROM.read(0) >> 8) | EEPROM.read(1); 
  SerialMon.printf("sensorID: %d\n",senserID);
  msg.myadress =  senserID;
  msg.temp = getTemp();
  byte temperatureByteData[sizeof(float)];
  memcpy(temperatureByteData, &msg.temp, sizeof(float));
  // msg.temp = -127;
  msg.water = digitalRead( SW_LOW) * 49 + digitalRead( SW_HIGH) * 51; //ここに水位
  msg.bootcount = bootCount;
  Serial.printf("boot:%d \nWater:%d \nTemp:%f\n" ,msg.bootcount,msg.water,msg.temp);
  SerialLoRa.flush();

  uint8_t payload[]={msg.conf_0, msg.conf_1, msg.channel ,

                    msg.myadress & 0xff ,msg.myadress >> 8 ,
                    msg.water &0xff, 0x00,
                    msg.bootcount & 0xff, msg.bootcount >> 8, 
                    temperatureByteData[0],temperatureByteData[1],temperatureByteData[2],temperatureByteData[3],
                    0x00,0x00};
  
  SerialMon.printf("I send data\r\n");
  for (size_t i = 0; i < sizeof(payload); i++)
  {
    SerialMon.printf(" %02x",payload[i]);
  }
  SerialMon.println();
  SerialLoRa.write((uint8_t *)&payload, sizeof(payload));
  SerialLoRa.flush();
  delay(100);

  #ifdef SECOND_ADDRESS
    timerWrite(timer, 0);
    delay(5000);
    timerWrite(timer, 0);
    msg.myadress = SECOND_ADDRESS;
    msg.temp = getTemp();
    // msg.temp = -127;
    msg.water = digitalRead( SECOND_SW_LOW) * 49 + digitalRead( SECOND_SW_HIGH) * 51; //ここに水位
    msg.bootcount = bootCount;
    SerialMon.println(SECOND_ADDRESS);
    SerialMon.printf("boot:%d \nWater:%d \nTemp:%f\n" ,msg.bootcount,msg.water,msg.temp);
    SerialLoRa.flush();
    uint8_t payload2[]={msg.conf_0, msg.conf_1, msg.channel ,

                      msg.myadress & 0xff ,msg.myadress >> 8 ,
                      msg.water &0xff, 0x00, 
                      msg.bootcount & 0xff, msg.bootcount >> 8, 
                      0x00, 0x00 ,0xfe, 0xc2, 0x00, 0x00};
      SerialMon.printf("I send data\n\n");
      for (size_t i = 0; i < sizeof(payload2); i++)
    {

    SerialMon.printf(" %02x",payload2[i]);
    }
    SerialMon.println();
    SerialLoRa.write((uint8_t *)&payload2, sizeof(payload2));
    SerialLoRa.flush();
    delay(100);
  #endif

  deep_sleep();
}

void loop() {}
