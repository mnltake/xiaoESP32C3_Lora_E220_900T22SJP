#include <Arduino.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <EEPROM.h>

// Set serial for debug console (to the Serial Monitor)
#define SerialMon Serial
// Set serial for LoRa (to the module)
#define SerialLoRa Serial1
// #define LTEGW 
#define WIFIGW 
// #define PCB
// #define MINI

//           ┌--4.7kΩ--┐
// L1 H1 COM DQ・GND　3V3
//  XH7pin
// #define DS18B20

//I2C 
#define I2C_DEV_ADDR 0x55

// ピン設定
#ifdef PCB
  #define LoRa_ModeSettingPin_M0 GPIO_NUM_2//D0 =GPIO2
  #define LoRa_ModeSettingPin_M1 GPIO_NUM_3//D1 =GPIO3
  #define LoRa_Rx_ESP_TxPin D6
  #define LoRa_Tx_ESP_RxPin D7
  #define LoRa_AUXPin GPIO_NUM_4//D2
  #define L1 D9
  #define H1 D10
  #define I2C_SDA D4
  #define I2C_SCL D5
  #define L2 D3
  #define H2 D8
  #define ONEWIRE_GND D5
  #define ONEWIRE_3V3 D8
  #define ONEWIRE_DQ D3

#elif defined(MINI)
  #define LoRa_ModeSettingPin_M0 GPIO_NUM_0
  #define LoRa_ModeSettingPin_M1 GPIO_NUM_1
  #define LoRa_Rx_ESP_TxPin GPIO_NUM_2
  #define LoRa_Tx_ESP_RxPin GPIO_NUM_3
  #define LoRa_AUXPin GPIO_NUM_4
  #define L1 GPIO_NUM_7
  #define H1 GPIO_NUM_6
  #define SW_COM GPIO_NUM_8
  #define I2C_SDA GPIO_NUM_10
  #define I2C_SCL GPIO_NUM_20
  #define L2 GPIO_NUM_10
  #define H2 GPIO_NUM_21
  #define ONEWIRE_GND GPIO_NUM_20
  #define ONEWIRE_3V3 GPIO_NUM_21
  #define ONEWIRE_DQ GPIO_NUM_9

#else
  #define LoRa_ModeSettingPin_M0 GPIO_NUM_20//D7
  #define LoRa_ModeSettingPin_M1 GPIO_NUM_20//D7
  #define LoRa_Rx_ESP_TxPin D8
  #define LoRa_Tx_ESP_RxPin D9
  #define LoRa_AUXPin D10
  #define L1 D0
  #define H1 D1
  #define SW_COM D2
  #define I2C_SDA D4
  #define I2C_SCL D5
  #define L2 D3
  #define H2 D6
  #define ONEWIRE_GND D5
  #define ONEWIRE_3V3 D6
  #define ONEWIRE_DQ D3
#endif


// E220-900T22S(JP)のbaud rate
#define LoRa_BaudRate 9600
#define wdtTimeout 60000

RTC_DATA_ATTR int16_t senserID = 0;
RTC_DATA_ATTR int16_t senserID_2nd = 0;
RTC_DATA_ATTR uint16_t bootCount = 0;
uint16_t waitmillsec = 1000*55;//センサーごとに変える
uint64_t sleepSec = 20*60;
esp_sleep_source_t  wakeup_reason;

#ifdef LTEGW
uint8_t loraChannel = 0x09;
uint8_t conf[] ={0xc0, 0x00, 0x08, 
                senserID >> 8, //ADDH
                senserID & 0xff, //ADDL
                0b01110000, // baud_rate 9600 bps  SF:9 BW:125
                0b11100001, //subpacket_size 32, rssi_ambient_noise_flag on, transmitting_power 13 dBm
                loraChannel, //own_channel
                0b11000101, //RSSI on ,fix mode,wor_cycle 3000 ms
                0x00, //CRYPT
                0x00};
#endif
#ifdef WIFIGW
uint8_t loraChannel = 0x00;
uint8_t conf[] ={0xc0, 0x00, 0x08, 
                senserID >> 8, //ADDH
                senserID & 0xff, //ADDL
                0b01110000, // baud_rate 9600 bps  SF:9 BW:125
                0b11100001, //subpacket_size 32, rssi_ambient_noise_flag on, transmitting_power 13 dBm
                loraChannel, //own_channel
                0b11000111, //RSSI on ,fix mode,wor_cycle 4000 ms
                0x00, //CRYPT
                0x00};
#endif

#ifdef DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
  OneWire oneWire(ONEWIRE_DQ);
  DallasTemperature sensors(&oneWire);
#endif

//WDT
#include "esp_system.h"
hw_timer_t *timer = NULL;

struct  __attribute__((packed, aligned(4))) msgStruct{ 
  char targetAdressH = 0x00;//GateWay adress 0x0000
  char targetAdressL = 0x00;
  char targetChannel = loraChannel;
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
   * @brief WOR受信モード(M0=0,M1=1)へ移行する
   */
  void SwitchToWORReceivingMode(void){
  digitalWrite(LoRa_ModeSettingPin_M0, 0);
  digitalWrite(LoRa_ModeSettingPin_M1, 1);
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

#define CHANNEL 1
// ESP-NOW初期化
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNow Init Failed. Restarting...");
    ESP.restart();
  }
}
// データ送信処理
void SendDataToLoRa(const uint8_t *payload, size_t len) {
  SerialLoRa.write(payload, len);
  SerialLoRa.flush();
  delay(100);
}
// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}
// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
  EEPROM.write(0, data[0]);  //ADDH
  EEPROM.write(1, data[1]);  //ADDL
  EEPROM.write(2, data[2]);  //ADDH_2nd
  EEPROM.write(3, data[3]);  //ADDL_2nd
  EEPROM.commit();
  senserID = data[0]<<8 | data[1];
  senserID_2nd = data[2]<<8 | data[3];
  SerialMon.printf("change sensorID: %d\n",senserID);
  SerialMon.printf("change sensorID_2nd: %d\n",senserID_2nd);
  return ;
}

float getTemp(){
  #ifdef  DS18B20
    pinMode( ONEWIRE_GND ,OUTPUT);
    pinMode( ONEWIRE_3V3 ,OUTPUT);
    digitalWrite ( ONEWIRE_GND ,LOW);
    digitalWrite (ONEWIRE_3V3 ,HIGH);
    sensors.begin();
    delay(10);
    sensors.requestTemperatures(); 
    Serial.print("Temperature:");
    Serial.println(sensors.getTempCByIndex(0));
    return sensors.getTempCByIndex(0);
  #endif

  return -127;
}

void IRAM_ATTR deep_sleep(){
    #ifdef LTEGW
      SwitchToWORReceivingMode();
      delay(100);
      esp_deep_sleep_enable_gpio_wakeup(BIT(4), ESP_GPIO_WAKEUP_GPIO_LOW);
    #endif
    #ifdef WIFIGW
      SwitchToConfigurationMode();
      if (bootCount < 10) {
        sleepSec = 25;
      }
     	esp_sleep_enable_timer_wakeup(sleepSec * 1000 * 1000 - micros());
    #endif
    gpio_hold_en(LoRa_ModeSettingPin_M0);
    gpio_hold_en(LoRa_ModeSettingPin_M1);
    gpio_deep_sleep_hold_en();
    Serial.println();
    Serial.println("Going to sleep now");
    delay(1000);
    // while(!digitalRead(LoRa_AUXPin)){};
    esp_deep_sleep_start();
}

// 起床理由の確認
void PrintWakeupReason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  const char *reason_str[] = {
    "Undefined", "All", "EXT0", "EXT1", "Timer",
    "Touchpad", "ULP", "GPIO", "UART", "WiFi",
    "CoCPU", "CoCPU Trap", "BT"
  };
  Serial.printf("Wakeup reason: %s\n", reason < 13 ? reason_str[reason] : "Unknown");
}

void setup() {
  #ifndef PCB
    digitalWrite ( SW_COM ,LOW);
    pinMode( SW_COM ,OUTPUT);
  #endif
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &deep_sleep, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  // timerAlarmEnable(timer);                          //enable interrupt
  timerWrite(timer, 0);
  SerialMon.begin(115200);
  delay(500);
    // E220-900T22S(JP)へのLoRa初期設定
  SerialLoRa.end(); // end()を実行　←←追加
  delay(1000); // 1秒待つ　 ←←追加
  SerialLoRa.begin(LoRa_BaudRate, SERIAL_8N1, LoRa_Tx_ESP_RxPin,LoRa_Rx_ESP_TxPin);
  EEPROM.begin(4);
  PrintWakeupReason();
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if (ESP_SLEEP_WAKEUP_GPIO  == wakeup_reason) {
    Serial.println("Waked up from external GPIO!");
    gpio_hold_dis(LoRa_ModeSettingPin_M0);
    gpio_hold_dis(LoRa_ModeSettingPin_M1);
    gpio_deep_sleep_hold_dis();
    SwitchToConfigurationMode();//boot text clear
    SwitchToNormalMode();
    SerialMon.printf("\n sensorID: %d\n",senserID);
    // msg.myadress =  senserID;
    delay(100);
  }else if (ESP_SLEEP_WAKEUP_TIMER == wakeup_reason) {
    Serial.println("Waked up from TIMER!");
    gpio_hold_dis(LoRa_ModeSettingPin_M0);
    gpio_hold_dis(LoRa_ModeSettingPin_M1);
    gpio_deep_sleep_hold_dis();

    SwitchToConfigurationMode();//boot text clear
    SwitchToNormalMode();
    delay(100);

  }else{
    Serial.println("Waked up from nomal power on!");
    //Set device in AP mode to begin with
    WiFi.mode(WIFI_AP);
    configDeviceAP();
    // // This is the mac address of the Slave in AP Mode
    // Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    // // Init ESPNow with a fallback logic
    InitESPNow();
    esp_now_register_recv_cb(OnDataRecv);
    delay(20000);
	  WiFi.enableSTA(false);
    senserID = (EEPROM.read(0) << 8) | EEPROM.read(1); 
    senserID_2nd = (EEPROM.read(2) << 8) | EEPROM.read(3); 
    // senserID = OWN_ADDRESS; 
    SerialMon.printf("\n sensorID: %d\n",senserID);
    // msg.myadress =  senserID;
    
    SwitchToConfigurationMode();
    // while(!digitalRead(LoRa_AUXPin)){}
    SerialMon.printf("I send conf\r\n");
    for (size_t i = 0; i < sizeof(conf); i++)
    {
      
      SerialMon.printf(" %02x",conf[i]);
    }
    SerialLoRa.write((uint8_t *)&conf, sizeof(conf));

    delay(100);
    // while(!digitalRead(LoRa_AUXPin)){}
    deep_sleep();
  }

  Serial.println("Wake and start ");
  pinMode( L1 ,INPUT_PULLUP);
  pinMode( H1 ,INPUT_PULLUP);
  pinMode( L2 ,INPUT_PULLUP);
  pinMode( H2 ,INPUT_PULLUP);

  #ifdef LTEGW
  delay(waitmillsec);//他と重ならない秒数
  SerialMon.printf("waitmillsec: %d\n",waitmillsec);
  #endif

  SerialMon.printf("sensorID: %d\n",senserID);
  msg.myadress =  senserID ;
  msg.temp = getTemp();
  byte temperatureByteData[sizeof(float)];
  memcpy(temperatureByteData, &msg.temp, sizeof(float));
  // msg.temp = -127;
  msg.water = digitalRead( L1 ) * 49 + digitalRead( H1 ) * 51; //ここに水位
  msg.bootcount = bootCount;
  Serial.printf("boot:%d \nWater:%d \nTemp:%f\n" ,msg.bootcount,msg.water,msg.temp);
  SerialLoRa.flush();

  uint8_t payload[]={msg.targetAdressH, msg.targetAdressL, msg.targetChannel ,

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
  SendDataToLoRa(payload, sizeof(payload));

  if (senserID_2nd){
    timerWrite(timer, 0);
    delay(2000);
    digitalWrite ( I2C_SCL,LOW);
    pinMode( I2C_SCL ,OUTPUT);
    msg.myadress = senserID_2nd;
    msg.temp = getTemp();
    // msg.temp = -127;
    msg.water = digitalRead( L2 ) * 49 + digitalRead( H2 ) * 51; //ここに水位
    msg.bootcount = bootCount;
    SerialMon.println(senserID_2nd);
    SerialMon.printf("boot:%d \nWater:%d \nTemp:%f\n" ,msg.bootcount,msg.water,msg.temp);
    SerialLoRa.flush();
    uint8_t payload2[]={msg.targetAdressH, msg.targetAdressL, msg.targetChannel ,

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
    SendDataToLoRa(payload, sizeof(payload));
  }

  bootCount++;
  deep_sleep();
}

void loop() {
}

