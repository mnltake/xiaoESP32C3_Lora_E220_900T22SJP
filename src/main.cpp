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
#define LoRa_ModeSettingPin_M0 GPIO_NUM_2//D0 =GPIO2
#define LoRa_ModeSettingPin_M1 GPIO_NUM_3//D1 =GPIO3
#define LoRa_Rx_ESP_TxPin D8
#define LoRa_Tx_ESP_RxPin D9
#define LoRa_AUXPin GPIO_NUM_4//D2

// E220-900T22S(JP)のbaud rate
#define LoRa_BaudRate 9600

int16_t senserID = -1;
#define OWN_ADDRESS 131
// #define SECOND_ADDRESS 305

#define SW_LOW D4
#define SW_HIGH D5
#define SW_COM D6
// #define I2C_VCC D3
// #define I2C_SDA D4
// #define I2C_SCL D5
#define SECOND_SW_LOW D3
// #define SECOND_SW_HIGH D6
// #define SECOND_SW_COM D5
// #define SENSOR_GND D5
// #define SENSOR_3V3 D6
// #define SENSOR_DQ D3
uint64_t sleepSec = 60*60 - 5;//実行時間5ｓ
RTC_DATA_ATTR uint16_t bootCount = 0;
esp_sleep_source_t  wakeup_reason;
//WDT
#include "esp_system.h"
const int wdtTimeout = 60*1000;  //time in ms to trigger the watchdog
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
  char targetAdressH = 0x00;//GateWay adress 0x0000
  char targetAdressL = 0x00;
  char targetChannel = 0x09;
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
void onReceive(int len){
  byte buf[2];
  int j =0;
  while(Wire.available()){
    buf[j]=Wire.read();
    SerialMon.printf("%02x ",buf[j]);
    j++;
  }
    EEPROM.write(0, buf[0]);  //ADDH
    EEPROM.write(1, buf[1]);  //ADDL
    EEPROM.commit();
  senserID = buf[0]<<8 | buf[1];
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
    SwitchToWORReceivingMode();
		delay(100);
    Serial.println();
		if (ESP_OK == gpio_hold_en(LoRa_ModeSettingPin_M0)){
			Serial.println("HOLD LoRa_ModeSettingPin_M0");
		}else{
			Serial.println("NO HOLD LoRa_ModeSettingPin_M0");
		}
		if (ESP_OK == gpio_hold_en(LoRa_ModeSettingPin_M1)){
				Serial.println("HOLD LoRa_ModeSettingPin_M1");
			}else{
				Serial.println("NO HOLD 1LoRa_ModeSettingPin_M1");
			}
    if(esp_sleep_is_valid_wakeup_gpio(LoRa_AUXPin)){
      esp_deep_sleep_enable_gpio_wakeup(BIT(4), ESP_GPIO_WAKEUP_GPIO_LOW);
      gpio_deep_sleep_hold_en();
      //Go to sleep now
      Serial.println("Going to sleep now");
      delay(1000);
      esp_deep_sleep_start();
    }else{
      Serial.println("NG pin");
    }

}
void wakeup_cause_print() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case 0: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_UNDEFINED"); break;
    case 1: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_ALL"); break;
    case 2: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_EXT0"); break;
    case 3: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_EXT1"); break;
    case 4: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_TIMER"); break;
    case 5: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_TOUCHPAD"); break;
    case 6: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_ULP"); break;
    case 7: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_GPIO"); break;
    case 8: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_UART"); break;
    case 9: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_WIFI"); break;
    case 10: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_COCPU"); break;
    case 11: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG"); break;
    case 12: Serial.println("Wakeup caused by ESP_SLEEP_WAKEUP_BT"); break;
    default: Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}
void setup() {
  // put your setup code here, to run once:
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &deep_sleep, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
  timerWrite(timer, 0);
  SerialMon.begin(9600);
  delay(500);
    // E220-900T22S(JP)へのLoRa初期設定
  SerialLoRa.end(); // end()を実行　←←追加
  delay(1000); // 1秒待つ　 ←←追加
  SerialLoRa.begin(LoRa_BaudRate, SERIAL_8N1, LoRa_Tx_ESP_RxPin,LoRa_Rx_ESP_TxPin);
  EEPROM.begin(2);
  wakeup_cause_print();
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if (ESP_SLEEP_WAKEUP_GPIO  == wakeup_reason) {
    Serial.println("Waked up from external GPIO!");
    gpio_hold_dis(LoRa_ModeSettingPin_M0);
    gpio_hold_dis(LoRa_ModeSettingPin_M1);
    gpio_deep_sleep_hold_dis();
    SwitchToNormalMode();
    senserID = OWN_ADDRESS; 
    SerialMon.printf("\n sensorID: %d\n",senserID);
    msg.myadress =  senserID;
    delay(100);
  }else{
    Serial.println("Waked up from nomal power on!");
    // Wire.begin((uint8_t)I2C_DEV_ADDR, I2C_SDA, I2C_SCL, 100000);
    // Wire.onReceive(onReceive);
    delay(10000);

    // senserID = (EEPROM.read(0) << 8) | EEPROM.read(1); 
    senserID = OWN_ADDRESS; 
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
    deep_sleep();
  }
//	e32ttl.setMode(MODE_0_NORMAL);
//	delay(1000);
  Serial.println("Wake and start ");
  pinMode( SW_LOW ,INPUT_PULLUP);
  pinMode( SW_HIGH ,INPUT_PULLUP);
  pinMode( SW_COM ,OUTPUT);
  digitalWrite ( SW_COM ,LOW);
  // pinMode( I2C_VCC ,OUTPUT);
  // digitalWrite ( I2C_VCC ,HIGH);
  // pinMode( SENSOR_3V3 ,OUTPUT);
  // pinMode( SENSOR_GND ,OUTPUT);
  // digitalWrite ( SENSOR_GND ,LOW);

  #ifdef SECOND_ADDRESS
    pinMode( SECOND_SW_LOW ,INPUT_PULLUP);
    pinMode( SECOND_SW_HIGH ,INPUT_PULLUP);
    pinMode( SECOND_SW_COM ,OUTPUT);
    digitalWrite ( SECOND_SW_COM ,LOW);
  #endif
  

  SerialLoRa.flush();
  // ノーマルモード(M0=0,M1=0)へ移行する
  SwitchToNormalMode();
  while(!digitalRead(LoRa_AUXPin)){}


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
    SerialLoRa.write((uint8_t *)&payload2, sizeof(payload2));
    SerialLoRa.flush();
    delay(100);
  #endif

 
}

void loop() {
  for(int j=0 ;j<10;j++){
  delay(3000);
  // senserID = (EEPROM.read(0) >> 8) | EEPROM.read(1); 
  SerialMon.printf("sensorID: %d\n",senserID);
  msg.myadress =  senserID +j;
  msg.temp = getTemp();
  byte temperatureByteData[sizeof(float)];
  memcpy(temperatureByteData, &msg.temp, sizeof(float));
  // msg.temp = -127;
  msg.water = digitalRead( SW_LOW) * 49 + digitalRead( SW_HIGH) * 51; //ここに水位
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
  SerialLoRa.write((uint8_t *)&payload, sizeof(payload));
  SerialLoRa.flush();
  delay(100);
  }
  bootCount++;
  deep_sleep();
}

