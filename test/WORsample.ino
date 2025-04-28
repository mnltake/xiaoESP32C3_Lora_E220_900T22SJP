#include <Arduino.h>
#define SerialMon Serial
#define SerialLoRa Serial1

// E220-900T22S(JP)へのピンアサイン
#define LoRa_ModeSettingPin_M0 GPIO_NUM_2//D0 
#define LoRa_ModeSettingPin_M1 GPIO_NUM_3//D1
#define LoRa_Rx_ESP_TxPin GPIO_NUM_21//D6
#define LoRa_Tx_ESP_RxPin GPIO_NUM_20//D7
#define LoRa_AUXPin GPIO_NUM_4//D2

// E220-900T22S(JP)のbaud rate
#define LoRa_BaudRate 9600
int16_t senserID = 0x0001;
uint64_t sleepSec = 60*60;//
RTC_DATA_ATTR uint16_t bootCount = 0;
esp_sleep_source_t  wakeup_reason;

uint8_t conf[] ={0xc0, 0x00, 0x08, 
                senserID >> 8, //ADDH
                senserID & 0xff, //ADDL
                0b01110000, // baud_rate 9600 bps  SF:9 BW:125
                0b11100000, //subpacket_size 32, rssi_ambient_noise_flag on, transmitting_power 13 dBm
                0x09, //own_channel
                0b10000101, //RSSI on ,fix mode,wor_cycle 3000 ms
                0x00, //CRYPT
                0x00};
struct  msgStruct{ 
  char targetAdressH = 0x00;//GateWay adress 0x0000
  char targetAdressL = 0x00;
  char targetChannel = 0x09;
  uint16_t myadress = senserID;
  uint16_t bootcount ;
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

void IRAM_ATTR deep_sleep(){
    SwitchToWORReceivingMode();
	delay(100);
    Serial.println();
    gpio_hold_en(LoRa_ModeSettingPin_M0);
    gpio_hold_en(LoRa_ModeSettingPin_M1);
    esp_deep_sleep_enable_gpio_wakeup(BIT(LoRa_AUXPin), ESP_GPIO_WAKEUP_GPIO_LOW);
    gpio_deep_sleep_hold_en();
      //Go to sleep now
    Serial.println("Going to sleep now");
    delay(1000);
    esp_deep_sleep_start();
}
void wakeup_cause_print(esp_sleep_source_t  wakeup_reason) {
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
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);
  SerialMon.begin(9600);
  delay(500);
  // E220-900T22S(JP)へのLoRa初期設定
  SerialLoRa.end(); 
  delay(1000); 
  SerialLoRa.begin(LoRa_BaudRate, SERIAL_8N1, LoRa_Tx_ESP_RxPin,LoRa_Rx_ESP_TxPin);
  wakeup_reason = esp_sleep_get_wakeup_cause();
  wakeup_cause_print(wakeup_reason);
  
  if (ESP_SLEEP_WAKEUP_GPIO  == wakeup_reason) {
    Serial.println("Waked up from external GPIO!");
    gpio_hold_dis(LoRa_ModeSettingPin_M0);
    gpio_hold_dis(LoRa_ModeSettingPin_M1);
    gpio_deep_sleep_hold_dis();
    SwitchToNormalMode();
    delay(100);
  }else{
    Serial.println("Waked up from nomal power on!");
    SerialMon.printf("\n sensorID: %d\n",senserID);
    SwitchToConfigurationMode();
    while(!digitalRead(LoRa_AUXPin)){}
    SerialMon.printf("I send conf\r\n");
    for (size_t i = 0; i < sizeof(conf); i++)
    {
      SerialMon.printf(" %02x",conf[i]);
    }
    SerialLoRa.write((uint8_t *)&conf, sizeof(conf));
    delay(10000);
    deep_sleep();
  }
  Serial.println("Wake and start ");
  SwitchToNormalMode();
  msg.bootcount = bootCount;
  Serial.printf("boot:%d \n" ,msg.bootcount);
  SerialLoRa.flush();
  uint8_t payload[]={msg.targetAdressH, msg.targetAdressL, msg.targetChannel ,
                    msg.myadress & 0xff ,msg.myadress >> 8 ,
                    msg.bootcount & 0xff, msg.bootcount >> 8, 
                    };
  SerialMon.printf("I send data\r\n");
  for (size_t i = 0; i < sizeof(payload); i++)
  {
    SerialMon.printf(" %02x",payload[i]);
  }
  SerialMon.println();
  SerialLoRa.write((uint8_t *)&payload, sizeof(payload));
  SerialLoRa.flush();
  delay(100);
  bootCount++;
  deep_sleep();
}

void loop() {}

