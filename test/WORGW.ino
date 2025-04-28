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
   * @brief WOR送信モード(M0=1,M1=0)へ移行する
   */
  void SwitchToWORReceivingMode(void){
  digitalWrite(LoRa_ModeSettingPin_M0, 1);
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