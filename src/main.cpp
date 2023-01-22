#include "esp32_e220900t22s_jp_lib.h"
#include <Arduino.h>
#include <FS.h>
// #include <SD.h>
// #include <SPI.h>

CLoRa lora;
struct LoRaConfigItem_t config;
struct RecvFrameE220900T22SJP_t data;

/** prototype declaration **/
void LoRaRecvTask(void *pvParameters);
void LoRaSendTask(void *pvParameters);
void ReadDataFromConsole(char *msg, int max_msg_len);

void setup() {
  // put your setup code here, to run once:
  SerialMon.begin(9600);
  delay(1000); // SerialMon init wait
  SerialMon.println();

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

  // マルチタスク
  xTaskCreateUniversal(LoRaRecvTask, "LoRaRecvTask", 8192, NULL, 1, NULL,
                       0);
  xTaskCreateUniversal(LoRaSendTask, "LoRaSendTask", 8192, NULL, 1, NULL,
                       1);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}

void LoRaRecvTask(void *pvParameters) {
  while (1) {
    if (lora.RecieveFrame(&data) == 0) {
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
    }

    delay(1);
  }
}

void LoRaSendTask(void *pvParameters) {
  int c=0xFF;
  while (1) {
    char msg[200] = {0};
    msg[0]=c--;

    // ESP32がコンソールから読み込む
    // ReadDataFromConsole(msg, (sizeof(msg) / sizeof(msg[0])));
    SerialMon.printf("I send data\n");
    if (lora.SendFrame(config, (uint8_t *)msg, strlen(msg)) == 0) {
      SerialMon.printf("send succeeded.\n");
      SerialMon.printf("\n");
    } else {
      SerialMon.printf("send failed.\n");
      SerialMon.printf("\n");
    }

    SerialMon.flush();

    delay(1000);
  }
}

void ReadDataFromConsole(char *msg, int max_msg_len) {
  int len = 0;
  char *start_p = msg;

  while (len < max_msg_len) {
    if (SerialMon.available() > 0) {
      char incoming_byte = SerialMon.read();
      if (incoming_byte == 0x00 || incoming_byte > 0x7F)
        continue;
      *(start_p + len) = incoming_byte;
      // 最短で3文字(1文字 + CR LF)
      if (incoming_byte == 0x0a && len >= 2 && (*(start_p + len - 1)) == 0x0d) {
        break;
      }
      len++;
    }
    delay(1);
  }

  // msgからCR LFを削除
  len = strlen(msg);
  for (int i = 0; i < len; i++) {
    if (msg[i] == 0x0D || msg[i] == 0x0A) {
      msg[i] = '\0';
    }
  }
}