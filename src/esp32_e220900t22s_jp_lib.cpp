#include "esp32_e220900t22s_jp_lib.h"
#include <vector>

SemaphoreHandle_t xMutex;

template <typename T>
bool ConfRange(T target, T min, T max);

int CLoRa::LoadConfigSetting(const char *filename,
                             struct LoRaConfigItem_t &config) {
  int ret = 0;

  if (!SPIFFS.begin(true)) {
    SerialMon.printf("SPIFFS Mount Failed.\n");
  }

  // 最初にデフォルト値をセット
  SetDefaultConfigValue(config);

  // ESP32内にコンフィグファイルがあるか否か
  if (!SPIFFS.exists(filename)) {
    SerialMon.printf("config file is NOT exists.\n");
    ret = 1;
  } else {
    // コンフィグ値を設定
    // 値範囲外や読取エラー時はデフォルト値使用
    ret = OpenConfigFile(filename, config);
  }

  return ret;
}

int CLoRa::InitLoRaModule(struct LoRaConfigItem_t &config) {
  int ret = 0;

  xMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(xMutex);

  // コンフィグモード(M0=1,M1=1)へ移行する
  SwitchToConfigurationMode();

  SerialLoRa.end(); // end()を実行　←←追加
  delay(1000); // 1秒待つ　 ←←追加
  SerialLoRa.begin(LoRa_BaudRate, SERIAL_8N1, LoRa_Tx_ESP_RxPin,LoRa_Rx_ESP_TxPin);

  // Configuration
  std::vector<uint8_t> command = {0xc0, 0x00, 0x08};
  std::vector<uint8_t> response = {};

  // Register Address 00H, 01H
  uint8_t ADDH = config.own_address >> 8;
  uint8_t ADDL = config.own_address & 0xff;
  command.push_back(ADDH);
  command.push_back(ADDL);

  // Register Address 02H
  uint8_t REG0 = 0;
  REG0 = REG0 | (config.baud_rate << 5);
  REG0 = REG0 | (config.air_data_rate);
  command.push_back(REG0);

  // Register Address 03H
  uint8_t REG1 = 0;
  REG1 = REG1 | (config.subpacket_size << 6);
  REG1 = REG1 | (config.rssi_ambient_noise_flag << 5);
  REG1 = REG1 | (config.transmission_pause_flag << 4);
  REG1 = REG1 | (config.transmitting_power);
  command.push_back(REG1);

  // Register Address 04H
  uint8_t REG2 = config.own_channel;
  command.push_back(REG2);

  // Register Address 05H
  uint8_t REG3 = 0;
  REG3 = REG3 | (config.rssi_byte_flag << 7);
  REG3 = REG3 | (config.transmission_method_type << 6);
  REG3 = REG3 | (config.lbt_flag << 4);
  REG3 = REG3 | (config.wor_cycle);
  command.push_back(REG3);

  // Register Address 06H, 07H
  uint8_t CRYPT_H = config.encryption_key >> 8;
  uint8_t CRYPT_L = config.encryption_key & 0xff;
  command.push_back(CRYPT_H);
  command.push_back(CRYPT_L);

  SerialMon.printf("# Command Request\n");
  for (auto i : command) {
    SerialMon.printf("0x%02x ", i);
  }
  SerialMon.printf("\n");

  if (xSemaphoreTake(xMutex, (portTickType)100) == pdTRUE) {
    for (auto i : command) {
      SerialLoRa.write(i);
    }
    SerialLoRa.flush();
    xSemaphoreGive(xMutex);
  }

  delay(100);

  while (SerialLoRa.available()) {
    if (xSemaphoreTake(xMutex, (portTickType)100) == pdTRUE) {
      uint8_t data = SerialLoRa.read();
      response.push_back(data);
      xSemaphoreGive(xMutex);
    }
  }

  SerialMon.printf("# Command Response\n");
  for (auto i : response) {
    SerialMon.printf("0x%02x ", i);
  }
  SerialMon.printf("\n");

  if (response.size() != command.size()) {
    ret = 1;
  }

  return ret;
}

int CLoRa::RecieveFrame(struct RecvFrameE220900T22SJP_t *recv_frame) {
  int len = 0;
  uint8_t *start_p = recv_frame->recv_data;

  memset(recv_frame->recv_data, 0x00,
         sizeof(recv_frame->recv_data) / sizeof(recv_frame->recv_data[0]));

  while (1) {
    while (SerialLoRa.available()) {
      if (xSemaphoreTake(xMutex, (portTickType)100) == pdTRUE) {
        uint8_t ch = SerialLoRa.read();
        *(start_p + len) = ch;
        len++;
        xSemaphoreGive(xMutex);
      }
      if (len > 200) {
        return 1;
      }
    }
    if ((SerialLoRa.available() == 0) && (len > 0)) {
      delay(10);
      if (SerialLoRa.available() == 0) {
        recv_frame->recv_data_len = len - 1;
        recv_frame->rssi = recv_frame->recv_data[len - 1] - 256;
        break;
      }
    }
    delay(100);
  }

  return 0;
}

int CLoRa::SendFrame(struct LoRaConfigItem_t &config, uint8_t *send_data,
                     int size) {
  uint8_t subpacket_size = 0;
  switch (config.subpacket_size) {
  case 0b00:
    subpacket_size = 200;
    break;
  case 0b01:
    subpacket_size = 128;
    break;
  case 0b10:
    subpacket_size = 64;
    break;
  case 0b11:
    subpacket_size = 32;
    break;
  default:
    subpacket_size = 200;
    break;
  }
  if (size > subpacket_size) {
    SerialMon.printf("send data length too long\n");
    return 1;
  }
  uint8_t target_address_H = config.target_address >> 8;
  uint8_t target_address_L = config.target_address & 0xff;
  uint8_t target_channel = config.target_channel;

  uint8_t frame[3 + size] = {target_address_H, target_address_L,
                             target_channel};

  memmove(frame + 3, send_data, size);

#if 1 /* print debug */
  for (int i = 0; i < 3 + size; i++) {
    if (i < 3) {
      SerialMon.printf("%02x", frame[i]);
    } else {
      // SerialMon.printf("%c", frame[i]);
      SerialMon.printf("%02x", frame[i]);
    }
  }
  SerialMon.printf("\n");
#endif

  if (xSemaphoreTake(xMutex, (portTickType)100) == pdTRUE) {
    for (auto i : frame) {
      SerialLoRa.write(i);
    }
    SerialLoRa.flush();
    delay(100);
    while (SerialLoRa.available()) {
      while (SerialLoRa.available()) {
        SerialLoRa.read();
      }
      delay(100);
    }
    xSemaphoreGive(xMutex);
  }

  return 0;
}

void CLoRa::SwitchToNormalMode(void) {
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);

  digitalWrite(LoRa_ModeSettingPin_M0, 0);
  digitalWrite(LoRa_ModeSettingPin_M1, 0);
  delay(100);
}

void CLoRa::SwitchToWORSendingMode(void) {
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);

  digitalWrite(LoRa_ModeSettingPin_M0, 1);
  digitalWrite(LoRa_ModeSettingPin_M1, 0);
  delay(100);
}

void CLoRa::SwitchToWORReceivingMode(void) {
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);

  digitalWrite(LoRa_ModeSettingPin_M0, 0);
  digitalWrite(LoRa_ModeSettingPin_M1, 1);
  delay(100);
}

void CLoRa::SwitchToConfigurationMode(void) {
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);

  digitalWrite(LoRa_ModeSettingPin_M0, 1);
  digitalWrite(LoRa_ModeSettingPin_M1, 1);
  delay(100);
}

void CLoRa::SetDefaultConfigValue(struct LoRaConfigItem_t &config) {
  const LoRaConfigItem_t default_config = {
      0x0000, // own_address 0
      0b011, // baud_rate 9600 bps
      0b10000, // air_data_rate SF:9 BW:125
      0b11, // subpacket_size 200
      0b1, // rssi_ambient_noise_flag 有効
      0b0, // transmission_pause_flag 有効
      0b01, // transmitting_power 13 dBm
      0x00, // own_channel 0
      0b1, // rssi_byte_flag 有効
      0b0, // transmission_method_type トランスペアレント送信モード(default)
      0b0, // lbt_flag 有効
      0b011, // wor_cycle 2000 ms
      0x0000, // encryption_key 0
      0x0000, // target_address 0
      0x00}; // target_channel 0

  config = default_config;
}

int CLoRa::OpenConfigFile(const char *filename,
                          struct LoRaConfigItem_t &config) {
  int ret = 0;

  // フラッシュメモリのファイルを開く
  File file = SPIFFS.open(filename, FILE_READ);
  if (!file) {
    SerialMon.printf("file open failed.\n");
    return 1;
  }

  while (file.available()) {
    String key = file.readStringUntil('=');
    String val = file.readStringUntil('\n');

    key.toLowerCase();
    // 設定値を内部変数に読み取る
    ReadConfigValue(key.c_str(), val.c_str());
  }

  // 設定値をセットする
  SetConfigValue(config);

  return ret;
}

int CLoRa::ReadConfigValue(const char *key, const char *val) {
  int number = 0;
  int err = 0;

  if (strcmp(key, "own_address") == 0) {
    number = atoi(val);
    own_address_val = number;
  } else if (strcmp(key, "baud_rate") == 0) {
    number = atoi(val);
    baud_rate_val = number;
  } else if (strcmp(key, "bw") == 0) {
    number = atoi(val);
    bw_val = number;
  } else if (strcmp(key, "sf") == 0) {
    number = atoi(val);
    sf_val = number;
  } else if (strcmp(key, "subpacket_size") == 0) {
    number = atoi(val);
    subpacket_size_val = number;
  } else if (strcmp(key, "transmitting_power") == 0) {
    number = atoi(val);
    transmitting_power_val = number;
  } else if (strcmp(key, "own_channel") == 0) {
    number = atoi(val);
    own_channel_val = number;
  } else if (strcmp(key, "wor_cycle") == 0) {
    number = atoi(val);
    wor_cycle_val = number;
  } else if (strcmp(key, "encryption_key") == 0) {
    number = atoi(val);
    encryption_key_val = number;
  } else if (strcmp(key, "target_address") == 0) {
    number = atoi(val);
    target_address_val = number;
  } else if (strcmp(key, "target_channel") == 0) {
    number = atoi(val);
    target_channel_val = number;
  }

  return err;
}

// コンフィグ値が設定範囲内か否か
template <typename T>
bool ConfRange(T target, T min, T max) {
  if (target >= min && target <= max) {
    return true;
  } else {
    return false;
  }
}

int CLoRa::SetConfigValue(struct LoRaConfigItem_t &config) {
  int err = 0;

  // own_address
  if (ConfRange((int)own_address_val, 0, 65535)) {
    config.own_address = own_address_val;
  } else {
    err = 1;
    SerialMon.printf("own_address invalid value.\n");
    SerialMon.printf("default own_address value is used.\n");
  }

  // baud_rate
  switch (baud_rate_val) {
  case 1200:
    config.baud_rate = 0b000;
    break;
  case 2400:
    config.baud_rate = 0b001;
    break;
  case 4800:
    config.baud_rate = 0b010;
    break;
  case 9600:
    config.baud_rate = 0b011;
    break;
  case 19200:
    config.baud_rate = 0b100;
    break;
  case 38400:
    config.baud_rate = 0b101;
    break;
  case 57600:
    config.baud_rate = 0b110;
    break;
  case 115200:
    config.baud_rate = 0b111;
    break;
  default:
    err = 1;
    SerialMon.printf("baud_rate invalid value.\n");
    SerialMon.printf("default baud_rate value is used.\n");
    break;
  }

  // air_data_rate
  switch (bw_val) {
  case 125:
    switch (sf_val) {
    case 5:
      config.air_data_rate = 0b00000;
      break;
    case 6:
      config.air_data_rate = 0b00100;
      break;
    case 7:
      config.air_data_rate = 0b01000;
      break;
    case 8:
      config.air_data_rate = 0b01100;
      break;
    case 9:
      config.air_data_rate = 0b10000;
      break;
    default:
      err = 1;
      SerialMon.printf("sf invalid value.\n");
      SerialMon.printf("default sf value is used.\n");
      break;
    }
    break;
  case 250:
    switch (sf_val) {
    case 5:
      config.air_data_rate = 0b00001;
      break;
    case 6:
      config.air_data_rate = 0b00101;
      break;
    case 7:
      config.air_data_rate = 0b01001;
      break;
    case 8:
      config.air_data_rate = 0b01101;
      break;
    case 9:
      config.air_data_rate = 0b10001;
      break;
    case 10:
      config.air_data_rate = 0b10101;
      break;
    default:
      err = 1;
      SerialMon.printf("sf invalid value.\n");
      SerialMon.printf("default sf value is used.\n");
      break;
    }
    break;
  case 500:
    switch (sf_val) {
    case 5:
      config.air_data_rate = 0b00010;
      break;
    case 6:
      config.air_data_rate = 0b00110;
      break;
    case 7:
      config.air_data_rate = 0b01010;
      break;
    case 8:
      config.air_data_rate = 0b01110;
      break;
    case 9:
      config.air_data_rate = 0b10010;
      break;
    case 10:
      config.air_data_rate = 0b10110;
      break;
    case 11:
      config.air_data_rate = 0b11010;
      break;
    default:
      err = 1;
      SerialMon.printf("sf invalid value.\n");
      SerialMon.printf("default sf value is used.\n");
      break;
    }
    break;
  default:
    err = 1;
    SerialMon.printf("bw invalid value.\n");
    SerialMon.printf("default bw and sf value is used.\n");
    break;
  }

  // subpacket_size
  switch (subpacket_size_val) {
  case 200:
    config.subpacket_size = 0b00;
    break;
  case 128:
    config.subpacket_size = 0b01;
    break;
  case 64:
    config.subpacket_size = 0b10;
    break;
  case 32:
    config.subpacket_size = 0b11;
    break;
  default:
    err = 1;
    SerialMon.printf("subpacket_size invalid value.\n");
    SerialMon.printf("default subpacket_size value is used.\n");
    break;
  }

  // transmitting_power
  switch (transmitting_power_val) {
  case 13:
    config.transmitting_power = 0b01;
    break;
  case 7:
    config.transmitting_power = 0b10;
    break;
  case 0:
    config.transmitting_power = 0b11;
    break;
  default:
    err = 1;
    SerialMon.printf("transmitting_power invalid value.\n");
    SerialMon.printf("default transmitting_power value is used.\n");
    break;
  }

  // own_channel
  switch (bw_val) {
  case 125:
    if (ConfRange((int)own_channel_val, 0, 37)) {
      config.own_channel = own_channel_val;
    } else {
      err = 1;
      SerialMon.printf("own_channel invalid value.\n");
      SerialMon.printf("default own_channel value is used.\n");
    }
    break;
  case 250:
    if (ConfRange((int)own_channel_val, 0, 36)) {
      config.own_channel = own_channel_val;
    } else {
      err = 1;
      SerialMon.printf("own_channel invalid value.\n");
      SerialMon.printf("default own_channel value is used.\n");
    }
    break;
  case 500:
    if (ConfRange((int)own_channel_val, 0, 30)) {
      config.own_channel = own_channel_val;
    } else {
      err = 1;
      SerialMon.printf("own_channel invalid value.\n");
      SerialMon.printf("default own_channel value is used.\n");
    }
    break;
  default:
    err = 1;
    SerialMon.printf("bw invalid value.\n");
    SerialMon.printf("default own_channel value is used.\n");
    break;
  }

  // wor_cycle
  switch (wor_cycle_val) {
  case 500:
    config.wor_cycle = 0b000;
    break;
  case 1000:
    config.wor_cycle = 0b001;
    break;
  case 1500:
    config.wor_cycle = 0b010;
    break;
  case 2000:
    config.wor_cycle = 0b011;
    break;
  default:
    err = 1;
    SerialMon.printf("wor_cycle invalid value.\n");
    SerialMon.printf("default wor_cycle value is used.\n");
    break;
  }

  // encryption_key
  if (ConfRange((int)encryption_key_val, 0, 65535)) {
    config.encryption_key = encryption_key_val;
  } else {
    err = 1;
    SerialMon.printf("encryption_key invalid value.\n");
    SerialMon.printf("default encryption_key value is used.\n");
  }

  // target_address
  if (ConfRange((int)target_address_val, 0, 65535)) {
    config.target_address = target_address_val;
  } else {
    err = 1;
    SerialMon.printf("target_address invalid value.\n");
    SerialMon.printf("default target_address value is used.\n");
  }

  // target_channel
  switch (bw_val) {
  case 125:
    if (ConfRange((int)target_channel_val, 0, 37)) {
      config.target_channel = target_channel_val;
    } else {
      err = 1;
      SerialMon.printf("target_channel invalid value.\n");
      SerialMon.printf("default target_channel value is used.\n");
    }
    break;
  case 250:
    if (ConfRange((int)target_channel_val, 0, 36)) {
      config.target_channel = target_channel_val;
    } else {
      err = 1;
      SerialMon.printf("target_channel_val invalid value.\n");
      SerialMon.printf("default target_channel_val value is used.\n");
    }
    break;
  case 500:
    if (ConfRange((int)target_channel_val, 0, 30)) {
      config.target_channel = target_channel_val;
    } else {
      err = 1;
      SerialMon.printf("target_channel_val invalid value.\n");
      SerialMon.printf("default target_channel_val value is used.\n");
    }
    break;
  default:
    err = 1;
    SerialMon.printf("bw invalid value.\n");
    SerialMon.printf("default target_channel_val value is used.\n");
    break;
  }

  return err;
}