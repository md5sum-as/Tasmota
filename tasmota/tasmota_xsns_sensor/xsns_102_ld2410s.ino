/*
  xsns_102_ld2410s.ino - HLK-LD2410S 24GHz smart wave motion sensor support for Tasmota

  SPDX-FileCopyrightText: 2022 Theo Arends, 2024 md5sum-as (https://github.com/md5sum-as)

  SPDX-License-Identifier: GPL-3.0-only
*/
 #define USE_WEBSERVER
 #define USE_LD2410S
#ifdef USE_LD2410S
#ifndef USE_LD2410

/*********************************************************************************************\
 * HLK-LD2410S 24GHz smart wave motion sensor
 * 
 * Attention!
 * This module works with HLK-LD2410, HLK-LD2410B (md5sum-as tested), HLK-LD2410C (md5sum-as tested) devices. 
 * The module does not support HLK-LD2410S (md5sum-as tested) and is not guaranteed to work with other devices.
 * 
 * 
 * LD2410Duration 0                            - Set factory default settings
 * LD2410Duration 1..65535                     - Set no-one duration in seconds (default 5)
 * LD2410MovingSens 50,50,40,30,20,15,15,15,15 - Set moving distance sensitivity for up to 9 gates (at 0.75 meter interval)
 * LD2410StaticSens 0,0,40,40,30,30,20,20,20   - Set static distance sensitivity for up to 9 gates (at 0.75 meter interval)
 *
 * LD2410Get                                   - Read last sensors
 * LD2410EngineeringStart                      - Start engineering mode
 * LD2410EngineeringEnd                        - End engineering mode
 *
 * Inspiration:
 * https://community.home-assistant.io/t/mmwave-wars-one-sensor-module-to-rule-them-all/453260/2
 * Resources:
 * https://drive.google.com/drive/folders/1p4dhbEJA3YubyIjIIC7wwVsSo8x29Fq-?spm=a2g0o.detail.1000023.17.93465697yFwVxH
 *
 * Internal info:
 * - After a LD2410 serial command a response takes about 10mS
 * - After a LD2410 restart it takes at least 1000mS before commands are allowed
\*********************************************************************************************/

#define XSNS_102                         102

#undef TM_SERIAL_BUFFER_SIZE
#define TM_SERIAL_BUFFER_SIZE            128
#define LD2410S_BUFFER_SIZE               TM_SERIAL_BUFFER_SIZE  // 128

#define LD2410S_NUM_GATES                16

#define LD2410S_CMND_START_CONFIGURATION  0xFF
#define LD2410S_CMND_END_CONFIGURATION    0xFE
#define LD2410S_CMND_SET_COMMON           0x70
#define LD2410S_CMND_READ_COMMON          0x71
#define LD2410S_CMND_AUTO_THRESHOLD       0x09
#define LD2410S_CMND_WRITE_TRIGGER        0x72
#define LD2410S_CMND_READ_TRIGGER         0x73
#define LD2410S_CMND_WRITE_HOLD           0x76
#define LD2410S_CMND_READ_HOLD            0x77
#define LD2410S_CMND_OUTPUT_MODE          0x7A

const uint8_t LD2410_config_header[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t LD2410_config_footer[4] = {0x04, 0x03, 0x02, 0x01};
const uint8_t LD2410_target_header[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t LD2410_target_footer[4] = {0xF8, 0xF7, 0xF6, 0xF5};

#include <TasmotaSerial.h>
TasmotaSerial *LD2410Serial = nullptr;

struct {
  uint8_t *buffer;
// Common params
  uint8_t far_end;
  uint8_t near_end;
  uint8_t hold_duration;
  uint8_t status_report_f;
  uint8_t distance_report_f;
  uint8_t response_speed;
// gates param
  uint16_t trigger_energy[LD2410S_NUM_GATES];
  uint16_t hold_energy[LD2410S_NUM_GATES];
// Report values
  uint16_t detect_distance;
  uint16_t energy[LD2410S_NUM_GATES];
  uint8_t human;
  uint8_t human_last;
  
  uint8_t state;
  uint8_t step;
  uint8_t retry;
  uint8_t next_step;
  uint8_t byte_counter;
  uint8_t ack;
} LD2410S;

/********************************************************************************************/

uint32_t ToBcd(uint32_t value) {
  return ((value >> 4) * 10) + (value & 0xF);
}

/********************************************************************************************/

void Ld1410HandleTargetData(void) {
  if (LD2410S.step > 150) {LD2410S.step = 20;} //Stop boot delay on receive valid data
/* 
F4F3F2F1
4600
01
02
9100
4600
00000000
543D0000
1D160000
2E070000
D8020000
B0040000
DF000000
FD010000
74040000
B4070000
72090000
F0040000
C8000000
F0040000
A4060000
CE050000
F8F7F6F5
*/

}



void Ld1410HandleConfigData(void) {
  LD2410S.ack = 0;
  if ((LD2410S.buffer[8]==0) && (LD2410S.buffer[7]==1)) {
    LD2410S.ack = LD2410S.buffer[6];
  }
  if (LD2410S_CMND_READ_COMMON == LD2410S.buffer[6]) {
    // FDFCFBFA - header 0,1,2,3
    // 1C00 - datalen 4,5
    // 7101 - command 6,7
    // 0000 - ACK 8,9
    // 00000000 - Near 10,11,12,13
    // 0C000000 - Far 14,15,16,17
    // 0A000000 - Hold 18,19,20,21
    // 28000000 - StatusF 22,23,24,25
    // 05000000 - DistanceF 26,27,28,29
    // 05000000 - RespSpeed 30,31,32,33
    // 04030201
    LD2410S.near_end = LD2410S.buffer[10];
    LD2410S.far_end = LD2410S.buffer[14];
    LD2410S.hold_duration = LD2410S.buffer[18];
    LD2410S.status_report_f = LD2410S.buffer[22];
    LD2410S.distance_report_f = LD2410S.buffer[26];
    LD2410S.response_speed = LD2410S.buffer[30];
  }
}

bool Ld2410Match(const uint8_t *header, uint32_t offset) {
  for (uint32_t i = 0; i < 4; i++) {
    if (LD2410S.buffer[offset +i] != header[i]) { return false; }
  }
  return true;
}

bool Ld2410MatchShort(uint32_t offset) {
  if (LD2410S.buffer[offset] != 0x6e) { return false; }
  if (LD2410S.buffer[offset+1] > 0x03) { return false; }
  if (LD2410S.buffer[offset+4] != 0x62) { return false; }
  LD2410S.detect_distance=LD2410S.buffer[offset+3] << 8 | LD2410S.buffer[offset+2];
  LD2410S.human = LD2410S.buffer[offset+1];
  if (LD2410S.step > 150) {LD2410S.step = 20;}  //Stop boot delay on receive valid data
  return true;
}

void Ld2410Input(void) {
  while (LD2410Serial->available()) {
    yield();                                                    // Fix watchdogs

    LD2410S.buffer[LD2410S.byte_counter++] = LD2410Serial->read();
//    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("LD2: Rcvd %*_H"), LD2410.byte_counter, LD2410.buffer);

    if (LD2410S.byte_counter < 5) { continue; }                  // Need first four header bytes

    uint32_t header_start = LD2410S.byte_counter -5;             // Fix interrupted header transmits
    bool short_report  = (Ld2410MatchShort(header_start));
    if (short_report) {
      LD2410S.byte_counter = 0;                                    // Finished
      break;                                                      // Exit loop to satisfy yields
    }
    bool target_header = (Ld2410Match(LD2410_target_header, header_start));  // F4F3F2F1
    bool config_header = (Ld2410Match(LD2410_config_header, header_start));  // FDFCFBFA
    if ((target_header || config_header) && (header_start != 0)) {
      memmove(LD2410S.buffer, LD2410S.buffer + header_start, 5);  // Sync buffer with header
      LD2410S.byte_counter = 5;
    }
    if (LD2410S.byte_counter < 6) { continue; }                  // Need packet size bytes

    target_header = (Ld2410Match(LD2410_target_header, 0));     // F4F3F2F1
    config_header = (Ld2410Match(LD2410_config_header, 0));     // FDFCFBFA
    if (target_header || config_header) {
      uint32_t len = LD2410S.buffer[4] +10;                      // Total packet size
      if (len > LD2410S_BUFFER_SIZE) {
        LD2410S.byte_counter = 0;                                // Invalid data
        break;                                                  // Exit loop to satisfy yields
      }
      if (LD2410S.byte_counter < len) { continue; }              // Need complete packet
      if (target_header) {                                      // F4F3F2F1
        if (Ld2410Match(LD2410_target_footer, len -4)) {        // F8F7F6F5
          AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("LD2: Rcvd %*_H"), len, LD2410S.buffer);
          Ld1410HandleTargetData();
        }
      }
      else if (config_header) {                                 // FDFCFBFA
        if (Ld2410Match(LD2410_config_footer, len -4)) {        // 04030201
          AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("LD2: Rcvd %*_H"), len, LD2410S.buffer);
          Ld1410HandleConfigData();
          LD2410Serial->setReadChunkMode(0);                    // Disable chunk mode fixing Hardware Watchdogs
        }
      }
    }
    LD2410S.byte_counter = 0;                                    // Finished or bad received footer
    break;                                                      // Exit loop to satisfy yields
  }
  // If here then LD2410.byte_counter could still be partial correct for next loop
}

void Ld2410SendCommand(uint32_t command, uint8_t *val = nullptr, uint32_t val_len = 0);
void Ld2410SendCommand(uint32_t command, uint8_t *val, uint32_t val_len) {
  uint32_t len = val_len +12;
  uint8_t buffer[len];
  buffer[0] = 0xFD;
  buffer[1] = 0xFC;
  buffer[2] = 0xFB;
  buffer[3] = 0xFA;
  buffer[4] = val_len +2;
  buffer[5] = 0x00;
  buffer[6] = command;
  buffer[7] = 0x00;
  if (val) {
    for (uint32_t i = 0; i < val_len; i++) {
      buffer[8 +i] = val[i];
    }
  }
  buffer[8 +val_len] = 0x04;
  buffer[9 +val_len] = 0x03;
  buffer[10 +val_len] = 0x02;
  buffer[11 +val_len] = 0x01;

  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("LD2: Send %*_H"), len, buffer);

  LD2410Serial->setReadChunkMode(1);                            // Enable chunk mode introducing possible Hardware Watchdogs
  LD2410Serial->flush();
  LD2410Serial->write(buffer, len);
}

void Ld2410SetConfigMode(void) {                                // 0xFF
  uint8_t value[2] = { 0x01, 0x00 };
  Ld2410SendCommand(LD2410S_CMND_START_CONFIGURATION, value, sizeof(value));
}

void Ld2410SetOutputMode(uint8 mode) {
  uint8_t value[6] = { 0,0,0,0,0,0 };
  value[2] = mode?1:0;
  Ld2410SendCommand(LD2410S_CMND_OUTPUT_MODE, value, sizeof(value));
}

void Ld2410ReadCommonParameters(void) {
  /*
  Detection of the closest distance door  0x0A  0~16
  Detection of the farthest distance door 0x05  1~16
  No delay time                           0x06  10 ~ 120s
  Status reporting frequency              0x02  0.5 ~ 8（0.5SteppingHz)
  Distance reporting frequency            0x0C  0.5 ~ 8（0.5SteppingHz)
  Response speed                          0x0B  5(normal)/10(fast)-
*/
  uint8_t value[12] = { 0x0A, 0x00, 0x05, 0x00, 0x06, 0x00, 0x02, 0x00, 0x0C, 0x00, 0x0B, 0x00};
  Ld2410SendCommand(LD2410S_CMND_READ_COMMON, value, sizeof(value));
}

void Ld2410SetCommonParametrs(void) {
  uint8_t value[36] = { 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x0C, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x0B, 0x00, 0x00, 0x00, 0x00, 0x00 };
value[2] = LD2410S.near_end;
value[8] = LD2410S.far_end;
value[14] = LD2410S.hold_duration;
value[20] = LD2410S.status_report_f;
value[26] = LD2410S.distance_report_f;
value[32] = LD2410S.response_speed;
Ld2410SendCommand(LD2410S_CMND_SET_COMMON, value, sizeof(value));
}

/********************************************************************************************/

void Ld2410Every100MSecond(void) {
  if (LD2410S.step) {
    LD2410S.step--;
    switch (LD2410S.step) {
      // boot module delay
      case 200:
        LD2410S.step = 15;
        break;

      // 50 - write common
      case 49:
        Ld2410SetCommonParametrs();
        break;

      // 40 - read params
      case 39:
        Ld2410ReadCommonParameters();
        break;

      case 37:
        if (LD2410S.ack != LD2410S_CMND_READ_COMMON) {
          if (LD2410S.retry--) {
            LD2410S.step=40;
            break;
          }
        }
        LD2410S.step=5; // End command
        break;

      case 28:
        break;
      case 26:
        break;
      // 20 - loop
      // 15 - Config mode
      case 14:
        Ld2410SetConfigMode();                                  // Stop running mode
        break;
      case 10:
        if (LD2410S.ack != LD2410S_CMND_START_CONFIGURATION) {
          if (LD2410S.retry--) {
            LD2410S.step = 20;                                    // Retry
          } else {
            LD2410S.step = 0;
            AddLog(LOG_LEVEL_DEBUG, PSTR("LD2: Not detected"));
          }
        } else {
          LD2410S.step = LD2410S.next_step;
        }
        break;
      case 1:
        Ld2410SendCommand(LD2410S_CMND_END_CONFIGURATION);
        break;
      default:
        break;
    }
  }
}

void Ld2410EverySecond(void) {
  if (LD2410S.human != LD2410S.human_last) {
    LD2410S.human_last = LD2410S.human;
    MqttPublishSensor();
  }
}

void Ld2410Detect(void) {
  if (PinUsed(GPIO_LD2410_RX) && PinUsed(GPIO_LD2410_TX)) {
    LD2410S.buffer = (uint8_t*)malloc(LD2410S_BUFFER_SIZE);    // Default 64
    AddLog(LOG_LEVEL_DEBUG, PSTR("LD2: Buff size %d"), LD2410S_BUFFER_SIZE);
    if (!LD2410S.buffer) { AddLog(LOG_LEVEL_DEBUG, PSTR("LD2: No buff")); return; }
    LD2410Serial = new TasmotaSerial(Pin(GPIO_LD2410_RX), Pin(GPIO_LD2410_TX), 2);
    if (LD2410Serial->begin(115200)) {
      if (LD2410Serial->hardwareSerial()) { ClaimSerial(); }
#ifdef ESP32
      AddLog(LOG_LEVEL_DEBUG, PSTR("LD2: Serial UART%d"), LD2410Serial->getUart());
#endif

     LD2410S.retry = 4;
     LD2410S.step = 250;
     LD2410S.next_step = 40;
    }
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

const char kLd2410Commands[] PROGMEM = "LD2410S_|"  // Prefix
  "Common|Get|EngineeringEnd|EngineeringStart";

void (* const Ld2410Command[])(void) PROGMEM = {
  &CmndLd2410Common, &CmndLd2410last, &CmndLd2410EngineeringEnd, &CmndLd2410EngineeringStart };

void Ld2410Response(void) {
  // Response_P(PSTR("{\"LD2410\":{\"Duration\":%d,\"Moving\":{\"Gates\":%d,\"Sensitivity\":["),
  //   LD2410.no_one_duration, LD2410.max_moving_distance_gate);
  // for (uint32_t i = 0; i <= LD2410_MAX_GATES; i++) {
  //   ResponseAppend_P(PSTR("%s%d"), (i==0)?"":",", LD2410.moving_sensitivity[i]);
  // }
  // ResponseAppend_P(PSTR("]},\"Static\":{\"Gates\":%d,\"Sensitivity\":["), LD2410.max_static_distance_gate);
  // for (uint32_t i = 0; i <= LD2410_MAX_GATES; i++) {
  //   ResponseAppend_P(PSTR("%s%d"), (i==0)?"":",", LD2410.static_sensitivity[i]);
  // }
  // ResponseAppend_P(PSTR("]}}}"));
}

void CmndLd2410Common(void) {
  if (ArgC() == 6) {
    uint32_t param[6] = { 0 };
    ParseParameters(6, param);
    param[3]=(param[3]/5)*5;
    param[3]=(param[4]/5)*5;
    if (param[0]>16) {param[0] = 16;}
    if (param[1]>16) {param[1] = 16;}
    if (param[1]<1) {param[1] = 1;}
    if (param[2]>120) {param[2] = 120;}
    if (param[2]<10) {param[2] = 10;}
    if (param[3]>80) {param[3] = 80;}
    if (param[3]<5) {param[3] = 5;}
    if (param[4]>80) {param[4] = 80;}
    if (param[4]<5) {param[4] = 5;}
    LD2410S.near_end = (uint8_t)param[0];
    LD2410S.far_end = (uint8_t)param[1];
    LD2410S.hold_duration = (uint8_t)param[2];
    LD2410S.status_report_f = (uint8_t)param[3];
    LD2410S.distance_report_f = (uint8_t)param[4];
    LD2410S.response_speed = (param[5]>5?10:5);

    LD2410S.retry = 4;
    LD2410S.step = 15;
    LD2410S.next_step = 50;

  }
  Response_P(PSTR("{\"LD2410S_Common\":{\"Near Door\": %d,\"Far Door\":%d,\"Hold Time\":%d,\"Status freq\":%d,\"Distance freq\":%d,\"Responce speed\":%d}}"),
              LD2410S.near_end, LD2410S.far_end, LD2410S.hold_duration, LD2410S.status_report_f, LD2410S.distance_report_f,LD2410S.response_speed);
}

void CmndLd2410last(void) {
}

void CmndLd2410EngineeringEnd(void) {
}

void CmndLd2410EngineeringStart(void) {
}

/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/

#ifdef USE_WEBSERVER
const char HTTP_SNS_LD2410_CM[] PROGMEM =
  "{s}LD2410 " D_DETECT_DISTANCE "{m}%1_f " D_UNIT_CENTIMETER "{e}";

const char HTTP_SNS_LD2410_ENG[] PROGMEM =
  "{s}LD2410 " D_MOVING_ENERGY_T "{m}%d %d %d %d %d %d %d %d %d{e}"
  "{s}LD2410 " D_STATIC_ENERGY_T "{m}%d %d %d %d %d %d %d %d %d{e}"
  "{s}LD2410 " D_LD2410_LIGHT "{m}%d{e}"
  "{s}LD2410 " D_LD2410_PIN_STATE "{m}%d{e}";
#endif

void Ld2410Show(bool json) {
  float detect_distance = LD2410S.detect_distance;

  if (json) {
    ResponseAppend_P(PSTR(",\"LD2410\":{\"" D_JSON_DISTANCE "\":%1_f, \"" D_JSON_PEOPLE "\":%1d}"), &detect_distance, LD2410S.human);

#ifdef USE_WEBSERVER
  } else {
     WSContentSend_PD(HTTP_SNS_LD2410_CM, &detect_distance);
//     if (LD2410.web_engin_mode == 1) {
//       WSContentSend_PD(HTTP_SNS_LD2410_ENG, 
//           LD2410.engineering.moving_gate_energy[0],LD2410.engineering.moving_gate_energy[1],LD2410.engineering.moving_gate_energy[2],
//           LD2410.engineering.moving_gate_energy[3],LD2410.engineering.moving_gate_energy[4],LD2410.engineering.moving_gate_energy[5],
//           LD2410.engineering.moving_gate_energy[6],LD2410.engineering.moving_gate_energy[7],LD2410.engineering.moving_gate_energy[8],
//           LD2410.engineering.static_gate_energy[0],LD2410.engineering.static_gate_energy[1],LD2410.engineering.static_gate_energy[2],
//           LD2410.engineering.static_gate_energy[3],LD2410.engineering.static_gate_energy[4],LD2410.engineering.static_gate_energy[5],
//           LD2410.engineering.static_gate_energy[6],LD2410.engineering.static_gate_energy[7],LD2410.engineering.static_gate_energy[8],
//           LD2410.engineering.light,LD2410.engineering.out_pin);
//     }
 #endif
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns102(uint32_t function) {
  bool result = false;

  if (FUNC_INIT == function) {
    Ld2410Detect();
  }
  else if (LD2410Serial) {
    switch (function) {
      case FUNC_LOOP:
      case FUNC_SLEEP_LOOP:
        Ld2410Input();
        break;
      case FUNC_EVERY_100_MSECOND:
        Ld2410Every100MSecond();
        break;
      case FUNC_EVERY_SECOND:
        Ld2410EverySecond();
        break;
      case FUNC_JSON_APPEND:
        Ld2410Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        Ld2410Show(0);
        break;
#endif  // USE_WEBSERVER
      case FUNC_COMMAND:
        result = DecodeCommand(kLd2410Commands, Ld2410Command);
        break;
    }
  }
  return result;
}

#endif  // USE_LD2410
#endif  // USE_LD2410S
