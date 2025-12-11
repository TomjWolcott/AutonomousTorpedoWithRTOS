/***
 * Seeed_Arduino_SSCMA.h
 * Description: A drive for Seeed Grove AI Family.
 * 2022 Copyright (c) Seeed Technology Inc.  All right reserved.
 * Author: Hongtai Liu(lht856@foxmail.com)
 *
 * Copyright (C) 2020  Seeed Technology Co.,Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Converted to c and edited to make work with the autono-torpedo on aug7 2025 by tomwol
/*
#ifndef SEEED_ARDUINO_SSCMA_H
#define SEEED_ARDUINO_SSCMA_H

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define I2C_ADDRESS (0x62 << 1)
#define I2C_PORT hi2c1

#define HEADER_LEN (uint8_t)4
#define MAX_PL_LEN (uint8_t)250
#define MAX_SPI_PL_LEN (uint16_t)4095
#define CHECKSUM_LEN (uint8_t)2

#define PACKET_SIZE (uint16_t)(HEADER_LEN + MAX_PL_LEN + CHECKSUM_LEN)

#define FEATURE_TRANSPORT 0x10
#define FEATURE_TRANSPORT_CMD_READ 0x01
#define FEATURE_TRANSPORT_CMD_WRITE 0x02
#define FEATURE_TRANSPORT_CMD_AVAILABLE 0x03
#define FEATURE_TRANSPORT_CMD_START 0x04
#define FEATURE_TRANSPORT_CMD_STOP 0x05
#define FEATURE_TRANSPORT_CMD_RESET 0x06

#define RESPONSE_PREFIX "\r{"
#define RESPONSE_SUFFIX "}\n"

#define RESPONSE_PREFIX_LEN (sizeof(RESPONSE_PREFIX) - 1)
#define RESPONSE_SUFFIX_LEN (sizeof(RESPONSE_SUFFIX) - 1)

#define CMD_PREFIX "AT+"
#define CMD_SUFFIX "\r\n"

#define CMD_TYPE_RESPONSE 0
#define CMD_TYPE_EVENT 1
#define CMD_TYPE_LOG 2

const char CMD_AT_ID[] = "ID?";
const char CMD_AT_NAME[] = "NAME?";
const char CMD_AT_VERSION[] = "VER?";
const char CMD_AT_STATS[] = "STAT";
const char CMD_AT_BREAK[] = "BREAK";
const char CMD_AT_RESET[] = "RST";
//const char CMD_AT_WIFI[] = "WIFI";
//const char CMD_AT_WIFI_VER[] = "WIFIVER"; // wifi version
//const char CMD_AT_WIFI_STA[] = "WIFISTA"; // wifi status
//const char CMD_AT_WIFI_IN4[] = "WIFIIN4";
//const char CMD_AT_WIFI_IN6[] = "WIFIIN6";
//const char CMD_AT_MQTTSERVER[] = "MQTTSERVER";
//const char CMD_AT_MQTTPUBSUB[] = "MQTTPUBSUB";
//const char CMD_AT_MQTTSERVER_STA[] = "MQTTSERVERSTA"; // mqtt status
const char CMD_AT_INVOKE[] = "INVOKE";
const char CMD_AT_SAMPLE[] = "SAMPLE";
const char CMD_AT_INFO[] = "INFO";
const char CMD_AT_TSCORE[] = "TSCORE";
const char CMD_AT_TIOU[] = "TIOU";
const char CMD_AT_ALGOS[] = "ALGOS";
const char CMD_AT_MODELS[] = "MODELS";
const char CMD_AT_MODEL[] = "MODEL";
const char CMD_AT_SENSORS[] = "SENSORS";
const char CMD_AT_ACTION[] = "ACTION";
const char CMD_AT_LED[] = "led";
const char CMD_AT_SAVE_JPEG[] = "save_jpeg()";

#define CMD_OK 0
#define CMD_AGAIN 1
#define CMD_ELOG 2
#define CMD_ETIMEDOUT 3
#define CMD_EIO 4
#define CMD_EINVAL 5
#define CMD_ENOMEM 6
#define CMD_EBUSY 7
#define CMD_ENOTSUP 8
#define CMD_EPERM 9
#define CMD_EUNKNOWN 10

const char EVENT_INVOKE[] = "INVOKE";
const char EVENT_SAMPLE[] = "SAMPLE";
//const char EVENT_WIFI[] = "WIFI";
//const char EVENT_MQTT[] = "MQTT";
const char EVENT_SUPERVISOR[] = "SUPERVISOR";

const char LOG_AT[] = "AT";
const char LOG_LOG[] = "LOG";

//typedef std::function<void(const char *resp, size_t len)> ResponseCallback;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint8_t score;
    uint8_t target;
} boxes_t;

typedef struct
{
    uint8_t target;
    uint8_t score;
} classes_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t score;
    uint8_t target;
} point_t;

typedef struct
{
    boxes_t box;
    point_t points[10];
} keypoints_t;

typedef struct
{
    uint16_t prepocess;
    uint16_t inference;
    uint16_t postprocess;
} perf_t;

//typedef struct
//{
//    int status;
//    int security;
//    char ssid[64];
//    char password[64];
//} wifi_t;

//typedef struct
//{
//    int status;
//    char ipv4[16];
//    char ipv6[64];
//    char netmask[16];
//    char gateway[16];
//} wifi_status_t;

//typedef struct
//{
//    int status;
//    uint16_t port;
//    bool use_ssl;
//    char server[128];
//    char username[128];
//    char password[128];
//    char client_id[128];
//} mqtt_t;
//
//typedef struct
//{
//    int status;
//} mqtt_status_t;

typedef struct {
    char name[32];
    char ID[32];

    char *tx_buf; // for cmd
    uint32_t tx_len;
    char *rx_buf; // for response
    uint32_t rx_len;
    char *payload;

//    pref_t perf; // performance metrics
//	boxes_t boxes[100]; // Maximum of 100 boxes, the last box will have x=0, y=0, score=0, target=255
//	classes_t classes[30]; // Maximum of 30 classes, the last class will have target=255, score=0
//	point_t points[100]; // Maximum of 100 points, the last point will have x=0,y=0,z=0,score=0,target=255
//	keypoints_t keypoints[100]; // Maximum of 100 points, the last keypoint will have the end box
} SSCMA;

SSCMA SSCMA_init();

void SSCMA_begin(SSCMA *dev);
void SSCMA_invoke(SSCMA *dev, int times, bool filter, bool show);
int SSCMA_available(SSCMA *dev);
//int SSCMA_read(SSCMA *dev, char *data, int length);
//int SSCMA_write(SSCMA *dev, const char *data, int length);
void SSCMA_reset(SSCMA *dev);
//void SSCMA_fetch(SSCMA *dev, ResponseCallback RespCallback);

char *SSCMA_ID(bool cache = true);
char *SSCMA_name(bool cache = true);
String SSCMA_info(bool cache = true);

int SSCMA_clean_actions();
int SSCMA_save_jpeg();

class SSCMA
{
private:
//    TwoWire *_wire;
//    HardwareSerial *_serial;
//    SPIClass *_spi;
//    int32_t _cs;
//    int32_t _sync;
//    int32_t _rst;
//    uint32_t _baud;
//    uint16_t _address;
//    int _wait_delay;
    perf_t _perf;
    std::vector<boxes_t> _boxes;
    std::vector<classes_t> _classes;
    std::vector<point_t> _points;
    std::vector<keypoints_t> _keypoints;

    char _name[32] = {0};
    char _ID[32] = {0};

    uint32_t rx_end = 0;

    String _image = "";
    String _info = "";

    char *tx_buf; // for cmd
    uint32_t tx_len;
    char *rx_buf; // for response
    uint32_t rx_len;
    char *payload; // for json payload

public:
    SSCMA();
    ~SSCMA();

//    bool begin(TwoWire *wire = &Wire, int32_t rst = -1, uint16_t address = I2C_ADDRESS,
//               uint32_t wait_delay = 2, uint32_t clock = SSCMA_IIC_CLOCK);
//    bool begin(HardwareSerial *serial, int32_t rst = -1, uint32_t baud = SSCMA_UART_BAUD,
//               uint32_t wait_delay = 2);
//    bool begin(SPIClass *spi, int32_t cs = -1, int32_t sync = -1, int32_t rst = -1,
//               uint32_t baud = SSCMA_SPI_CLOCK, uint32_t wait_delay = 2);
//    int invoke(int times = 1, bool filter = 0, bool show = 0);
//    int available();
//    int read(char *data, int length);
//    int write(const char *data, int length);
//    void reset();
//    void fetch(ResponseCallback RespCallback);

//    perf_t &perf() { return _perf; }
//    std::vector<boxes_t> &boxes() { return _boxes; }
//    std::vector<classes_t> &classes() { return _classes; }
//    std::vector<point_t> &points() { return _points; }
//    std::vector<keypoints_t> &keypoints() { return _keypoints; }

//    int WIFIVER(char *version);

//    int WIFI(wifi_t &wifi);
//    int MQTT(mqtt_t &mqtt);
//
//    int WIFISTA(wifi_status_t &wifi_status);
//    int MQTTSTA(mqtt_status_t &mqtt_status);

    char *ID(bool cache = true);
    char *name(bool cache = true);
    String info(bool cache = true);

    // actions
    int clean_actions();
    int save_jpeg();

//    String last_image() { return _image; }

//    bool set_rx_buffer(uint32_t size);
//    bool set_tx_buffer(uint32_t size);

private:
    int i2c_write(const char *data, int length);
    int i2c_read(char *data, int length);
    int i2c_available();
    void i2c_cmd(uint8_t feature, uint8_t cmd, uint16_t len = 0, uint8_t *data = NULL);

//    int spi_write(const char *data, int length);
//    int spi_read(char *data, int length);
//    int spi_available();
//    void spi_cmd(uint8_t feature, uint8_t cmd, uint16_t len = 0, uint8_t *data = NULL);

    int wait(int type, const char *cmd, uint32_t timeout = 1000);
    void praser_event();
    void praser_log();
};

#endif
*/