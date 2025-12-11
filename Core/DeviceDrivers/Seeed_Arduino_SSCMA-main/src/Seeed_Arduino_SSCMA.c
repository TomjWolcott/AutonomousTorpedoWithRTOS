/***
 * Seeed_Arduino_GroveAI.cpp
 * Description: A drive for Seeed Grove AI Family.
 * 2022 Copyright (c) Seeed Technology Inc.  All right reserved.
 * Author: Hongtai Liu(lht856@foxmail.com)
 * 2022-4-24
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
#include "Seeed_Arduino_SSCMA.h"
#include <stdlib.h>

SSCMA SSCMA_init() {
    SSCMA sscma;
}

bool SSCMA_begin()
{
    response.clear();

    if (_rst >= 0)
    {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, LOW);
        delay(50);
        pinMode(_rst, INPUT);
        delay(500);
    }

    return ID(false) && name(false);
}

int SSCMA_write(const char *data, int length)
{
    return i2c_write(data, length);
}

int SSCMA_read(char *data, int length)
{
    return i2c_read(data, length);
}

int SSCMA_available()
{
    return i2c_available();
}

int SSCMA_i2c_cmd(uint8_t feature, uint8_t cmd, uint16_t len, uint8_t *data)
{
    HAL_Delay(2);
    uint8_t *buf = calloc(len + 6, sizeof(uint8_t));
    if (buf == NULL) return -1;

    buf[0] = feature;
    buf[1] = cmd;
    buf[2] = len >> 8;
    buf[3] = len & 0xFF;
    if (data != NULL) {
        memcpy(buf + 4, data, len);
    }
    // TODO checksum
    buf[len + 4] = 0;
    buf[len + 5] = 0;
    int status = HAL_I2C_Master_Transmit(&I2C_PORT, I2C_ADDRESS, buf, len + 6, 100);
    free(buf);

    if (status != HAL_OK) return -2;

    return 0;
}

int SSCMA_i2c_available()
{
    uint8_t receive_buf[2] = {0};
    uint8_t transmit_buf[6] = {
            FEATURE_TRANSPORT,
            FEATURE_TRANSPORT_CMD_AVAILABLE,
            0, 0,
            // TODO checksum
            0, 0
    };
    delay(2);

    if (HAL_I2C_Master_Transmit(&I2C_PORT, I2C_ADDRESS, transmit_buf, 6, 100) != HAL_OK) {
        return -1;
    }
    HAL_Delay(2)
    if (HAL_I2C_Master_Receive(&I2C_PORT, I2C_ADDRESS, receive_buf, 2, 100) != HAL_OK) {
        return -2;
    }

    return (receive_buf[0] << 8) | receive_buf[1];
}

static int SSCMA_i2c_read_packet(char *data, int length) {
    HAL_Delay(2);

    uint8_t tx[6] = {
            FEATURE_TRANSPORT,
            FEATURE_TRANSPORT_CMD_READ,
            (MAX_PL_LEN >> 8) & 0xFF,
            MAX_PL_LEN & 0xFF,
            // TODO Checksum
            0, 0
    };

    if (HAL_I2C_Master_Transmit(&I2C_PORT, (I2C_ADDRESS << 1), tx, sizeof(tx), 100) != HAL_OK) {
        return -1;
    }

    HAL_Delay(2);

    if (HAL_I2C_Master_Receive(&I2C_PORT, I2C_ADDRESS, (uint8_t*)(data), length, 100) != HAL_OK) {
        return -2;
    }

    return 0;
}

static int SSCMA_i2c_write_packet(char *data, int length) {
    HAL_Delay(2);

    uint8_t tx[MAX_PL_LEN + 6];
    tx[0] = FEATURE_TRANSPORT;
    tx[1] = FEATURE_TRANSPORT_CMD_WRITE;
    tx[2] = (MAX_PL_LEN >> 8) & 0xFF;
    tx[3] = MAX_PL_LEN & 0xFF;
    memcpy(&tx[4], data, length);
    // TODO Checksum
    tx[4 + MAX_PL_LEN] = 0;
    tx[5 + MAX_PL_LEN] = 0;

    if (HAL_I2C_Master_Transmit(&I2C_PORT, I2C_ADDRESS, tx, MAX_PL_LEN + 6, 100) != HAL_OK) {
        return -1;
    }

    return 0;
}

int SSCMA_i2c_read(char *data, int length) {
    uint16_t packets = length / MAX_PL_LEN;
    uint8_t remain = length % MAX_PL_LEN;
    for (uint16_t i = 0; i < packets; i++) {
        delay(2);
        int status = SSCMA_i2c_read_packet(data + i * MAX_PL_LEN, MAX_PL_LEN);

        if (status != 0) return status;
    }

    if (remain) {
        delay(2);
        int status = SSCMA_i2c_read_packet(data + packets * MAX_PL_LEN, MAX_PL_LEN);

        if (status != 0) return status;
    }

    return length;
}

int SSCMA_i2c_write(const char *data, int length) {
    uint16_t packets = length / MAX_PL_LEN;
    uint16_t remain = length % MAX_PL_LEN;
    for (uint16_t i = 0; i < packets; i++) {
        delay(2);
        int status = SSCMA_i2c_write_packet(data + i * MAX_PL_LEN, MAX_PL_LEN);

        if (status != 0) return status;
    }

    if (remain) {
        delay(2);
        int status = SSCMA_i2c_write_packet(data + packets * MAX_PL_LEN, remain);

        if (status != 0) return status;
    }

    return length;
}

void SSCMA_praser_event()
{
    if (strstr(response["name"], CMD_AT_INVOKE))
    {
        if (response["data"].containsKey("perf"))
        {
            _perf.prepocess = response["data"]["perf"][0];
            _perf.inference = response["data"]["perf"][1];
            _perf.postprocess = response["data"]["perf"][2];
        }

        if (response["data"].containsKey("boxes"))
        {
            _boxes.clear();
            JsonArray boxes = response["data"]["boxes"];
            for (size_t i = 0; i < boxes.size(); i++)
            {
                JsonArray box = boxes[i];
                boxes_t b;
                b.x = box[0];
                b.y = box[1];
                b.w = box[2];
                b.h = box[3];
                b.score = box[4];
                b.target = box[5];
                _boxes.push_back(b);
            }
        }

        if (response["data"].containsKey("classes"))
        {
            _classes.clear();
            JsonArray classes = response["data"]["classes"];
            for (size_t i = 0; i < classes.size(); i++)
            {
                JsonArray cls = classes[i];
                classes_t c;
                c.target = cls[1];
                c.score = cls[0];
                _classes.push_back(c);
            }
        }

        if (response["data"].containsKey("points"))
        {
            _points.clear();
            JsonArray points = response["data"]["points"];
            for (size_t i = 0; i < points.size(); i++)
            {
                JsonArray point = points[i];
                point_t p;
                p.x = point[0];
                p.y = point[1];
                // p.z = point[2];
                p.score = point[2];
                p.target = point[3];
                _points.push_back(p);
            }
        }

        if (response["data"].containsKey("keypoints"))
        {
            _keypoints.clear();
            JsonArray keypoints = response["data"]["keypoints"];
            for (size_t i = 0; i < keypoints.size(); i++)
            {
                keypoints_t k;
                JsonArray box = keypoints[i][0];
                JsonArray points = keypoints[i][1];
                k.box.x = box[0];
                k.box.y = box[1];
                k.box.w = box[2];
                k.box.h = box[3];
                k.box.score = box[4];
                k.box.target = box[5];

                for (size_t j = 0; j < points.size(); j++)
                {
                    point_t p;
                    p.x = points[j][0];
                    p.y = points[j][1];
                    // p.z = points[j][2];
                    p.score = points[j][2];
                    p.target = points[j][3];
                    k.points.push_back(p);
                }
                _keypoints.push_back(k);
            }
        }
        if (response["data"].containsKey("image"))
        {
            _image = response["data"]["image"].as<String>();
        }
    }
}

void SSCMA_praser_log()
{
}

int SSCMA_wait(int type, const char *cmd, uint32_t timeout)
{
    int ret = CMD_OK;
    uin32_t startTime = HAL_Delay;
    while (millis() - startTime <= timeout)
    {
        int len = SSCMA_available();
        if (len == 0)
            continue;
        if (len + rx_end > this->rx_len)
        {
            len = this->rx_len - rx_end;
            if (len <= 0)
            {
                rx_end = 0;
                continue;
            }
        }

        rx_end += SSCMA_read(rx_buf + rx_end, len);
        rx_buf[rx_end] = '\0';

        while (char *suffix = strnstr(rx_buf, RESPONSE_SUFFIX, rx_end))
        {
            if (char *prefix = strnstr(rx_buf, RESPONSE_PREFIX, suffix - rx_buf))
            {
                // get json payload
                len = suffix - prefix + RESPONSE_SUFFIX_LEN;
                payload = (char *)malloc(len);

                if (!payload)
                {
                    continue;
                }

                memcpy(payload, prefix + 1, len - 1); // remove "\r" and "\n"
                memmove(rx_buf, suffix + RESPONSE_SUFFIX_LEN, rx_end - (suffix - rx_buf) - RESPONSE_SUFFIX_LEN);
                rx_end -= suffix - rx_buf + RESPONSE_SUFFIX_LEN;
                payload[len - 1] = '\0';
                //Serial.printf("\npayload :%s", payload);
                // parse json response
                // for(size_t i = 0; i < strlen(payload); i++){
                //     Serial.printf("%c", payload[i]);
                // }
                response.clear();
                DeserializationError error = deserializeJson(response, payload);
                free(payload);
                if (error)
                {
                    continue;
                }

                if (response["type"] == CMD_TYPE_EVENT)
                {
                    praser_event();
                }

                if (response["type"] == CMD_TYPE_LOG)
                {
                    praser_log();
                }

                ret = response["code"];

                if (response["type"] == type && strncmp(response["name"], cmd, sizeof(cmd)) == 0)
                {
                    return ret;
                }
            }
            else
            {
                // discard this reply
                memmove(rx_buf, suffix + RESPONSE_PREFIX_LEN, rx_end - (suffix - rx_buf) - RESPONSE_PREFIX_LEN);
                rx_end -= suffix - rx_buf + RESPONSE_PREFIX_LEN;
                rx_buf[rx_end] = '\0';
            }
        }
    }

    return CMD_ETIMEDOUT;
}

int SSCMA_invoke(int times, bool filter, bool show)
{
    char cmd[64] = {0};

    if (show && rx_len < 16 * 1024)
    {
        return CMD_ENOTSUP;
    }

    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=%d,%d,%d" CMD_SUFFIX,
             CMD_AT_INVOKE, times, !filter, filter); // AT+INVOKE=1,0,1\r\n
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, CMD_AT_INVOKE) == CMD_OK)
    {
        if (wait(CMD_TYPE_EVENT, CMD_AT_INVOKE) == CMD_OK)
        {
            return CMD_OK;
        }
    }

    return CMD_ETIMEDOUT;
}

int SSCMA_WIFI(wifi_t &wifi)
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s?" CMD_SUFFIX, CMD_AT_WIFI);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, "WIFI?", 1000) == CMD_OK)
    {
        wifi.status = response["data"]["status"];
        wifi.security = response["data"]["config"]["security"];
        strcpy(wifi.ssid, response["data"]["config"]["name"]);
        strcpy(wifi.password, response["data"]["config"]["password"]);
        return CMD_OK;
    }

    return CMD_ETIMEDOUT;
}

int SSCMA_MQTT(mqtt_t &mqtt)
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s?" CMD_SUFFIX, CMD_AT_MQTTSERVER);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, "MQTTSERVER?", 1000) == CMD_OK)
    {
        mqtt.status = response["data"]["status"];
        mqtt.port = response["data"]["config"]["port"];
        mqtt.use_ssl = response["data"]["config"]["use_ssl"] == 1;
        strcpy(mqtt.server, response["data"]["config"]["address"]);
        strcpy(mqtt.username, response["data"]["config"]["username"]);
        strcpy(mqtt.password, response["data"]["config"]["password"]);
        strcpy(mqtt.client_id, response["data"]["config"]["client_id"]);
        return CMD_OK;
    }

    return CMD_ETIMEDOUT;
}

char *SSCMA_ID(bool cache)
{
    if (cache && _ID)
    {
        return _ID;
    }
    char cmd[64] = {0};

    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s" CMD_SUFFIX, CMD_AT_ID);

    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, CMD_AT_ID) == CMD_OK)
    {
        strcpy(_ID, response["data"]);
        return _ID;
    }

    return NULL;
}
char *SSCMA_name(bool cache)
{
    if (cache && _name[0])
    {
        return _name;
    }
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s" CMD_SUFFIX, CMD_AT_NAME);

    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, CMD_AT_NAME, 3000) == CMD_OK)
    {
        strcpy(_name, response["data"]);
        return _name;
    }

    return NULL;
}

String SSCMA_info(bool cache)
{
    if (cache && _info.length())
    {
        return _info;
    }

    char cmd[64] = {0};

    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s?" CMD_SUFFIX, CMD_AT_INFO);

    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, CMD_AT_INFO, 3000) == CMD_OK)
    {
        _info = response["data"]["info"].as<String>();
        return _info;
    }

    return "";
}

int SSCMA_WIFISTA(wifi_status_t &wifi_status)
{
    char cmd[128] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=%d" CMD_SUFFIX, CMD_AT_WIFI_STA, wifi_status.status);

    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, CMD_AT_WIFI_STA) == CMD_OK)
    {
        snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=\"%s\",\"%s\",\"%s\"" CMD_SUFFIX, CMD_AT_WIFI_IN4, wifi_status.ipv4, wifi_status.netmask, wifi_status.gateway);
        write(cmd, strlen(cmd));
        if (wait(CMD_TYPE_RESPONSE, CMD_AT_WIFI_IN4) == CMD_OK)
        {
            return CMD_OK;
        }
    }
    return CMD_ETIMEDOUT;
}

int SSCMA_MQTTSTA(mqtt_status_t &mqtt_status)
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=%d" CMD_SUFFIX, CMD_AT_MQTTSERVER_STA, mqtt_status.status);

    write(cmd, strlen(cmd));
    if (wait(CMD_TYPE_RESPONSE, CMD_AT_MQTTSERVER_STA) == CMD_OK)
    {
        return CMD_OK;
    }

    return CMD_ETIMEDOUT;
}

int SSCMA_WIFIVER(char *version)
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=\"%s\"" CMD_SUFFIX, CMD_AT_WIFI_VER, version);

    write(cmd, strlen(cmd));
    if (wait(CMD_TYPE_RESPONSE, CMD_AT_WIFI_VER) == CMD_OK)
    {
        return CMD_OK;
    }

    return CMD_ETIMEDOUT;
}

bool SSCMA_set_rx_buffer(uint32_t size)
{
    if (size == 0)
    {
        return false;
    }
    if (this->rx_len == 0)
    {
        this->rx_buf = (char *)malloc(size);
    }
    else
    {
        this->rx_buf = (char *)realloc(this->rx_buf, size);
    }
    if (this->rx_buf)
    {
        this->rx_end = 0;
        this->rx_len = size;
    }
    return this->rx_buf != NULL;
}
bool SSCMA_set_tx_buffer(uint32_t size)
{
    if (size == 0)
    {
        return false;
    }
    if (this->tx_len == 0)
    {
        this->tx_buf = (char *)malloc(size);
    }
    else
    {
        this->tx_buf = (char *)realloc(this->tx_buf, size);
    }
    if (this->tx_buf)
    {
        this->tx_len = size;
    }
    return this->tx_buf != nullptr;
}

int SSCMA_clean_actions()
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=\"\"" CMD_SUFFIX, CMD_AT_ACTION);

    write(cmd, strlen(cmd));
    if (wait(CMD_TYPE_RESPONSE, CMD_AT_ACTION) == CMD_OK)
    {
        return CMD_OK;
    }
    return CMD_ETIMEDOUT;
}

int SSCMA_save_jpeg()
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=\"save_jpeg()\"" CMD_SUFFIX, CMD_AT_ACTION);

    write(cmd, strlen(cmd));
    if (wait(CMD_TYPE_RESPONSE, CMD_AT_ACTION) == CMD_OK)
    {
        return CMD_OK;
    }
    return CMD_ETIMEDOUT;
}
*/