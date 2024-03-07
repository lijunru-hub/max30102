/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "max30102.h"

static const char *TAG = "max30102";

/**
 * @brief MAX30102 I2C Address
 *
 */
#define MAX30102_I2C_ADDR           0x57 /*!< max30102 i2c address */

/**
 * @brief chip register definition
 */
#define MAX30102_REG_INTERRUPT_STATUS_1          0x00        /**< interrupt status 1 register */
#define MAX30102_REG_INTERRUPT_STATUS_2          0x01        /**< interrupt status 2 register */
#define MAX30102_REG_INTERRUPT_ENABLE_1          0x02        /**< interrupt enable 1 register */
#define MAX30102_REG_INTERRUPT_ENABLE_2          0x03        /**< interrupt enable 2 register */
#define MAX30102_REG_FIFO_WRITE_POINTER          0x04        /**< fifo write pointer register */
#define MAX30102_REG_OVERFLOW_COUNTER            0x05        /**< overflow counter register */
#define MAX30102_REG_FIFO_READ_POINTER           0x06        /**< fifo read pointer register */
#define MAX30102_REG_FIFO_DATA_REGISTER          0x07        /**< fifo data register */
#define MAX30102_REG_FIFO_CONFIG                 0x08        /**< fifo config register */
#define MAX30102_REG_MODE_CONFIG                 0x09        /**< mode config register */
#define MAX30102_REG_SPO2_CONFIG                 0x0A        /**< spo2 config register */
#define MAX30102_REG_LED1_PA                     0x0C        /**< led pulse amplitude 1 register */
#define MAX30102_REG_LED2_PA                     0x0D        /**< led pulse amplitude 2 register */
#define MAX30102_REG_MULTI_LED_MODE_CONTROL_1    0x11        /**< multi led mode control 1 register */
#define MAX30102_REG_MULTI_LED_MODE_CONTROL_2    0x12        /**< multi led mode control 2 register */
#define MAX30102_REG_DIE_TEMP_INTEGER            0x1F        /**< die temperature integer register */
#define MAX30102_REG_DIE_TEMP_FRACTION           0x20        /**< die temperature fraction register */
#define MAX30102_REG_DIE_TEMP_CONFIG             0x21        /**< die temperature config register */
#define MAX30102_REG_REVISION_ID                 0xFE        /**< revision id register */
#define MAX30102_REG_PART_ID                     0xFF        /**< part id register */

typedef struct
{
    i2c_port_t i2c_port;
    uint16_t dev_addr;
    float meastime;
    float lastmeastime;
    float firxv[5];
    float firyv[5];
    float fredxv[5];
    float fredyv[5];
    float hrarray[10];
    float spo2array[10];
    int hrarraycnt;
} max30102_dev_t;

static esp_err_t max30102_read(max30102_dev_t *dev, uint8_t reg_addr, uint8_t *data_buf, const uint8_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ret == ESP_OK);
    ret = i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_WRITE, true);
    assert(ret == ESP_OK);
    ret = i2c_master_write_byte(cmd, reg_addr, true);
    assert(ret == ESP_OK);
    ret = i2c_master_start(cmd);
    assert(ret == ESP_OK);
    ret = i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_READ, true);
    assert(ret == ESP_OK);
    if (len > 1) {
        ret = i2c_master_read(cmd, data_buf, len - 1, I2C_MASTER_ACK);
        assert(ret == ESP_OK);
    }
    ret = i2c_master_read_byte(cmd, data_buf + len -1, I2C_MASTER_NACK);
    assert(ret == ESP_OK);
    ret = i2c_master_stop(cmd);
    assert(ret == ESP_OK);
    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    assert(ret == ESP_OK);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t max30102_write(max30102_dev_t *dev, const uint8_t reg_addr, const uint8_t *const data_buf, const uint8_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ret == ESP_OK);
    ret = i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_WRITE, true);
    assert(ret == ESP_OK);
    ret = i2c_master_write_byte(cmd, reg_addr, true);
    assert(ret == ESP_OK);
    ret = i2c_master_write(cmd, data_buf, len, true);
    // ret = i2c_master_write_byte(cmd, *data_buf, true);
    assert(ret == ESP_OK);
    ret = i2c_master_stop(cmd);
    assert(ret == ESP_OK);
    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    assert(ret == ESP_OK);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t max30102_create(i2c_port_t port, max30102_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invoid handle");
    max30102_dev_t *max30102_dev = (max30102_dev_t *)calloc(1, sizeof(max30102_dev_t));
    ESP_RETURN_ON_FALSE(max30102_dev != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for max30102_dev_t");
    max30102_dev->i2c_port = port;
    max30102_dev->dev_addr = MAX30102_I2C_ADDR;

    *handle = max30102_dev;
    return ESP_OK;
}

esp_err_t max30102_config(max30102_handle_t sensor)
{
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid sensor handle");
    max30102_dev_t *dev = (max30102_dev_t *) sensor;

    esp_err_t ret;
    uint8_t data;
    data = (0x2 << 5); // sample averaging 0=1,1=2,2=4,3=8,4=16,5+=32
    ret = max30102_write(dev, MAX30102_REG_FIFO_CONFIG, &data, 1);
    assert(ret == ESP_OK);
    data = 0x03; // mode = red and ir samples
    ret = max30102_write(dev, MAX30102_REG_MODE_CONFIG, &data, 1);
    assert(ret == ESP_OK);
    data = (0x3 << 5) + (0x3 << 2) + 0x3; // first and last 0x3, middle smap rate 0=50,1=100,etc
    ret = max30102_write(dev, MAX30102_REG_SPO2_CONFIG, &data, 1);
    assert(ret == ESP_OK);
    data = 0xd0; // ir pulse power
    ret = max30102_write(dev, MAX30102_REG_LED1_PA, &data, 1);
    assert(ret == ESP_OK);
    data = 0xa0; // red pulse power
    ret = max30102_write(dev, MAX30102_REG_LED2_PA, &data, 1);
    return ret;
}

esp_err_t max30102_deinit(max30102_handle_t sensor)
{
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid sensor handle");
    max30102_dev_t *dev = (max30102_dev_t *) sensor;
    free(dev);
    return ESP_OK;
}

esp_err_t max30102_get_data(max30102_handle_t sensor, max30102_data_t *data)
{
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid sensor handle");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid data pointer");

    max30102_dev_t *dev = (max30102_dev_t *) sensor;

    esp_err_t ret = ESP_OK;
    uint8_t rptr, wptr = 0;
    int samp;
    uint8_t reg_data[256] = {0};
    ret = max30102_read(dev, MAX30102_REG_FIFO_WRITE_POINTER, &wptr, 1);
    assert(ret == ESP_OK);
    ret = max30102_read(dev, MAX30102_REG_FIFO_READ_POINTER, &rptr, 1);
    samp = ((32 + wptr) - rptr) % 32;
    ret = max30102_read(dev, MAX30102_REG_FIFO_DATA_REGISTER, reg_data, 6 * samp);
    for (int i = 0; i < samp; i++) {
        dev->meastime += 0.01;

        dev->firxv[0] = dev->firxv[1];
        dev->firxv[1] = dev->firxv[2];
        dev->firxv[2] = dev->firxv[3];
        dev->firxv[3] = dev->firxv[4];
        //(1/3.48311) coefficient 2048->587
        dev->firxv[4] = (1 / 3.48311) * (256 * 256 * (reg_data[6 * i + 3] % 4) + 256 * reg_data[6 * i + 4] + reg_data[6 * i + 5]);

        dev->firyv[0] = dev->firyv[1];
        dev->firyv[1] = dev->firyv[2];
        dev->firyv[2] = dev->firyv[3];
        dev->firyv[3] = dev->firyv[4];
        dev->firyv[4] = (dev->firxv[0] + dev->firxv[4]) - 2 * dev->firxv[2] + (-0.1718123813 * dev->firyv[0]) + (0.3686645260 * dev->firyv[1]) + (-1.1718123813 * dev->firyv[2]) + (1.9738037992 * dev->firyv[3]);

        dev->fredxv[0] = dev->fredxv[1];
        dev->fredxv[1] = dev->fredxv[2];
        dev->fredxv[2] = dev->fredxv[3];
        dev->fredxv[3] = dev->fredxv[4];
        dev->fredxv[4] = (1 / 3.48311) * (256 * 256 * (reg_data[6 * i + 0] % 4) + 256 * reg_data[6 * i + 1] + reg_data[6 * i + 2]);

        dev->fredyv[0] = dev->fredyv[1];
        dev->fredyv[1] = dev->fredyv[2];
        dev->fredyv[2] = dev->fredyv[3];
        dev->fredyv[3] = dev->fredyv[4];
        dev->fredyv[4] = (dev->fredxv[0] + dev->fredxv[4]) - 2 * dev->fredxv[2] + (-0.1718123813 * dev->fredyv[0]) + (0.3686645260 * dev->fredyv[1]) + (-1.1718123813 * dev->fredyv[2]) + (1.9738037992 * dev->fredyv[3]);

        if (-1.0 * dev->firyv[4] >= 100 && -1.0 * dev->firyv[2] > -1 * dev->firyv[0] && -1.0 * dev->firyv[2] > -1 * dev->firyv[4] && dev->meastime - dev->lastmeastime > 0.5) {
            dev->hrarray[dev->hrarraycnt % 5] = 60 / (dev->meastime - dev->lastmeastime);
            dev->spo2array[dev->hrarraycnt % 5] = 110 - 25 * ((dev->fredyv[4] / dev->fredxv[4]) / (dev->firyv[4] / dev->firxv[4]));

            if (dev->spo2array[dev->hrarraycnt % 5] > 100) {
                dev->spo2array[dev->hrarraycnt % 5] = 99.9;
            }
            dev->lastmeastime = dev->meastime;
            dev->hrarraycnt++;

            data->heart_rate = (dev->hrarray[0] + dev->hrarray[1] + dev->hrarray[2] + dev->hrarray[3] + dev->hrarray[4]) / 5;
            if (data->heart_rate < 40 || data->heart_rate > 150) {
                data->heart_rate = 0;
            }
            data->spo2 = (dev->spo2array[0] + dev->spo2array[1] + dev->spo2array[2] + dev->spo2array[3] + dev->spo2array[4]) / 5;
            if (data->spo2 < 50 || data->spo2 > 101) {
                data->spo2 = 0;
            }
            data->hand_detected = true;
        } else {
            data->hand_detected = false;
            data->heart_rate = 0;
            data->spo2 = 0;
        }
    }
    return ESP_OK;
}
