/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

typedef void *max30102_handle_t;

typedef struct {
    bool hand_detected;
    float heart_rate;
    float spo2;
} max30102_data_t;

esp_err_t max30102_create(i2c_port_t port, max30102_handle_t *handle);
esp_err_t max30102_config(max30102_handle_t sensor);
esp_err_t max30102_deinit(max30102_handle_t sensor);
esp_err_t max30102_get_data(max30102_handle_t sensor, max30102_data_t *data);

#ifdef __cplusplus
} /* end of extern "C" */
#endif