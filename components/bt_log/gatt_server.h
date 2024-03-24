#ifndef GATT_SERVER_H
#define GATT_SERVER_H

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/ble.h"

struct config_t {
    // General
    uint32_t textColor;

    // Kalman Filter
    float processNoise;

    // Sensor
    double accelPos[10];
    double offset;
    int16_t xOffset;
    int16_t yOffset;
    int16_t zOffset;
};

extern uint16_t data_handle;

int gatt_svr_init(void);

#endif