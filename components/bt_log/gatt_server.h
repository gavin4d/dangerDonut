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

static struct config_t {
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
} config;

static struct data_t {
        // General
        uint64_t time;
        uint32_t packetNumber;
        double batteryVoltage;

        // Kalman Filter
        double angleMeasurment; // radians since last zero crossing
        double velocityMeasurment; // measured velocity
        uint16_t angle; // angle from the last zero passing (65,536 LSBs per rotation)
        double angularVelocity; // angular velocity measurement in radians per second
        double angularAcceleration; // angular acceleration measurement in radians per second per second
        double variance_A; // variance estimate of the angle measurement
        double variance_AV; // variance estimate of the angular velocity measurement
        double variance_AA; // variance estimate of the angular acceleration measurement

        // Sensor Input
        double xAccel;
        double yAccel;
        double zAccel;
        double rotationPeriod;
} data;

extern uint16_t data_handle;

int gatt_svr_init(void);

#endif
