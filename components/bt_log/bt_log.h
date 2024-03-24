#ifndef BT_LOG_H
#define BT_LOG_H

#include <stdint.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"

class bt_log {

private:

    void onConnect();
    void onReceive();


public:

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

    bt_log() {};
    ~bt_log() {};

    void init();
    //void setConnectCallback();
    void setReceiveCallback(void(* callback)());
    bool transmit();
};

#endif //BT_LOG_H