#include <stdint.h>
#include <esp_timer.h>
#include "esp_dsp.h"

#define LSB2DEG 360/65536
#define DEG2LSB 65536/360
#define RAD2LSB 65536/6.28
#define LSB2RAD 6.28/65536
#define ROT2LSB 65536
#define LSB2ROT 1/65536

#define PROCESS_NOISE 0.003 // trust the process

// State of the system
struct systemState{
    uint16_t angle; // angle from the last zero passing (65,536 LSBs per rotation)
    double angularVelocity; // angular velocity measurement in radians per second
    double angularAcceleration; // angular acceleration measurement in radians per second per second
    double variance_A; // variance estimate of the angle measurement
    double variance_AV; // variance estimate of the angular velocity measurement
    double variance_AA; // variance estimate of the angular acceleration measurement
    uint64_t time; // time of state
};

class kalmanFilter {

    public:

        kalmanFilter();

        /**
         * Sets the system state for this cycle
         * @param state the measured state of the system
         */
        void setInitState(systemState state);

        /**
         * Input a measurement to the kalman filter for this cycle
         * @param angle angle in radian from the last zero passing (0 - 2π)
         * @param angularVelocity angular velocity measurement in radians per second
         * @param variance_A variance estimate of the angle measurement
         * @param variance_AV variance estimate of the angular velocity measurement
         */
        void makeMeasurement(double angle, double angularVelocity, double variance_A, double variance_AV);

        /**
         * Input a measurement to the kalman filter for this cycle
         * @param state measured state of the system. Make sure angle is in LSBs
         */
        void makeMeasurement(systemState state);

        /**
         * Completes all calculations for this cycle and returns the current calculated system state
         * @return current state of the system as calculated by the Kalman filter
         */
        systemState stateUpdate();

        void adjustAngle(double angle);
        void adjustVelocity(double velocity);
        void adjustAccel(double accel);

    private:
        systemState estimateState;
        systemState measurmentState;
        double deltaTime;

        /**
         * @brief Predicts the next system state given the current state using a dynamic model
         */
        void predict();

        /**
         * @brief Uses a state measurement and prediction to output the most likely system state
         */
        void update();

        int32_t constrainAngle(int32_t input);

};