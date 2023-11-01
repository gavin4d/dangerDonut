#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
#include <DShotRMT.h>
#include <driver/timer.h>
#include <HD107S.h>
#include <ADXL375.h>
#include <uni_log.h>
#include <POVDisplay.h>

#include "orientator.h"

#define LED_COUNT 11U
#define LED_DATA_PIN GPIO_NUM_8
#define LED_CLOCK_PIN GPIO_NUM_18
#define LED_STRIP_RMT_INTR_NUM 11U
#define LED_CLOCK_SPEED 32000000
#define RMT_CHANNEL	RMT_CHANNEL_0

#define LEFT_DSHOT_RMT_CHANNEL RMT_CHANNEL_1
#define LEFT_DSHOT_GPIO GPIO_NUM_9
#define RIGHT_DSHOT_RMT_CHANNEL RMT_CHANNEL_2
#define RIGHT_DSHOT_GPIO GPIO_NUM_10 

#define BATTERY_SENSE GPIO_NUM_12

#define IR_PIN GPIO_NUM_11

#define ADXL_MISO_PIN GPIO_NUM_21
#define ADXL_MOSI_PIN GPIO_NUM_14
#define ADXL_CLOCK_PIN GPIO_NUM_13
#define ADXL_CS_PIN GPIO_NUM_38

#define CONTROLLER_RESPONSE_TIMEOUT 3000000U
#define STICK_DEAD_ZONE 0.1
#define SPIN_POWER_THRESHOLD 0.02
#define DRIVE_SENSITIVITY 0.1
#define TURN_SENSITIVITY 0.015
//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

GamepadPtr myGamepad;
DShotRMT esc_l, esc_r;
bool buttons[5] = {false, false, false, false, false}; // B, <-, ->, ^, v
bool controller_idle = false;
int spinDirection = 1; // 1: clockwise, -1: withershin
double offset = 0.0;
esp_timer_create_args_t create_timer;
esp_timer_handle_t test_timer;

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    if (myGamepad == nullptr) {
        Console.printf("CALLBACK: Gamepad is connected\n");
        //LED1.set_pixel(0, 10, 0, 0);
        //LED1.show();

        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        GamepadProperties properties = gp->getProperties();
        Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                        properties.product_id);
        myGamepad = gp;
        foundEmptySlot = true;
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Gamepad connected, but could not find empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    if (myGamepad == gp) {
        Console.printf("CALLBACK: Gamepad is disconnected\n");
        //LED1.set_pixel(0, 5, 0, 10);
        //LED1.show();
        myGamepad = nullptr;
        foundGamepad = true;
    }

    if (!foundGamepad) {
        Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

void setMotorPower(float left_power, float right_power) {
    //Console.printf("Motor Power: left: %02f, right: %02f\n", left_power, right_power);
    if (left_power < 0) {
        esc_l.sendThrottle(-left_power*1000 + 48);
    } else {
        esc_l.sendThrottle(left_power*1000 + 1048);
    }
    if (right_power < 0) {
        esc_r.sendThrottle(-right_power*1000 + 48);
    } else {
        esc_r.sendThrottle(right_power*1000 + 1048);
    }
}

double velocity_data[1000];
double angle_data[1000];
double time_data[1001];
int readCounter = 0;
int cooldown = 0;
uint8_t decor = 0;
ADXL375 ADXL;
orientator sensor;
HD107S LED;
//esp_timer_handle_t clearLEDsTimer;
POVDisplay display;

void zeroHeadingCallback() {
    double period =  sensor.getPeriod();
    display.clear();
    display.drawSprite(0,1,11);
    display.setPixel(0,10, 0xff0000ff);
    display.setPixel(1,10, 0xff0000ff);
    display.setPixel(0,9, 0xff0000ff);
    display.setPixel(1,9, 0xff0000ff);
    // int RPM = 60000/period;
    // if (RPM >= 1000) display.drawSprite(28, 0, RPM/1000 % 10);
    // if (RPM >= 100) display.drawSprite(35, 0, RPM/100 % 10);
    // if (RPM >= 10) display.drawSprite(43, 0, RPM/10 % 10);
    // display.drawSprite(50, 0, RPM % 10);
    // for (int i = 2; i < 84; i++) {
    //     display.setPixel(i, 10, sensor.getIRData(i*period/84) ? 0xff000000 : 0xff0000ff);
    // }
    display.makeFrame(period, spinDirection);
    // for (int i = 1; i < 11; i++)
    //     LED.setLED(i,  RGBL(0, 255, 0,16));
    // LED.update();
    // esp_timer_start_once(clearLEDsTimer, sensor.getPeriod()/84);
}

void clearLightsCallback(void *args) {
    for (int i = 1; i < 11; i++)
        LED.setLED(i,  RGBL(0, 0, 0,16));
    LED.update();
} 

// Arduino setup function. Runs in CPU 1
void setup() {

    // esp_timer_create_args_t new_timer;
    // new_timer.callback = &clearLightsCallback;
    // new_timer.dispatch_method = ESP_TIMER_TASK;
    // esp_timer_create(&new_timer, &clearLEDsTimer);

    hd107s_config_t LED_config;
    LED_config.dataPin = LED_DATA_PIN;
    LED_config.clockPin = LED_CLOCK_PIN;
    LED_config.numLEDs = LED_COUNT;
    LED_config.clockSpeed = LED_CLOCK_SPEED;
    //LED.setup(LED_config);

    display = POVDisplay(LED_config);
    display.setBrightness(64);
    display.setPixel(0,10, 0xff0000ff);
    display.setPixel(1,10, 0xff0000ff);
    display.setPixel(0,9, 0xff0000ff);
    display.setPixel(1,9, 0xff0000ff);
    //display.drawSprite(40,0,10);

    adxl375_spi_config_t adxl_config;
    adxl_config.clockPin = ADXL_CLOCK_PIN;
    adxl_config.misoPin = ADXL_MISO_PIN;
    adxl_config.mosiPin = ADXL_MOSI_PIN;
    adxl_config.csPin = ADXL_CS_PIN;
    Console.println("testing ADXL");
    if (ADXL.setup(adxl_config)) {
        Console.println("adxl375 sensor found");
    } else {
        Console.println("error with adxl375");
    }

    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    BP32.forgetBluetoothKeys();
    esc_l.install(LEFT_DSHOT_GPIO, LEFT_DSHOT_RMT_CHANNEL);
    esc_r.install(RIGHT_DSHOT_GPIO, RIGHT_DSHOT_RMT_CHANNEL);

    sensor.setup(IR_PIN, ADXL);
    sensor.setCallback(&zeroHeadingCallback);

    //sensor.zeroCrossingTime = esp_timer_get_time();
    //LED.setLED(4, RGBL(255, 255, 255, 16));
    //LED.update();
    
    // for (int i = 0; i < 11; i++) {
    //     LED.setLED(i, RGBL(0,0,0,0));
    // }
}

// Arduino loop function. Runs in CPU 1
double hue = 0;
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    //uint64_t loopStart = esp_timer_get_time();
    BP32.update();


    double logVelocity = 0;
    double logAngle = 0;
    if (cooldown <= 0) {
        sensor.update(logAngle, logVelocity);
        //LED.setLED(4, RGBL(127,0,0, sensor.useAccel ? 8 : 0));
        //LED.setLED(7, RGBL(0,127,0, sensor.useIR ? 8 : 0));
        //LED.setLED(3, RGBL(127,127,0, sensor.useAccel ? 8 : 0));
        //LED.setLED(6, RGBL(0,127,127, sensor.useIR ? 8 : 0));
        cooldown = 4;
    } else {
        cooldown--;
    }
    //ESP_LOGI("orienation", "%f", sensor.getAngle());
    //Console.println(sensor.getPeriod());
    hue += 0.1;
    if (hue >= 360) hue = 0;
    // LED.setLED(0, LED.HSVL(hue, 1, 0.2, 9));

    if (myGamepad && myGamepad->isConnected()) {

        controller_idle = esp_timer_get_time() - myGamepad->getUpdateTime() > CONTROLLER_RESPONSE_TIMEOUT;

        if (!controller_idle) {

            float left_power = 0;
            float right_power = 0;
            float y = -((float)myGamepad->axisY())/512;
            float x = ((float)myGamepad->axisX())/512;
            float r = ((float)myGamepad->axisRX())/512;
            float spin_power = ((float)(myGamepad->throttle() - myGamepad->brake()))/1024;
            spinDirection = copysignf(1.0, spin_power);
            if (abs(spin_power) < SPIN_POWER_THRESHOLD) { // run in tank drive if not spinning
                if (abs(y) > STICK_DEAD_ZONE) {
                    left_power += y * DRIVE_SENSITIVITY;
                    right_power -= y * DRIVE_SENSITIVITY;
                }
                if (abs(r) > STICK_DEAD_ZONE) {
                    left_power += r * TURN_SENSITIVITY;
                    right_power += r * TURN_SENSITIVITY;
                }
            } else { // melty mode
                float hypot = hypotf(x,y);
                if (hypot < STICK_DEAD_ZONE) hypot = 0;
                const float theta = atan2f(y,x) + offset*2*PI;
                const float meltyPower = 0.25*hypot*sin(spinDirection*sensor.getAngle() + theta);
                left_power = spin_power + meltyPower;
                right_power = spin_power - meltyPower;
            }

            if (abs(left_power) > 1 || abs(right_power) > 1) {
                float max_power = max(abs(left_power), abs(right_power));
                left_power /= max_power;
                right_power /= max_power;
            }
            setMotorPower(left_power, right_power);

            if (myGamepad->b()) {
                if (!buttons[0]) {
                    buttons[0] = true;
                    //sensor.fillArray(spin_data);
                    for (int i = 0; i < 1000; i++) {
                        Console.printf("%lf	%lf	%lf	\n", angle_data[i], velocity_data[i], time_data[i+1]-time_data[i]);
                    }
                    Console.print("\n");
                }
            } else {
                buttons[0] = false;
            }

            if (myGamepad->a()) {
                if (readCounter < 1000 && logVelocity > 0.1) {
                    velocity_data[readCounter] = logVelocity;
                    angle_data[readCounter] = logAngle;
                    time_data[readCounter+1] = esp_timer_get_time()/1000;
                    readCounter++;
                }
            }

            //boolean isForwards = sensor.getAngle() < 0.2;
            //boolean isIR = !digitalRead(IR_PIN);
            // for (int i = 1; i < 11; i++)
            //     LED.setLED(i,  RGBL(0, isForwards ? 255 : 0, isIR ? 127 : 0,16));
            // LED.update();
            if (myGamepad->left()) {
                if (!buttons[1]) {
                    buttons[1] = true;
                    //sensor.setOffset(sensor.getOffset() + 0.05);
                    offset = (offset + 0.05)-floor(offset + 0.05);
                    Console.printf("offset: %f\n", sensor.getOffset());
                }
            } else {
                buttons[1] = false;
            }

            if (myGamepad->right()) {
                if (!buttons[2]) {
                    buttons[2] = true;
                    //sensor.setOffset(sensor.getOffset() - 0.05);
                    offset = (offset - 0.05)-floor(offset - 0.05);
                    Console.printf("offset: %f\n", sensor.getOffset());
                }
            } else {
                buttons[2] = false;
            }

            if (myGamepad->up()) {
                if (!buttons[3]) {
                    buttons[3] = true;
                    sensor.setAccelPos(sensor.getAccelPos() + 0.0001);
                    Console.printf("AccelPos: %f\n", sensor.getAccelPos());
                }
            } else {
                buttons[3] = false;
            }

            if (myGamepad->down()) {
                if (!buttons[4]) {
                    buttons[4] = true;
                    sensor.setAccelPos(sensor.getAccelPos() - 0.0001);
                    Console.printf("AccelPos: %f\n", sensor.getAccelPos());
                }
            } else {
                buttons[4] = false;
            }

            if (myGamepad->miscHome()) {
                myGamepad->disconnect();
            }
        } else {
            setMotorPower(0,0);
        }
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    vTaskDelay(1);
    //delay(15);
    //ESP_LOGI("loop time", "%li\n", (long)(esp_timer_get_time()-loopStart));
}
