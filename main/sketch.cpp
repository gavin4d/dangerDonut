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
#include <Preferences.h>
#include <driver/adc.h>
#include <QuickPID.h>
#include <bt_log.h>

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
#define SPIN_THRESHOLD 0.02//13
#define DRIVE_SENSITIVITY 0.1
#define TURN_SENSITIVITY 0.015
#define MELTY_SENSITIVITY 0.25
#define MELTY_TURN_SENSITIVITY 6
#define MAX_ACCELERATION 150
#define JERK_COMPENSATION 400

#define NUM_DECORATIONS 4

GamepadPtr myGamepad;
DShotRMT esc_l, esc_r;
bool buttons[9] = {false, false, false, false, false, false, false, false, false}; // B, <-, ->, ^, v, lb, rb, home, y
bool controller_idle = false;
int spinDirection = 1; // 1: clockwise, -1: withershin
double spin_power = 0;
double offset[2] = {-0.075, 0.15};
esp_timer_create_args_t create_timer;
esp_timer_handle_t test_timer;

double velocity_data[1000];
double velocity_e_data[1000];
double angle_data[1000];
double angle_e_data[1000];
double accel_data[1001];
int readCounter = 0;
int cooldown = 0;
int8_t decor = 0;
float animation = 0;
float disconnectedLEDPos = 0;
int64_t loopStart = 0;
Preferences preferences;
ADXL375 ADXL;
orientator sensor;
HD107S LED;
POVDisplay display;
double targetSpeed = 0;
int flipped = 1;
float pidInput, pidOutput, pidSetPoint;
float Kp = 0.004, Ki = 0.005, Kd = 0.0001; 
//QuickPID pid(&pidInput, &pidOutput, &pidSetPoint);

float getBattVoltage() {
    int voltageReading = 0;
    adc2_get_raw(ADC2_CHANNEL_1, ADC_WIDTH_12Bit, &voltageReading);
    return (float)voltageReading * 1.90 / 4095 * 4.4;
}

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    if (myGamepad == nullptr) {
        Console.printf("CALLBACK: Gamepad is connected\n");

        display.clearStrip();
        for (int i = 0; i < 11; i++)
            display.setStripPixel(i, 0xff00ff00);
        display.makeLEDStrip();

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
        esc_l.sendThrottle(-left_power*999 + 48);
    } else {
        esc_l.sendThrottle(left_power*999 + 1048);
    }
    if (right_power < 0) {
        esc_r.sendThrottle(-right_power*999 + 48);
    } else {
        esc_r.sendThrottle(right_power*999 + 1048);
    }
}

void zeroHeadingCallback() {
    double period = sensor.getPeriod();
    if (period < 10 || period >= 500) return;
    int RPM = 60000/period; // min((int)round(sensor.getVelocity()/ACCEL_POS_SPREAD), NUM_ACCEL_POS-1);
    display.clear();
    uint32_t color = 0xFF2255FF;
    if (getBattVoltage() < 6.5) color = 0xFFFF2222;
    switch (decor) {
    case 0: // RPM
        if (RPM >= 1000) display.drawDigit(28, 0, RPM/1000 % 10, color);
        if (RPM >= 100) display.drawDigit(35, 0, RPM/100 % 10, color);
        if (RPM >= 10) display.drawDigit(43, 0, RPM/10 % 10, color);
        display.drawDigit(50, 0, RPM % 10, color);
        break;
    case 1: // donut
        display.drawSprite(0,0,11);
        break;
    case 2: // nyan cat
        if (animation < 0) animation = 83;
        if (((int)(animation + 2) % 12) < 4) display.drawSprite(20, 4, 17-((int)(animation + 2) % 12));
        if (((int)(animation + 8) % 12) < 4) display.drawSprite(64, 0, 17-((int)(animation + 8) % 12));
        if (((int)(animation + 4) % 12) < 4) display.drawSprite(47, 4, 17-((int)(animation + 4) % 12));
        if (((int)(animation + 6) % 12) < 4) display.drawSprite(0, 2, 17-((int)(animation + 6) % 12));
        if(((int)animation % 8) >= 4) {
            display.drawSprite(animation,0,12);
        } else {
            display.drawSprite(animation,0,13);
        }
        animation -= period/84;
        break;
    case 3: // pride flags
        if (animation >= 9) animation = 0;
        for (int i = 0; i < 84; i++)
            display.drawSprite(i, 0, 18 + (int)animation);
        animation += period/4000;
        break;
    default: // debug
        display.drawSprite(40,0,10);
        break;
    }

    for (int i = 2; i < 84; i++) {
        if (!sensor.getIRData(i*period/84)) {
            display.setPixel(i, 9, 0xff00ff00);
            display.setPixel(i, 10, 0xff00ff00);
        }
    }
    display.setPixel(0,10, 0xff0000ff);
    display.setPixel(1,10, 0xff0000ff);
    display.setPixel(0,9, 0xff0000ff);
    display.setPixel(1,9, 0xff0000ff);
    display.setPixel(0,8, 0xff0000ff);
    display.setPixel(1,8, 0xff0000ff);
    display.makeFrame(period, spinDirection);
}

void onStopCallback() {
    display.clearStrip();
    for (int i = 0; i < 11; i++)
        display.setStripPixel(i, 0xff00ff00);
    display.makeLEDStrip();
}

float velocityFollow(float current, float target, float deltaTime) {
    float step = deltaTime*MAX_ACCELERATION;
    if (abs(target - current) >= step) {
        return current + copysignf(step, target-current);
    }
    return target;
}

// Arduino setup function. Runs in CPU 1
void setup() {

    preferences.begin("config");
    // preferences.putInt("XOffset", -9);
    // preferences.putInt("YOffset", -6);
    // preferences.putInt("ZOffset", 15);

    Console.println("Offsets:");
    Console.printf("X: %d\n", preferences.getInt("XOffset"));
    Console.printf("Y: %d\n", preferences.getInt("YOffset"));
    Console.printf("Z: %d\n", preferences.getInt("ZOffset"));

    hd107s_config_t LED_config;
    LED_config.dataPin = LED_DATA_PIN;
    LED_config.clockPin = LED_CLOCK_PIN;
    LED_config.numLEDs = LED_COUNT;
    LED_config.clockSpeed = LED_CLOCK_SPEED;
    //LED.setup(LED_config);

    display = POVDisplay(LED_config);
    display.setBrightness(64);

    adxl375_spi_config_t adxl_config;
    adxl_config.clockPin = ADXL_CLOCK_PIN;
    adxl_config.misoPin = ADXL_MISO_PIN;
    adxl_config.mosiPin = ADXL_MOSI_PIN;
    adxl_config.csPin = ADXL_CS_PIN;
    adxl_config.xOffset = preferences.getInt("XOffset", 0);
    adxl_config.yOffset = preferences.getInt("YOffset", 0);
    adxl_config.zOffset = preferences.getInt("ZOffset", 0);

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
    sensor.setZeroCrossCallback(&zeroHeadingCallback);
    sensor.setOnStopCallback(&onStopCallback);

    char readLocation[10] = "AccelPos ";
    Console.println("Accel config:");
    for (int i = 0; i < NUM_ACCEL_POS; i++) {
        readLocation[8] = i+'0';
        sensor.setAccelPos(preferences.getDouble(readLocation, 0.03), i);
        Console.printf("%s: %f\n",readLocation , preferences.getDouble(readLocation));
    }

    adc2_config_channel_atten(ADC2_CHANNEL_1, ADC_ATTEN_6db);

    //pidSetPoint = 0;
    //pid.SetTunings(Kp, Ki, Kd);
    //pid.SetMode(pid.Control::automatic);
    //pid.SetOutputLimits(0,1);
    //pid.SetSampleTimeUs(4000);
}

// Arduino loop function. Runs in CPU 1
void loop() {

    double deltaTime = ((double)(esp_timer_get_time() - loopStart))/1000000;
    loopStart = esp_timer_get_time();
    BP32.update();

    if (myGamepad && myGamepad->isConnected()) {

        controller_idle = esp_timer_get_time() - myGamepad->getUpdateTime() > CONTROLLER_RESPONSE_TIMEOUT;

        double logVelocity = 0;
        double logAngle = 0;
        double logAngle_e = 0;
        double logVelocity_e = 0;
        double logAccel_e = 0;
        sensor.update(logAngle, logVelocity, logAngle_e, logVelocity_e, logAccel_e);
        //ESP_LOGI("imu", "x: %f, y: %f, z: %f", -sensor.getXAccel()/LSB2G_MULTIPLIER, sensor.getYAccel()/LSB2G_MULTIPLIER, -sensor.getZAccel()/LSB2G_MULTIPLIER);

        //Console.printf("%03f\n", sensor.getZSign());
        //pidInput = sensor.getVelocity();
        double previousPower = pidOutput;
        //pid.Compute();
        if (sensor.getVelocity() < 75)
            flipped = sensor.getZSign();

        if (!controller_idle) {

            float left_power = 0;
            float right_power = 0;
            float y = -((float)myGamepad->axisY())/512;
            float x = ((float)myGamepad->axisX())/512;
            float r = ((float)myGamepad->axisRX())/512;

            //targetSpeed = velocityFollow(targetSpeed, VELOCITY_MAX*(float)(myGamepad->throttle() - myGamepad->brake())/1024, deltaTime);
            //pidSetPoint = abs(targetSpeed);

            pidOutput = (float)(myGamepad->throttle() - myGamepad->brake())/1024;
            spinDirection = flipped*copysignf(1.0, pidOutput);
            pidOutput = copysignf(pidOutput, spinDirection);

            if (abs(pidOutput) < SPIN_THRESHOLD) { // run in tank drive if not spinning
                if (abs(y) > STICK_DEAD_ZONE) {
                    left_power += y * DRIVE_SENSITIVITY;
                    right_power -= y * DRIVE_SENSITIVITY;
                }
                if (abs(r) > STICK_DEAD_ZONE) {
                    left_power += r * TURN_SENSITIVITY;
                    right_power += r * TURN_SENSITIVITY;
                }
            } else { // melty mode
                sensor.adjustAccel((pidOutput - previousPower)*JERK_COMPENSATION);
                //ESP_LOGI("pid", "setPoint: %f, input: %f, output: %f", pidSetPoint, pidInput, pidOutput);
                float hypot = hypotf(x,y);
                if (hypot < STICK_DEAD_ZONE) hypot = 0;
                float theta = atan2f(y,x) + offset[(spinDirection+1)/2]*2*PI;
                if (flipped < 0) theta += PI; // reverse controls when flipped
                const float meltyPower = MELTY_SENSITIVITY*hypot*sin(spinDirection*sensor.getAngle() + theta);
                left_power = pidOutput + meltyPower;
                right_power = pidOutput - meltyPower;

                if (abs(r) > STICK_DEAD_ZONE) {
                    double turnAmount = -MELTY_TURN_SENSITIVITY*r*deltaTime;
                    sensor.adjustAngle(spinDirection*turnAmount);
                }
            }

            if (abs(left_power) > 1 || abs(right_power) > 1) {
                float max_power = max(abs(left_power), abs(right_power));
                left_power /= max_power;
                right_power /= max_power;
            }

            if (myGamepad->x()) { // self right using wheel's inertia
                left_power = 1;
                right_power = -1;
            }

            if (myGamepad->up()) {
                double angle = atan2(sensor.getYAccel(), sensor.getXAccel());
                if (angle < -PI/4 || angle > 3*PI/4) {
                    left_power = -1;
                    right_power = 1;
                } else {
                    left_power = 1;
                    right_power = -1;
                }
            }

            if (myGamepad->down()) {
                double angle = atan2(sensor.getYAccel(), sensor.getXAccel());
                if (angle < -PI/4 || angle > 3*PI/4) {
                    left_power = 1;
                    right_power = -1;
                } else {
                    left_power = -1;
                    right_power = 1;
                }
            }

            setMotorPower(left_power, right_power);

            if (myGamepad->b()) {
                if (!buttons[0]) {
                    buttons[0] = true;
                    for (int i = 0; i < 1000; i++) {
                        Console.printf("%lf	%lf	%lf	%lf	%lf 	\n", angle_data[i], velocity_data[i], angle_e_data[i], velocity_e_data[i], accel_data[i]);
                    }
                    Console.print("\n");
                }
            } else {
                buttons[0] = false;
            }

            if (myGamepad->a()) {
                if (readCounter >= 1000) readCounter = 0;
                if (/*readCounter < 1000 && */logVelocity > 0.1) {
                    velocity_data[readCounter] = logVelocity;
                    velocity_e_data[readCounter] = logVelocity_e;
                    angle_data[readCounter] = logAngle;
                    angle_e_data[readCounter] = logAngle_e;
                    accel_data[readCounter] = logAccel_e;
                    readCounter++;
                }
            }

            if (myGamepad->l1()) {
                if (!buttons[5]) {
                    buttons[5] = true;
                    animation = 0;
                    decor --;
                    if (decor < 0)
                        decor = NUM_DECORATIONS-1;
                }
            } else {
                buttons[5] = false;
            }

            if (myGamepad->r1()) {
                if (!buttons[6]) {
                    buttons[6] = true;
                    animation = 0;
                    decor ++;
                    if (decor >= NUM_DECORATIONS)
                        decor = 0;
                }
            } else {
                buttons[6] = false;
            }

            if (myGamepad->left()) {
                if (!buttons[1]) {
                    buttons[1] = true;
                    sensor.setAccelPos(sensor.getAccelPos() - 0.00005);
                    Console.printf("AccelPos: %f\n", sensor.getAccelPos());
                }
            } else {
                buttons[1] = false;
            }

            if (myGamepad->right()) {
                if (!buttons[2]) {
                    buttons[2] = true;
                    sensor.setAccelPos(sensor.getAccelPos() + 0.00005);
                    Console.printf("AccelPos: %f\n", sensor.getAccelPos());
                }
            } else {
                buttons[2] = false;
            }

            if (myGamepad->miscHome()) {
                if (!buttons[7]) {
                    buttons[7] = true;
                    setMotorPower(0,0);
                    myGamepad->disconnect();
                }
            } else {
                buttons[7] = false;
            }

            if (myGamepad->y()) {
                if (!buttons[8]) {
                    buttons[8] = true;
                    char saveLocation[10] = "AccelPos ";
                    for (int i = 0; i < NUM_ACCEL_POS; i ++) {
                        saveLocation[8] = i+'0';
                        preferences.putDouble(saveLocation, sensor.getAccelPos(i));
                        Console.print(saveLocation);
                        Console.printf(": %f\n", sensor.getAccelPos(i));
                    }
                }
            } else {
                buttons[8] = false;
            }
        } else {
            //pidSetPoint = velocityFollow(pidSetPoint, 0, deltaTime);
            //pid.SetOutputSum(0);
            setMotorPower(0,0);
        }
    } else {
        setMotorPower(0,0);
        sensor.stopZeroCrossCallback();
        display.clearStrip();
        display.setStripPixel(floor(abs(disconnectedLEDPos-9)), 0xffff0000);
        display.makeLEDStrip();
        disconnectedLEDPos += 0.03;
        if (disconnectedLEDPos >= 19)
            disconnectedLEDPos = 0;
    }
    vTaskDelay(1);
}
