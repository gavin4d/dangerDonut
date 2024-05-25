#include "gamepad.h"

void (* Gamepad::connectCB)() = nullptr;
void (* Gamepad::disconnectCB)() = nullptr;
uint8_t Gamepad::gamepad_data[16] = {0};

void Gamepad::setConnectCB(void (* callback)()) {
    connectCB = callback;
}

void Gamepad::setDisconnectCB(void (* callback)()) {
    disconnectCB = callback;
}

void Gamepad::setGamepadData(uint8_t* dataptr) {
    memcpy(gamepad_data, dataptr, 16);
}

// sticks
int16_t Gamepad::left_stick_x() {
    return (int16_t)(((gamepad_data[0]>>6) | (gamepad_data[1]<<2)) - 512);
}
int16_t Gamepad::left_stick_y() {
    return (int16_t)(((gamepad_data[2]>>6) | (gamepad_data[3]<<2)) - 512); 
}
int16_t Gamepad::right_stick_x() { 
    return (int16_t)(((gamepad_data[4]>>6) | (gamepad_data[5]<<2)) - 512);
}
int16_t Gamepad::right_stick_y() {
    return (int16_t)(((gamepad_data[6]>>6) | (gamepad_data[7]<<2)) - 512);
}
bool Gamepad::left_stick_button() {
    return (gamepad_data[14] >> 5) & 0x1;
}
bool Gamepad::right_stick_button() {
    return (gamepad_data[14] >> 6) & 0x1;
}

// triggers
uint16_t Gamepad::left_trigger() {
    return (uint16_t)((gamepad_data[8]) | ((gamepad_data[9] & 0x03)<<8));
}
uint16_t Gamepad::right_trigger() {
    return (uint16_t)((gamepad_data[10]) | ((gamepad_data[11] & 0x03)<<8));
}

// bumpers
bool Gamepad::left_bumper() {
    return (gamepad_data[13] >> 6) & 0x1;
}

bool Gamepad::right_bumper() {
    return (gamepad_data[13] >> 7) & 0x1;
}


// d-pad
bool Gamepad::up() {
    return gamepad_data[12] == 8 || gamepad_data[12] == 1 || gamepad_data[12] == 2;
}
bool Gamepad::right() {
    return gamepad_data[12] == 2 || gamepad_data[12] == 3 || gamepad_data[12] == 4;
}
bool Gamepad::down() {
    return gamepad_data[12] == 4 || gamepad_data[12] == 5 || gamepad_data[12] == 6;
}
bool Gamepad::left() {
    return gamepad_data[12] == 6 || gamepad_data[12] == 7 || gamepad_data[12] == 8;
}

// buttons
bool Gamepad::a() {
    return (gamepad_data[13]) & 0x1;
}
bool Gamepad::b() {
    return (gamepad_data[13] >> 1) & 0x1;
}
bool Gamepad::x() {
    return (gamepad_data[13] >> 3) & 0x1;
}
bool Gamepad::y() {
    return (gamepad_data[13] >> 4) & 0x1;
}

// menu buttons
bool Gamepad::share() {
    return (gamepad_data[14] >> 2) & 0x1;
}
bool Gamepad::menu() {
    return (gamepad_data[14] >> 3) & 0x1;
}
bool Gamepad::xbox() {
    return (gamepad_data[14] >> 4) & 0x1;
}




//// NimBLE stuff ////

static NimBLEAdvertisedDevice* advDevice;

static bool doConnect = false;
static bool connected = false;
static uint32_t scanTime = 0; /** scan time in milliseconds, 0 = scan forever */


class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        printf("Connected\n");
        /** After connection we should change the parameters if we don't need fast response times.
         *  These settings are 150ms interval, 0 latency, 450ms timout.
         *  Timeout should be a multiple of the interval, minimum is 100ms.
         *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
         *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 45 * 10ms = 450ms timeout
         */
        pClient->updateConnParams(120,120,0,45);

        if (Gamepad::connectCB != nullptr)
            Gamepad::connectCB();
    }

    void onDisconnect(NimBLEClient* pClient, int reason) {
        connected = false;
        printf("%s Disconnected, reason = %d - Starting scan\n",
               pClient->getPeerAddress().toString().c_str(), reason);
        if (Gamepad::disconnectCB != nullptr)
            Gamepad::disconnectCB();
        NimBLEDevice::getScan()->start(scanTime);
    }

    /********************* Security handled here **********************/

    /** Pairing process complete, we can check the results in connInfo */
    void onAuthenticationComplete(NimBLEConnInfo& connInfo){
        if(!connInfo.isEncrypted()) {
            printf("Encrypt connection failed - disconnecting\n");
            /** Find the client with the connection handle provided in desc */
            NimBLEDevice::getClientByID(connInfo.getConnHandle())->disconnect();
            return;
        }
        printf("authentication complete\n");
    }
};


/** Define a class to handle the callbacks when advertisments are received */
class scanCallbacks: public NimBLEScanCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        printf("Advertised Device found: %s\n", advertisedDevice->toString().c_str());
        if(advertisedDevice->isAdvertisingService(NimBLEUUID("1812")))
        {
            printf("Found Our Service\n");
            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = advertisedDevice;
            /** Ready to connect now */
            doConnect = true;
        }
    }

    /** Callback to process the results of the completed scan or restart it */
    void onScanEnd(NimBLEScanResults results) {
        printf("Scan Ended\n");
    }
};


/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
//    std::string str = "Notification";
//    str += " from ";
//    str += pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();
//    str += ": Service = " + pRemoteCharacteristic->getRemoteService()->getUUID().toString();
//    str += ", Characteristic = " + pRemoteCharacteristic->getUUID().toString();
//    str += ", Value = "; 
//    for (int i = 0; i < length; i++) {
//        str += std::to_string(pData[i]) + " ";
//    }
//    printf("%s\n", str.c_str());
    if (pRemoteCharacteristic->getUUID() == NimBLEUUID("2A4D") && length >= 16) {
        connected = true;
        Gamepad::setGamepadData(pData);
    }
}

/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;

NimBLEClient* pClient = nullptr;

/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer() {
    printf("connecting to server\n");

    /** Check if we have a client we should reuse first **/
    if(NimBLEDevice::getClientListSize()) {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if(pClient){
            if(!pClient->connect(advDevice, false)) {
                printf("Reconnect failed\n");
                return false;
            }
            printf("Reconnected client\n");
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if(!pClient) {
        if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
            printf("Max clients reached - no more connections available\n");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        printf("New client created\n");

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 12 * 10ms = 120ms timeout
         */
        pClient->setConnectionParams(12,12,0,300);
        /** Set how long we are willing to wait for the connection to complete (milliseconds), default is 30000. */
        pClient->setConnectTimeout(10000);

        if (!pClient->connect(advDevice)) {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            printf("Failed to connect, deleted client\n");
            return false;
        }
    }

    if(!pClient->isConnected()) {
        if (!pClient->connect(advDevice)) {
            printf("Failed to connect\n");
            return false;
        }
    }

    printf("Connected to: %s RSSI: %d\n",
          pClient->getPeerAddress().toString().c_str(),
          pClient->getRssi());

    /** Now we can read/write/subscribe the charateristics of the services we are interested in */
    NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteCharacteristic* pChrMap = nullptr;
    NimBLERemoteDescriptor* pDsc = nullptr;

    pSvc = pClient->getService("1812");
    if(pSvc) {     /** make sure it's not null */
        pSvc->getCharacteristics(true); // reload the characteristics because getCharacteristic() doesn't for some reasont
        pChr = pSvc->getCharacteristic("2A4D");
        pChrMap = pSvc->getCharacteristic("2A4B");
    }

    if(pChr && pChrMap) {     /** make sure it's not null */
        pChr->getDescriptors(true);
        if(pChrMap->canRead()) {
            printf("%s Value: %s\n",
            pChrMap->getUUID().toString().c_str(),
            pChrMap->readValue().c_str());
        }

        if(pChr->canNotify()) {
            //if(!pChr->registerForNotify(notifyCB)) {
            if(!pChr->subscribe(true, notifyCB)) {
                /** Disconnect if subscribe failed */
                pClient->disconnect();
                printf("failed to subscribe. Disconnecting.\n");
                return false;
            }
            printf("subscibed to notification.\n");
        }
    }

    else{
        printf("service not found.\n");
    }

    printf("Done with this device!\n");
    return true;
}

void connectTask (void * parameter){
    /** Loop here until we find a device we want to connect to */
    for(;;) {
        if(doConnect) {
            doConnect = false;
            /** Found a device we want to connect to, do it now */
            if(connectToServer()) {
                printf("Success! we should now be getting notifications!\n");
            } else {
                printf("Failed to connect, starting scan\n");
                connected = false;
                NimBLEDevice::getScan()->start(scanTime);
            }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

bool Gamepad::isConnected() {
    return connected;
}

void Gamepad::init() {
    printf("Starting NimBLE Client\n");
    /** Initialize NimBLE, no device name spcified as we are not advertising */
    NimBLEDevice::init("");

    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    // bonding, man in the middle protection, secure connections.
    NimBLEDevice::setSecurityAuth(true, false, true);

    /** Optional: set the transmit power, default is -3db */
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** 12db */

    /** Optional: set any devices you don't want to get advertisments from */
    // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

    /** create new scan */
    NimBLEScan* pScan = NimBLEDevice::getScan();

    /** create a callback that gets called when advertisers are found */
    pScan->setScanCallbacks (new scanCallbacks());

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(400);
    pScan->setWindow(100);

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in milliseconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(scanTime);

    printf("Scanning for peripherals\n");

    xTaskCreate(connectTask, "connectTask", 5000, NULL, 1, NULL);
}

