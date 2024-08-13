/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>

// BLE Service
BLEDfu bledfu;   // OTA DFU service
BLEDis bledis;   // device information
BLEUart bleuart; // uart over ble
BLEBas blebas;   // battery

const int BUFFSIZE = 10;
uint8_t buf[BUFFSIZE];

// Use built-in buttons if available, else use A0, A1
#ifdef PIN_BUTTON1
#define BUTTON_4 PIN_BUTTON1
#else
#define BUTTON_4 A0
#endif

#ifdef PIN_BUTTON2
#define BUTTON_5 PIN_BUTTON2
#else
#define BUTTON_5 A1
#endif

// Circuit Play Bluefruit has button active state = high
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
uint32_t const button_active_state = HIGH;
#else
uint32_t const button_active_state = LOW;
#endif

// Sensor data
float pre_motionX = 0;
float pre_motionY = 0;
float pre_motionZ = 0;
float motionX;
float motionY;
float motionZ;
float diffX;
float diffY;
float diffZ;

bool isConnected = false;

// floatToBytes(floatValue, buf, 0);
// void floatToBytes(float floatValue, uint8_t* buffer, int startIndex) {
//   // Copy the bytes of the float into the buffer
//   memcpy(&buffer[startIndex], &floatValue, sizeof(float));
// }

void setup() {
    Serial.begin(115200);
    CircuitPlayground.begin();

#if CFG_DEBUG
    // Blocking wait for connection when debug mode is enabled via IDE
    while (!Serial)
        yield();
#endif

    Serial.println("Bluefruit52 BLEUART Example");
    Serial.println("---------------------------\n");

    // pull high for active low, or pull low for active high
    if (button_active_state == HIGH) {
        pinMode(BUTTON_4, INPUT_PULLDOWN);
        pinMode(BUTTON_5, INPUT_PULLDOWN);
    } else {
        pinMode(BUTTON_4, INPUT_PULLUP);
        pinMode(BUTTON_5, INPUT_PULLUP);
    }

    // Setup the BLE LED to be enabled on CONNECT
    // Note: This is actually the default behavior, but provided
    // here in case you want to control this LED manually via PIN 19
    Bluefruit.autoConnLed(true);

    // Config the peripheral connection with maximum bandwidth
    // more SRAM required by SoftDevice
    // Note: All config***() function must be called before begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

    Bluefruit.begin();
    Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
    // Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // To be consistent OTA DFU should be added first if it exists
    bledfu.begin();

    // Configure and Start Device Information Service
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather52");
    bledis.begin();

    // Configure and Start BLE Uart Service
    bleuart.begin();

    // Start BLE Battery Service
    blebas.begin();
    blebas.write(100);

    // Set up and start advertising
    startAdv();

    Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
    Serial.println("Once connected, enter character(s) that you wish to send");
}

void startAdv(void) {
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include bleuart 128-bit uuid
    Bluefruit.Advertising.addService(bleuart);

    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();

    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     *
     * For recommended advertising interval
     * https://developer.apple.com/library/content/qa/qa1931/_index.html
     */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
    Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void loop() {

    // Forward data from HW Serial to BLEUART
    while (Serial.available()) {
        // Delay to wait for enough input, since we have a limited transmission buffer
        delay(2);
        // // read from serial then send
        // uint8_t buf[20];
        // int count = Serial.readBytes(buf, sizeof(buf));
        // bleuart.write(buf, count);
        // Serial.println(count);
    }
    // Forward from BLEUART to HW Serial
    while (bleuart.available()) {
        uint8_t ch;
        ch = (uint8_t)bleuart.read();
        Serial.write(ch);
    }

    // // !
    buf[0] = 33;
    // // A
    buf[1] = 65;
    motionX = CircuitPlayground.motionX() * -1;
    motionY = CircuitPlayground.motionY() * -1;
    motionZ = CircuitPlayground.motionZ();

    if (motionX < 0) {
        buf[2] = 45;
    } else {
        buf[2] = 43;
    }
    buf[3] = (int)abs(motionX);

    if (motionY < 0) {
        buf[4] = 45;
    } else {
        buf[4] = 43;
    }
    buf[5] = (int)abs(motionY);

    if (motionZ < 0) {
        buf[6] = 45;
    } else {
        buf[6] = 43;
    }
    buf[7] = (int)abs(motionZ);

    Serial.print("Acc: ");
    Serial.print(motionX);
    Serial.print(' ');

    Serial.print(motionY);
    Serial.print(' ');

    Serial.print(motionZ);
    Serial.print(' ');

    if (CircuitPlayground.leftButton()) {
        Serial.print("Left button pressed!");
        // !
        buf[0] = 33;
        // L
        buf[1] = 76;
        bleuart.write(buf, BUFFSIZE);
    }

    if (CircuitPlayground.rightButton()) {
        Serial.print("Right button pressed!");
        // !
        buf[0] = 33;
        // R
        buf[1] = 82;
        bleuart.write(buf, BUFFSIZE);
    }
    buf[10] = 255;
    bleuart.write(buf, BUFFSIZE);
    if (isConnected) {
        Serial.print("MSG: ");
        for (int i = 0; i < BUFFSIZE; i++) {
            Serial.print(buf[i], HEX);
            Serial.print(' ');
        }
    }

    Serial.println();

    delay(150);
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle) {
    // Get the reference to current connection
    BLEConnection *connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = {0};
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);

    isConnected = true;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;

    Serial.println();
    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);

    isConnected = false;
}
