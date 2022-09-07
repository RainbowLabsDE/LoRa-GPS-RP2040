#include <Arduino.h>
// An Arduino Sketch for an GPS-Tracker to work with The Things Network.
// This Sketch uses TinyGPSPlus https://github.com/mikalhart/TinyGPSPlus
// the Rocketscream Low-Power library https://github.com/rocketscream/Low-Power
// and the Arduino LMIC-Library by MCCI Catena https://github.com/mcci-catena/arduino-lmic
// The hardware used is an Afafruit Feather 32u4 Lora with the Ultimate GPS featherwing.
#include <SPI.h>
#include <TinyGPS++.h>
#include <hal/hal.h>
#include <lmic.h>

#include "secrets.h"

// use low power sleep; comment next line to not use low power sleep
//#define SLEEP

#ifdef SLEEP
#include "LowPower.h"
bool next = false;
#endif

const unsigned sendInterval = 5000; // ms

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// const unsigned TX_INTERVAL = 32; // multiple of 8

TinyGPSPlus gps;


void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 8,
    .dio = {/*dio0*/ 9, /*dio1*/ 10, /*dio2*/ LMIC_UNUSED_PIN}

    ,
    .rxtx_rx_active = 0,
    .rssi_cal = 10,
    .spi_freq = 8000000 /* 8 MHz */

};

#pragma pack(push, 1)
typedef struct {
    uint32_t latitude : 24;
    uint32_t longitude : 24;
    int16_t altitude : 16;          // altitude in dm
    uint8_t hdop : 8;               // Horizontal Dim. of Precision 1/20

} payload_t;
#pragma pack(pop)


payload_t payload = {.altitude = INT16_MIN};
// uint8_t coords[9];
// uint32_t LatitudeBinary, LongitudeBinary;
// uint16_t altitudeGps;
// uint8_t hdopGps;
static osjob_t sendjob;

void get_coords() {

    // For one second we parse GPS data and report some key values
    Serial.print("Receiving GPS data for 1s: ");
    for (unsigned long start = millis(); millis() - start < 1000;) {
        while (Serial1.available()) {
            char c = Serial1.read();
            // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
            gps.encode(c);
            Serial.write(c);
        }
    }
    Serial.println();

    if (gps.location.isValid()) {
        payload = {
            .latitude = (uint32_t)(((gps.location.lat() + 90) / 180.0) * 0xFFFFFF),
            .longitude = (uint32_t)(((gps.location.lng() + 180) / 360.0) * 0xFFFFFF),
            .altitude = (int16_t)(gps.altitude.meters() * 10),
            .hdop = (uint8_t)(gps.hdop.value() / 40)    // hdop.value() is hdop in 1/100th and goes up to 9999
        };

        char buf[256];
        snprintf(buf, sizeof(buf), "Lat: %f, Lon: %f, Alt: %f, HDOP: %d\n", gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.hdop.value());
        Serial.write(buf, strlen(buf));
    }
    else {
        digitalWrite(LED_BUILTIN, HIGH); // turn LED back on on GPS fix
    }
}

void do_send(osjob_t *j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        get_coords();
        LMIC_setTxData2(1, (uint8_t*)&payload, sizeof(payload), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        digitalWrite(LED_BUILTIN, LOW); // turn off LED to signal LoRa Join

        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;

    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        //  next = true;
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;
    default:
        Serial.println("Unknown event " + String(ev));
        break;
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn LED on initially
    delay(1000);
    Serial.begin(115200);
    Serial.println(F("Starting"));
    Serial.println(sizeof(payload_t));

    Serial.print("GPS Init... ");
    Serial1.begin(9600);
    Serial1.print("$PMTK301,2*2E\r\n"); // Select SBAS as DGPS source (RTCM)
    Serial1.print("$PMTK313,1*2E\r\n"); // Enable to search a SBAS satellite
    // Serial1.print("$PMTK513,1*28\r\n");
    Serial.println("done.");

    // LMIC init
    Serial.print("LMIC Init... ");
    os_init();
    Serial.println("done.");


    // Reset the MAC state. Session and pending data transfers will be discarded.
    Serial.print("LMIC Reset... ");
    LMIC_reset();
    LMIC_setAdrMode(0);
    
    Serial.println("done.");

    // LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job
    do_send(&sendjob);
}

uint32_t lastSendTime = 0;

void loop() { 
    os_runloop_once(); 

    if (millis() - lastSendTime > sendInterval) {
        lastSendTime = millis();
        Serial.print("Sending... ");
        do_send(&sendjob);
        Serial.println("done.");
    }

    // digitalWrite(LED_BUILTIN, gps.location.isValid()); // turn LED on if GPS lock is achieved
}