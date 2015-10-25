// Jetpack for devhouse Spindle.
// https://github.com/WeAreSpindle/Jetpacks
// By: herman@kopinga.nl, bob.voorneveld@wearespindle.com
// BSD license

// Stands of the shoulders of:
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
// Example sketch demonstrating the graphics capabilities of the SSD1331 library
// Adafruit LSM9DS0 9 DOF Board AHRS Example
// Adafruit Bluefruit Low Energy nRF8001 Print echo demo
// Adafruit GPS library basic test!
// Adafruit FONA basic test

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h> // Maybe we can get rid of this one?
#include "Adafruit_FONA.h"
#include "Adafruit_BLE_UART.h"
#include <stdio.h> // for function sprintf
#include "keys.h"
#include "Location.h"
#include <Bounce.h>

/**********
 FONA
***********/
#define FONA_RST 4
char replybuffer[255];
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint16_t battpercent = 0;

/**********
 BLUETOOTH
***********/
// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 15
#define ADAFRUITBLE_RDY 17     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 16

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
// Constantly checks for new events on the nRF8001
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

/**********
 GPS
***********/
// You should make the following connections with the Due and GPS module:
// GPS power pin to Arduino Due 3.3V output.
// GPS ground pin to Arduino Due ground.
// For hardware serial 1 (recommended):
//   GPS TX to Arduino Due Serial1 RX pin 19
//   GPS RX to Arduino Due Serial1 TX pin 18
#define GPSSerial Serial3

Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false

/**********
 OLED
***********/
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Display pins
// You can use any (4 or) 5 pins
#define sclk 20
#define mosi 21
#define cs   10
#define rst  9
#define dc   5

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define GREY            0x38E7

// Option 1: use any pins but a little slower
//Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, rst);

/*******************
ATTINY i2c leddriver
*******************/
const int ledAddress = 0x26;

/*************
 Housekeeping
*************/
#define STATUSLED 2
#define HEADLIGHT 6
#define BUTTONPIN0 22
#define BUTTONPIN1 23
#define LIGHTSENSOR A0
uint32_t timer = millis();
Bounce button0 = Bounce(BUTTONPIN0, 10);
Bounce button1 = Bounce(BUTTONPIN1, 10);
int largebatt = 0;
uint32_t currentMillis = 0;
unsigned long blinkBreak = 1000;
unsigned long previousBlink = 0;
unsigned long headlightBreak = 50;
unsigned long previousHeadlight = 0;

/*************
    SETUP
*************/
void setup(void) {
  // This boudrate is ignored by Teensy, always runs at full USB speed.
  Serial.begin(115200);
  BTLEserial.begin();  // ToDo: needs to initialize before display. Fix: learn SSD1331 driver proper transactions.

  // Initialize display first
  display.begin();
  display.fillScreen(BLACK);
  display.setTextColor(YELLOW);
  display.print("Spindle");
  display.setTextColor(WHITE, BLACK);
  display.print(" Jetpack\n\n");

  display.print("Serial... OK\n");
  display.print("Bluetooth... OK\n");

  Serial.println(F("Initializing FONA.... (May take 3 seconds)"));

  display.print("Fona...");

  Serial1.begin(4800); // FONA if you're using hardware serial

  // See if the FONA is responding
  if (! fona.begin(Serial1)) {           // can also try fona.begin(Serial1)
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  display.print(" OK\n");
  display.print("GPS... ");
  gpsSetup();
  display.print(" OK\n");
  
  display.print("9DOF... ");
  nineDOFSetup();
  display.print("OK");

  // Housekeeping
  pinMode(BUTTONPIN0, INPUT_PULLUP);
  pinMode(BUTTONPIN1, INPUT_PULLUP);
  pinMode(LIGHTSENSOR, INPUT);
  pinMode(STATUSLED, OUTPUT);
  pinMode(HEADLIGHT, OUTPUT);

  delay(2000);
  clearMiddle();
  display.setCursor(0,18);
  display.print("Setup OK");
  delay(2000);
  clearMiddle();
}

void loop() {
  // Millis is used multiple times in the loop, save it locally :)
  currentMillis = millis();

  // Buttons!
  button0.update();
  button1.update();

  if (button0.fallingEdge()) {
    clearMiddle();
    largebatt = !largebatt;
    updateDisplay();
  }
  if (button1.fallingEdge()) {
    Wire.beginTransmission(ledAddress);
    Wire.write(0x4);
    Wire.endTransmission();
  }
  if (button1.risingEdge()) {
    Wire.beginTransmission(ledAddress);
    Wire.write(0x0);
    Wire.endTransmission();
  }

  // Blink internal LED: working.
  if ((unsigned long)(currentMillis - previousBlink) >= blinkBreak) {
    digitalWrite(STATUSLED, !digitalRead(STATUSLED));
    previousBlink = currentMillis;
  }

  // Headlight code.
  if ((unsigned long)(currentMillis - previousHeadlight) >= headlightBreak) {
    analogWrite(HEADLIGHT, map(analogRead(LIGHTSENSOR), 0, 500, 128, 0));
    previousHeadlight = currentMillis;
  }

  // loop trough inputs
  nineDOFLoop();
  gpsLoop();

  // if millis() or timer wraps around, we'll just reset it
  if (timer > currentMillis)  timer = currentMillis;

  // approximately every 10 seconds or so, print out the current stats and sent the location
  if (currentMillis - timer > 10000) {
    updateDisplay();
    displayGPSDebugInfo();
    sendLocation();
    // this all took some time, so set currentMillis again
    currentMillis = millis();
  }

  if (Serial.available()) {
    doFona();
  }

  /**********
   BLUETOOTH
  ***********/
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial.println(F("* Advertising started"));
      delay(100);                                    // ToDo: SPI bug is here.
      display.drawChar(91, 8, 'b', WHITE, BLACK, 1);
    }
    if (status == ACI_EVT_CONNECTED) {
      Serial.println(F("* Connected!"));
      display.drawChar(91, 8, 'B', BLUE, BLACK, 1);
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
      display.drawChar(91, 8, 'b', WHITE, BLACK, 1);
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }
}

#ifdef __AVR__ // ToDo: Reminder to fix this properly.
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__

