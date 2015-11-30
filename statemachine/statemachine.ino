#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>

Adafruit_SSD1306 display(100);

#define SIM_STANDARD_DELAY 500

//int sendCheckReply (const __FlashStringHelper *send, const __FlashStringHelper *reply, uint16_t timeout = SIM_STANDARD_DELAY);
int sendCheckReply (char *send, char *reply, uint16_t timeout = SIM_STANDARD_DELAY);

#define SIM_RESET 4

#define STATE_SIM_0 0
#define STATE_SIM_BEGIN_1 1
#define STATE_SIM_BEGIN_2 2
#define STATE_SIM_BEGIN_3 3
#define STATE_SIM_BEGIN_4 4

int currentSIMState = STATE_SIM_0;
unsigned long simCurrentStateTimeout = 0;
unsigned long currentMillis = 0;
unsigned long simStateStart = 0;
bool stateTimeout = false;
bool simNeedsStartup = false;
bool simEvent = false;

// SendCheckReply
int sendCheckReplyState = 0;
unsigned long sendCheckReplyStateTimeout = 0;
unsigned long sendCheckReplyStateStart = 0;
uint16_t replyIndex = 0;
uint16_t replyLength = 0;
char replyBuffer[255];

#define SEND_CHECK_REPLY_STATE_SEND 0
#define SEND_CHECK_REPLY_STATE_RECEIVING 1
#define SEND_CHECK_REPLY_STATE_TIMEOUT 2
#define SEND_CHECK_REPLY_STATE_OK 3
#define SEND_CHECK_REPLY_STATE_INVALID 4
#define SEND_CHECK_REPLY_RETURN_BUSY 0
#define SEND_CHECK_REPLY_RETURN_OK 1
#define SEND_CHECK_REPLY_RETURN_INVALID -1
#define SEND_CHECK_REPLY_RETURN_TIMEOUT -2


void setup() {
  pinMode(SIM_RESET, OUTPUT);
  simNeedsStartup = true;
  Serial1.begin(4800);
  Serial.begin(115200);
  
  // Initialize display first
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  //  display.setRotation(1); // rotate display 90 degrees
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextColor(WHITE, BLACK); // 'inverted' text
  display.print("Jetpack 1\n\n");
  display.display();

  while (!Serial && millis() < 4000);
  display.print("Serial... OK\n");
  display.display();
//  pinMode(2, OUTPUT);
//  digitalWrite(2, HIGH);
//  delay(1000);
//  digitalWrite(2, LOW);

}

void loop () {
  currentMillis = millis();
  checkSIMState();
  updateDisplay();
  Serial.println(currentSIMState);
}

unsigned long displayStateStart = 0;
unsigned long displayCurrentStateTimeout = 0;

void updateDisplay() {
  // Check if state is timed out.
  if ((unsigned long)(currentMillis - displayStateStart) >= displayCurrentStateTimeout) {
    stateTimeout = true;
    displayStateStart = currentMillis;
  } else {
    stateTimeout = false;
  }

  if (stateTimeout) {
    display.setCursor(20,40);
    display.print("Sim state: ");
    display.print(currentSIMState);
    display.setCursor(20,50);
    display.print("Com state: ");
    display.print(sendCheckReplyState);
    display.display();
  }
}

void checkSIMState () {
  // Check if state is timed out.
  if ((unsigned long)(currentMillis - simStateStart) >= simCurrentStateTimeout) {
    stateTimeout = true;
  } else {
    stateTimeout = false;
  }
  
  switch (currentSIMState) {
    case STATE_SIM_0:
      if (simNeedsStartup) {
        currentSIMState = STATE_SIM_BEGIN_1;
        simStateStart = currentMillis;
        simNeedsStartup = false;
      }
    break;
    case STATE_SIM_BEGIN_1:
      if(!stateTimeout) {
        return;
      }
      digitalWrite(SIM_RESET, HIGH);
      simStateStart = currentMillis;
      simCurrentStateTimeout = 10;
      currentSIMState = STATE_SIM_BEGIN_2;
      simStateStart = currentMillis;      
    break;
    case STATE_SIM_BEGIN_2:
      if(!stateTimeout) {
        return;
      }
      digitalWrite(SIM_RESET, LOW);
      simStateStart = currentMillis;
      simCurrentStateTimeout = 100;
      currentSIMState = STATE_SIM_BEGIN_3;
      simStateStart = currentMillis;       
    break;
    case STATE_SIM_BEGIN_3:
      if(!stateTimeout) {
        return;
      }
      digitalWrite(SIM_RESET, HIGH);
      simStateStart = currentMillis;
      simCurrentStateTimeout = 7000;
      currentSIMState = STATE_SIM_BEGIN_4;
      simStateStart = currentMillis;       
    break;
    case STATE_SIM_BEGIN_4:
      if(!stateTimeout) {
        return;
      }
      int ok = sendCheckReply("AT", "ATOK");
//      if (ok != 0 && ok != 1) {
//        currentSIMState = STATE_SIM_BEGIN_1;
//      }
      Serial.println(ok);
    break;
  }  
}

int sendCheckReply (char *send, char *reply, uint16_t timeout) {
  // Check if state is timed out
  if ((unsigned long)(currentMillis - sendCheckReplyStateStart) >= sendCheckReplyStateTimeout) {
    stateTimeout = true;
  } else {
    stateTimeout = false;
  }

  switch(sendCheckReplyState) {
    case SEND_CHECK_REPLY_STATE_SEND:
      sendCheckReplyStateTimeout = timeout;
      while (Serial1.available()) Serial1.read();
      Serial1.println(send);
      sendCheckReplyState = SEND_CHECK_REPLY_STATE_RECEIVING;
      sendCheckReplyStateStart = currentMillis;
      stateTimeout = false;
      replyIndex = 0;
      replyLength = strlen(reply);
    break;
    case SEND_CHECK_REPLY_STATE_RECEIVING:
      // Check if time out happened before reading buffer
      if (stateTimeout) {
        sendCheckReplyState = SEND_CHECK_REPLY_STATE_TIMEOUT;
      }
      
      while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\r') continue;
        if (c == 0xA) {
          if(replyIndex == 0) {
            continue;
          } else {
            break;
          }
        }
        if (reply[replyIndex] == c) {
          if (replyLength == replyIndex +1) {
            sendCheckReplyState = SEND_CHECK_REPLY_STATE_OK;
            break;
          }
        } else {
          Serial.println("wrong!");
          // Wrong response
          sendCheckReplyState = SEND_CHECK_REPLY_STATE_INVALID;
          break;
        }
        replyIndex++;
      }
    break;
    case SEND_CHECK_REPLY_STATE_OK:
      return SEND_CHECK_REPLY_RETURN_OK;
    break;
    case SEND_CHECK_REPLY_STATE_INVALID:
      return SEND_CHECK_REPLY_RETURN_INVALID;
    break;
    case SEND_CHECK_REPLY_STATE_TIMEOUT:
    // do timeout stuff
       return SEND_CHECK_REPLY_RETURN_TIMEOUT;
    break;
  }
  return SEND_CHECK_REPLY_RETURN_BUSY;
}

