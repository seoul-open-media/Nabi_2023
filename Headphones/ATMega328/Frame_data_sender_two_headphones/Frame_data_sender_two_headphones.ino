#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

#define VERSION 0.1
#define I2C_ADDRESS 0x3C
#define RST_PIN -1

#define SERIAL_READ 0
#define DWM1000_SEND 1

#define _RADDERBOT 0
#define _LIGHT 1

SSD1306AsciiAvrI2c oled;

#define MY_ADDRESS  5 // 1,2 : IMU headphones, 3,4: Video Sync headphones,  5~7: Frame sender
#define MASTER_ADDRESS 255
//#define LADDER_ADDRESS 1

#define pan_id 10
#define LEN_DATA 30

int state;
int to_address;
byte data[LEN_DATA];
byte frameMSB, frameMiddleB,frameLSB = 0;

unsigned long lasttimeDisplayed = 0;

byte _mode, _group, _address, _preset, mode, destination_x, destination_y, function, sensor_min, sensor_max  = 0;

// connection pins
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_RST = 3; // reset pin
const uint8_t PIN_EXTON = 4; // ext_on pin
const uint8_t PIN_WAKEUP = 5; // WAKEUP pin
const uint8_t PIN_SS = 8; // spi select pin
const uint8_t PIN_LED = 13; // spi select pin
const uint8_t PIN_L1 = 6; // PWM1
const uint8_t PIN_L2 = 9; // PWM2
const uint8_t PIN_L3 = 10; // PWM3

volatile boolean sentAck = false;
volatile boolean receivedAck = false;

device_configuration_t DEFAULT_CONFIG = {
  false,
  true,
  true,
  true,
  false,
  SFDMode::STANDARD_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_850KBPS,
  PulseFrequency::FREQ_16MHZ,
  PreambleLength::LEN_256,
  PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
  true,
  true,
  true,
  false,
  true
};

void setup() {
  pinMode(PIN_EXTON, OUTPUT);
  pinMode(PIN_WAKEUP, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_EXTON, HIGH); // EXTON
  digitalWrite(PIN_WAKEUP, LOW); // wakeup state

  // turn off led
  analogWrite(PIN_L1, 0);
  analogWrite(PIN_L2, 0);
  analogWrite(PIN_L3, 0);

  Serial.begin(57600);
  //delay(2000);
  //while (!Serial);

  //Serial.println(F("### DW1000Ng-arduino-ranging-Initiator ###"));
  // initialize the driver
  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  // Serial.println("DW1000Ng initialized ...");
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

  DW1000Ng::setDeviceAddress(MY_ADDRESS);
  DW1000Ng::setNetworkId(pan_id);
  DW1000Ng::setAntennaDelay(16436);
  //delay(1000);
  DW1000Ng::setTXPower(522133279);  // 0x1F1F1F1F
  //    DW1000Ng::setTXPower(3233857728);  // 0xC0C0C0C0

  // Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  //Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  //Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  //Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  //Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) sent and received messages
  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);
  // anchor starts by transmitting a POLL message



#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.print("IMU data adapter v. ");
  oled.println(VERSION);
  oled.println();
  // oled.print("LPS Adapter");
  oled.println();
  oled.println();
  oled.println("ATmega328P");
  oled.println();
  oled.print("My_address ");
  oled.println(MY_ADDRESS);

  DW1000Ng::forceTRxOff();
  DW1000Ng::startReceive();


}

void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on received success
  receivedAck = true;
}

void loop() {

  if (Serial.available() > 3) {
    
    if (Serial.read() == 255) {
      frameMSB = Serial.read();
      frameMiddleB = Serial.read();
      frameLSB = Serial.read();

    } else {
      Serial.flush();
      while(Serial.available())Serial.read();
    }
  }

  if (receivedAck == true) {
    //     Serial.println("receivedAck");
    receivedAck = false;
    ///////////// get the distance data of LPS1,2 and update it
    byte lenData = 24; // 24bytes sent from Master, 19 was written originally
    byte r_data[lenData];
    DW1000Ng::getReceivedData(r_data, lenData);
    byte to_message = r_data[17];
    byte from_address = r_data[18];


    //   Serial.print("receivedAck from ");  Serial.print(from_address);Serial.print("   to address");  Serial.println(to_address);

    if (to_message == MY_ADDRESS && from_address == MASTER_ADDRESS) {
      // send frame data to headphones address 3, 4
      byte lenData = 24; // 24bytes sent from Master, 19 was written originally
      byte s_data[lenData];
      for (int i = 3; i < 5; i++) {
        DW1000Ng::forceTRxOff();
        data[18] = MY_ADDRESS;
        data[19] = i ;
        data[20] = frameMSB ;
        data[21] = frameMiddleB ;
        data[22] = frameLSB ;

        DW1000Ng::setTransmitData(data, LEN_DATA);
        DW1000Ng::startTransmit();
      }
      if (millis() > lasttimeDisplayed + 500) {
        lasttimeDisplayed = millis();
        oled.clear();
        oled.print("MSB:");
        oled.println(frameMSB);
        oled.print("MiddleB:");
        oled.println(frameMiddleB);
        oled.print("LSB:");
        oled.println(frameLSB);
        oled.println("");
      }

      DW1000Ng::startReceive();


    } else {
      DW1000Ng::forceTRxOff();
      DW1000Ng::startReceive();
    }
  }


  /*

    switch (state) {
      case SERIAL_READ:
      // bytes coming in
      // 255, 2nd_byte(0, 1), 3rd_byte(destination_x) .................. 8th_byte(sensor_max)
        if (Serial.available() > 7) {
          //  Serial.println("Got Final Result");
          // digitalWrite(13, HIGH);
          byte first_byte = Serial.read();
          byte second_byte = Serial.read();
          to_address = second_byte;
          //Serial.print("first_byte ="); Serial.println(first_byte);

          if (first_byte == 255) {
            if (second_byte == 0) { //
              destination_x = Serial.read();
              destination_y = Serial.read();
              mode = Serial.read();
              function = Serial.read();
              sensor_min = Serial.read();
              sensor_max = Serial.read();


            } else {
              _mode = Serial.read();
              _group = Serial.read();
              _address = Serial.read();
              _preset = Serial.read();
              // read all 8bytes
              sensor_min = Serial.read();
              sensor_max = Serial.read();

            }


            state = DWM1000_SEND;

          } else {
            while (Serial.available()) {
              Serial.read();
            }
          }
          //        oled.clear();
          //        oled.print("_mode ");
          //        oled.println(_mode);
          //        oled.print("_group ");
          //        oled.println(_group);
          //        oled.print("_address ");
          //        oled.println(_address);
          //        oled.print("_preset ");
          //        oled.println(_preset);
        }

        break;

      case DWM1000_SEND:

        if (to_address == _RADDERBOT) {
                  oled.clear();
                  oled.print("got data  and send ");

          //to Ladderbot master
          data[18] = MY_ADDRESS;
          data[19] = LADDER_ADDRESS;
          data[20] = destination_x;
          data[21] = destination_y;
          data[22] = mode;
          data[23] = function;
          data[24] = sensor_min;
          data[25] = sensor_max;

        } else if (to_address == _LIGHT) {
          //to lighting
          data[18] = MY_ADDRESS;
          data[19] = _address;
          data[20] = _mode;
          data[21] = _group;
          data[22] = _preset;
        }
        DW1000Ng::forceTRxOff();
        DW1000Ng::setTransmitData(data, LEN_DATA);
        DW1000Ng::startTransmit();
        while (!sentAck) {

        };

        sentAck = false;
        //        delay(5);
        DW1000Ng::startReceive();
        state = SERIAL_READ;
        break;

      default:
        state = SERIAL_READ;
        break;

    }

  */

}
