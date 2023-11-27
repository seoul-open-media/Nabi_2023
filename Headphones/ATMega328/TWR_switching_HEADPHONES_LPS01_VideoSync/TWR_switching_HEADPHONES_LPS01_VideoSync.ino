/*   sending distance in 10cm so 25m is the max distance

      in this case we're using 3 anchors for LPS
      so total_num_anchors -> total_num_anchors
      i don't change it but make shure you set total_num_anchors to 3 in Teensy

     get the distance from anchors address 104~106
     i_destination + 100
     gets my_address info from teensy(not changed)

     Compiled in Arduino 1.8.9 with modification
    Arduino pro promini 3.3V 8Mhz

   


    Data Structure
   https://docs.google.com/spreadsheets/d/1Cie-AcoIuojLqRC60rg6zebejEXcjnMsB474eh22s5Q/edit?usp=sharing
*/

#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

// serial message flag
#define SET_DEFAULT_VALUE 0
#define SEND_SONG_INFO 1
#define FINAL_RESULT 2
#define EXCHANGE_FINISH 3

//wireless message flag
#define EXCHANGE_FINISHED 104

byte my_address, network_id, total_num_anchors, poll_ack_check_threshold;
//#define my_address 1
//#define poll_ack_check_threshold 4//////////////////////////11ms! total around 270ms.
#define  resetPeriod 8 ///////////////////////////////// 8ms. is the minimum
#define r_resetPeriod 20
//#define total_num_anchors 20


byte i_destination_address = 1;
byte r_destination_address = 1;

#define master_address 255

//#define poll_num 2



#define LEN_DATA 24
byte d_data[LEN_DATA];
byte i_data[LEN_DATA];
byte r_data[LEN_DATA];
byte distance_result[3]; // buffer to store ranging data
byte failure_counter[3];
byte frameMSB[3] = {0, 0, 0};
byte frameMiddleB[3]= {0,0,0};
byte frameLSB[3] = {0, 0, 0};
//#define switchingPeriod 1000


//#define default_mode 0
#define initiator_mode 1
#define responder_mode 2


int32_t lasttimesent = 0;


byte my_mode, poll_count;
boolean ok_send_poll, received_poll_ack = false;

// connection pins
const uint8_t PIN_RST = 3; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 10; // spi select pin

// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile boolean receivedMessage = false;
boolean protocolFailed = false;

// timestamps to remember
uint64_t timePollSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;

//responder only
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timeRangeReceived;

// watchdog and reset period
uint32_t lastActivity , time_poll_sent, time_stamp;


// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

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
  pinMode(13, OUTPUT);
  // put your setup code here, to run once:
  // DEBUG monitoring
  Serial.begin(57600);
  //Serial.begin(115200);
  delay(2000);
  while (!(Serial.available() > 5));
  byte first_byte = Serial.read();
  byte second_byte = Serial.read();
  // my_address, network_id, total_num_anchors, poll_ack_check_threshold;
  if (first_byte == 255 && second_byte == SET_DEFAULT_VALUE) {
    my_address = Serial.read();
    network_id = Serial.read();
    total_num_anchors = Serial.read();
    poll_ack_check_threshold = Serial.read();
  } else {
    // Serial.print("Problem!");
    while (1) {
      pinMode(13, OUTPUT);
      digitalWrite(13, 1);
      delay(100);
      digitalWrite(13, 0);
      delay(100);
    }

  }

  // Serial.println(F("### DW1000Ng-arduino-ranging-Initiator ###"));
  // initialize the driver
  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  // Serial.println("DW1000Ng initialized ...");
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

  DW1000Ng::setDeviceAddress(my_address);
  DW1000Ng::setNetworkId(network_id);
  DW1000Ng::setAntennaDelay(16436);

  DW1000Ng::setTXPower(522133279);  // 0x1F1F1F1F
  // Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  /*
    delay(1000);
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
  */

  // attach callback for (successfully) sent and received messages
  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);
  // anchor starts by transmitting a POLL message
  receiver();
  noteActivity();
  initializeDataBuffer();
  //my_mode = default_mode;
  my_mode = responder_mode;
}
void initializeDataBuffer() {
  for (int i = 0; i < LEN_DATA; i++) {
    d_data[i] = 0;
    i_data[i] = 0;
    r_data[i] = 0;
  }

  for (int i = 0; i < 3; i++) {
    failure_counter[i] = 0;
  }
}
void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}


void r_resetInactive() {
  // anchor listens for POLL
  expectedMsgId = POLL;
  receiver();
  noteActivity();
  //Serial.println("resetInactive");
}


void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on received success
  receivedAck = true;
  //receivedMessage = true;
}
void receiver() {
  DW1000Ng::forceTRxOff();
  // so we don't need to restart the receiver manually
  DW1000Ng::startReceive();
  //  Serial.println("startReceive");
}
void transmitPoll(byte address) {
  i_data[0] = POLL;
  i_data[16] = my_mode; // initiator_mode
  i_data[17] = address;
  i_data[18] = my_address;
  i_data[19] = 0; // _message 0, in case
  DW1000Ng::setTransmitData(i_data, LEN_DATA);
  DW1000Ng::startTransmit();
  // Serial.print("TransmitPoll to"); Serial.println(address);
}


void transmitPollAck() {
  r_data[0] = POLL_ACK;
  r_data[16] = my_mode; // responder_mode
  r_data[17] = r_destination_address;
  r_data[18] = my_address;
  r_data[19] = 0; // _message 0, in case
  DW1000Ng::setTransmitData(r_data, LEN_DATA);
  DW1000Ng::startTransmit();
  // Serial.println("transmitPollAck");
}


void transmitRange() {
  i_data[0] = RANGE;
  i_data[16] = my_mode; // initiator_mode
  i_data[17] = i_destination_address + 100 ;
  i_data[18] = my_address;
  i_data[19] = 0; // _message 0, in case


  /* Calculation of future time */
  byte futureTimeBytes[LENGTH_TIMESTAMP];

  timeRangeSent = DW1000Ng::getSystemTimestamp();
  timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
  DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
  DW1000Ng::setDelayedTRX(futureTimeBytes);
  timeRangeSent += DW1000Ng::getTxAntennaDelay();

  DW1000NgUtils::writeValueToBytes(i_data + 1, timePollSent, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(i_data + 6, timePollAckReceived, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(i_data + 11, timeRangeSent, LENGTH_TIMESTAMP);
  DW1000Ng::setTransmitData(i_data, LEN_DATA);
  DW1000Ng::startTransmit(TransmitMode::DELAYED);
  //  Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
  // Serial.println("TransmitRange");

}

void printResult() {
  Serial.print("elapsed time: ");
  Serial.println((millis() - time_stamp));
  for (int i = 0; i < total_num_anchors; i++) {
    Serial.print(distance_result[i]);
    Serial.print(", ");
  }
  Serial.println();
}



void sendDistanceAndFrameData() {

  byte elapsed_time = (millis() - time_stamp);
  byte len_data = 15;
  // 255, FINAL_RESULT, d1, d2, d3, frameMSB[0], frameMiddleB[0], frameLSB[0], frameMSB[1], frameMiddleB[1],frameLSB[1],frameMSB[2], frameMiddleB[2],frameLSB[2],elapsed_time
  byte s_data[len_data] = {255, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0,   0};
  s_data[1] = FINAL_RESULT;
  memcpy(&s_data[2], distance_result, 3);
  s_data[5] = frameMSB[0];
  s_data[6] = frameMiddleB[0];
  s_data[7] = frameLSB[0];
  
  s_data[8] = frameMSB[1];
  s_data[9] = frameMiddleB[1];
  s_data[10] = frameLSB[1];
  
  s_data[11] = frameMSB[2];
  s_data[12] = frameMiddleB[2];
  s_data[13] = frameLSB[2];
  s_data[14] = elapsed_time;
  Serial.write(s_data, len_data);

}



void loop() {


  while (my_mode == initiator_mode)twrInitiator();
  while (my_mode == responder_mode)twrResponder();

}
void twrInitiator() {
  //


  if (!sentAck && !receivedAck) { // 패킷을 보내고 받는 동안이 아니라면



    //////////////////////////
    if (ok_send_poll == false) { // poll하고나서
      if (millis() - time_poll_sent > poll_ack_check_threshold ) { // 일정시간후에도
        if (received_poll_ack == false) { // poll_ack을 받지 못했다면
          // 그다음 주소로 이동 하여 폴
          distance_result[(i_destination_address - 1)] = 252;
          i_destination_address++;
          expectedMsgId = POLL_ACK;
          ok_send_poll = true;
          sentAck = false;
          receivedAck = false;
        }
      }
    }

    //////////////////////////

    if (ok_send_poll == true) {
      if (i_destination_address < (total_num_anchors + 1)) { // i_destination_address 1~3
        received_poll_ack = false;
        expectedMsgId = POLL_ACK;
        ok_send_poll = false;
        noteActivity();
        time_poll_sent = millis();
        DW1000Ng::forceTRxOff();
        transmitPoll(i_destination_address + 103); ////////////////////////////////////////////////// anchors from 104 to 106 for the distance measuring

        return;

      } else {///////////////////////////////////////// we checked all the range


        // be prepared and go out to responder_mode
        ok_send_poll = false;
        expectedMsgId = POLL;
        my_mode = responder_mode;
        DW1000Ng::forceTRxOff();
        DW1000Ng::startReceive();


        
        sendDistanceAndFrameData();//////////////////////////////////////////////////////////////// send the final data
        
        return;
        ///////////////////////////////////////////////

      }// if the destination headphones is off
    } else if (ok_send_poll == false && (millis() - lastActivity > resetPeriod)) {
      distance_result[(i_destination_address - 1)] = 253;
      i_destination_address++;
      ok_send_poll = true;
      /*
        Serial.print("i_destination_address");
        Serial.print((i_destination_address - 1)); // since we added 1 before
        Serial.println(" time out");
      */
    }
  }
  // continue on any success confirmation
  if (sentAck) {

    sentAck = false;
    DW1000Ng::startReceive();
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000Ng::getReceivedData(i_data, LEN_DATA);
    byte msgId = i_data[0];
    byte mode_from_sender = i_data[16];
    byte to_address = i_data[17];
    byte from_address = i_data[18];
    byte _message = i_data[19];
    //Serial.print("Initiator Received message from");
    //Serial.println(from_address);
    ////////////if the master's message matches my address, switch to initiator_mode
    if (from_address == master_address) {
      if (_message == my_address) {
        // already initiator mode
      } else if (_message != my_address) {
        // else change to responder_mode
        // Serial.write (255);
        // Serial.write (2);

        expectedMsgId = POLL;
        my_mode = responder_mode;
      }
      DW1000Ng::startReceive();
      return;
    }
    /////////////
    ////////
    if (to_address != my_address) {
      r_resetInactive();
      noteActivity();
      return;
    }
    /////////

    ////////////
    if ( mode_from_sender == responder_mode) {
      if (msgId != expectedMsgId) {
        // unexpected message, start over again
        // Serial.print("Received wrong message # "); Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        distance_result[(i_destination_address - 1)] = 254;
        i_destination_address++;
        ok_send_poll = true;

        return;
      }
      if (msgId == POLL_ACK) {
        received_poll_ack = true;
        timePollSent = DW1000Ng::getTransmitTimestamp();
        timePollAckReceived = DW1000Ng::getReceiveTimestamp();
        expectedMsgId = RANGE_REPORT;
        transmitRange();
        noteActivity();
      } else if (msgId == RANGE_REPORT) {

        expectedMsgId = POLL_ACK;
        float curRange;
        memcpy(&curRange, i_data + 1, 4);

        ////////////////////////////
        // int distanceCm = int((curRange / DISTANCE_OF_RADIO_INV) * 100); // distance in cm.
        int distanceCm = int((curRange / DISTANCE_OF_RADIO_INV) * 10); // distance in 10cm.
        // distanceCm /= 2;
        // raging report
        if ( distanceCm < 1) distanceCm = 1; // negative is 1cm
        if ( distanceCm > 250 ) distanceCm = 251; // negative is 1cm
        //////// get distance
        distance_result[(i_destination_address - 1)] = distanceCm;
        //////// get info


        ////////////////////////////
        i_destination_address++;
        ok_send_poll = true;
        noteActivity();

        /////////////////////////
      } else if (msgId == RANGE_FAILED) {
        expectedMsgId = POLL_ACK;

        distance_result[(i_destination_address - 1)] = 252;
        i_destination_address++;
        ok_send_poll = true;
        noteActivity();

      }
      /////////////////////////////////////
    }  
  }

}



void twrResponder() {
  int32_t curMillis = millis();
  /*
    ////////////////////////////////////////// to fake master
    if ( millis() > lasttimesent + 2400) {
    ok_send_poll = true;
    i_destination_address = 1; // 1~24

    expectedMsgId = POLL_ACK;
    my_mode = initiator_mode;
    // we don't need to get ready to message because we will send message in initiator mode
    // DW1000Ng::startReceive();

    noteActivity();
    time_stamp = millis();
    lasttimesent = millis();
    return;

    }
    ////////////////////////////////////////
  */
  if (!sentAck && !receivedAck) {

    ////////////////////
    // getSerialData();
    ////////////////////

    // check if inactive
    if (curMillis - lastActivity > r_resetPeriod) {
      r_resetInactive();
    }
    return;
  }


  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = r_data[0];
    if (msgId == POLL_ACK) {
      timePollAckSent = DW1000Ng::getTransmitTimestamp();
      noteActivity();
    }
    DW1000Ng::startReceive();
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000Ng::getReceivedData(r_data, LEN_DATA);
    byte msgId = r_data[0];
    byte mode_from_sender = r_data[16];
    byte to_address = r_data[17];
    byte from_address = r_data[18];
    byte _message = r_data[19];
    //Serial.print("responder Received message from");
    //Serial.println(from_address);
    //Serial.print("msgId"); Serial.print(msgId); Serial.print("mode"); Serial.print(mode_from_sender); Serial.print("to_address"); Serial.print(to_address); Serial.print("from_address"); Serial.print(from_address); Serial.print("_message"); Serial.println(_message);
    ////////////if the master's message matches my address, switch to initiator_mode
    if (from_address == master_address) {
      if (_message == my_address) {
        // be prepared and change to initiator mode
        ok_send_poll = true;
        i_destination_address = 1; // 1~24

        expectedMsgId = POLL_ACK;
        my_mode = initiator_mode;
        // we don't need to get ready to message because we will send message in initiator mode
        // DW1000Ng::startReceive();

        noteActivity();
        time_stamp = millis();
        return;
      } else if (_message != my_address) {
        //  else change to responder_mode
        // Serial.println("Not for me  and I remain in responder mode");
        r_resetInactive();
        noteActivity();
        return;
      }

    } else if (from_address == 5 || from_address == 6 || from_address == 7) { //  if its the data from the VideoSync Adapter
      switch (from_address) {
        case 5:
          frameMSB[0] = r_data[20];
          frameMiddleB[0] = r_data[21];
          frameLSB[0] = r_data[22];
          break;
        case 6:
          frameMSB[1] = r_data[20];
          frameMiddleB[1] = r_data[21];
          frameLSB[1] = r_data[22];
          break;
        case 7:
          frameMSB[2] = r_data[20];
          frameMiddleB[2] = r_data[21];
          frameLSB[2] = r_data[22];
          break;

        default:
          break;
      }


    }

    ////////
    if (to_address != my_address) {
      r_resetInactive();
      noteActivity();
      return;
    }
    /////////

    ///////////////
  }
}
void blinkLED() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}
