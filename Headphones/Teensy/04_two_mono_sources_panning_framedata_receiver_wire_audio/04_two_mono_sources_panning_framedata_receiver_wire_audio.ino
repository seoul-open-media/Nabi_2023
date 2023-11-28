/*
   - 4 mono sources at -90(stream), 0(bird), 0(bell), 90(rain) degrees rotate upon heading with gaussian radius distribution
   - EBIMU applied
   - Neokey setup
   - Heading calibrated (towards 0 degree)
   - 1 player with 4ch audio (44.1Khz,

   TODO
   - LPS
   - check if the user is in Zone 1 or 2
   - gain change upon zone 1, 2
*/


#include "fourSourcesToStereo.h"
#include "Wire.h"
#include <SC16IS750.h>
#include <string.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <play_wav.h>
#include <Audio.h>
#include <SPI.h>
//#include <SD.h>
#include <SerialFlash.h>
#include <Bounce2.h>
#include <Adafruit_NeoKey_1x4.h>
#include <seesaw_neopixel.h>
#include <EEPROM.h>
#include <Filter.h>
// GUItool: begin automatically generated code
AudioPlayWav             playWav1;
AudioPlayWav             playWav2;
AudioPlayWav             playWav3;

fourSourcesToStereo fourMonoPanner1;

AudioOutputI2S           output;

AudioMixer4              mixerL;         //L
AudioMixer4              mixerR;         //R
//AudioMixer4              mixer3;

AudioConnection          patchCord1(playWav1, 0, mixerL, 0);
AudioConnection          patchCord2(playWav1, 1, mixerR, 0);

AudioConnection          patchCord3(playWav2, 0, mixerL, 1);
AudioConnection          patchCord4(playWav2, 1, mixerR, 1);

AudioConnection          patchCord5(playWav3, 0, mixerL, 2);
AudioConnection          patchCord6(playWav3, 1, mixerR, 2);




AudioConnection          patchCord9(mixerL, 0, output, 0);
AudioConnection          patchCord10(mixerR, 0, output, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=853,409
// GUItool: end automatically generated code

#define numFiles 3
AudioPlayWav *player[numFiles] = {
  &playWav1,  &playWav2,  &playWav3
};

const char *filename[numFiles] = {
  "BAC.WAV",
  "RBR.WAV",
  "00INTRO.WAV"

};

#define DEBUG 1

// Headphone info
#define MY_ADDRESS 1
byte my_address = MY_ADDRESS;
#define VERSION 0.32
#define SOFT_VERSION 0.1

///////////////// LPS
#define NETWORK_ID 10
#define TOTAL_NUM_ANCHORS 3 // 12
#define POLL_ACK_CHECK_THRESHOLD 8///////////////////////// 8 total elapsed time around 170ms. with TOTAL_NUM_ANCHORS 20

#define d 7 // distance bewteen Anchor1 and Anchor2
#define p3_i 3.1 // x cordinate of Anchor3 
#define p3_j 4.1 // y cordinate of Anchor3

#define TOTAL_NUM_ZONE 2
#define GROUP_NUM_POINT 4
#define HWSERIAL Serial1

// serial message flag
#define SET_DEFAULT_VALUE 0
#define SEND_SONG_INFO 1
#define FINAL_RESULT 2
#define EXCHANGE_FINISH 3
#define FRAME_DATA 4

#define S1_POS_DEG -90
#define S2_POS_DEG 0
#define S3_POS_DEG 0
#define S4_POS_DEG 90

// Use these with the Teensy Audio Shield
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  11
#define SDCARD_MISO_PIN  12
#define SDCARD_SCK_PIN   13

// eeprom address
#define EP_ADDR_MY_ADDRESS 0
#define EP_ADDR_HEAD_OFFSET_MSB 1
#define EP_ADDR_HEAD_OFFSET_LSB 2



//Screen Settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

/* Set the delay between fresh samples */
#define IMU_POLL_INTERVAL 50 ////////////////////////////////////////////////// UPDATE INTERVAL OF EBIMU in ms.

// Pins and Settings
#define pi 3.1415926536
#define radTo14 2607.594587617613379
#define oneTo14 8191
#define qCalAddr 0 // EEPROM qCal address

// I2C to Uart Command Buffer
#define SBUF_SIZE 64

// Neokey
#define DOWN 0
#define UP 1
#define WRITE 2
#define MODE 3

bool cfg_mode = false;

#define NORMAL_MODE 0
#define RECORD_MODE 1

// Neokey Menu
#define SCREEN_MAIN_CFG_MENU 0
#define SCREEN_SET_HEADPHONES_ADDRESS 1
#define SCREEN_IMU_CALIBRATION 2
#define SCREEN_SET_IMU_OFFSET 3

int screen = SCREEN_MAIN_CFG_MENU;
int row, num_menu = 0;

// STATE
#define STATE_INIT 0
#define STATE_IMU_POLL 1
#define STATE_IMU_POLL_WAIT 2
#define STATE_IMU_READ 3
#define STATE_CHANGE_PANNER 4
#define STATE_IMU_WAIT 5

// STATE ZONE
#define STATE_ZONE_CHECK 0
#define STATE_ZONE1 1
#define STATE_ZONE2 2


// smoothe instances for BNO055
//Smoothed <float> smoothed1;
//Smoothed <float> smoothed2;
//Smoothed <float> smoothed3;
//-------------FrameInfo----------------------
byte frameMSB[3] = {0, 0, 0};
byte frameMiddleB[3] = {0, 0, 0};
byte frameLSB[3] = {0, 0, 0};
int frame[3] = {0, 0, 0};

//-------------IMU----------------------------
String imuStart = "<start>";
String imuPoll = "*";

char sbuf[SBUF_SIZE];
signed int sbuf_cnt = 0;

int state;
boolean isCheckTimer;
//signed int imuPollingWait, gpsPollingWait, navWait;
unsigned long imuMillis, gpsMillis, navMillis, imuPollTime;

int imu_poll_wait_time = 70;

float euler[3];
float heading_offset = 0;

float imuHead, imuPitch, imuRoll;
float radHead, radPitch, radRoll;
float Head, Pitch, Roll;

float radius1, radius2, radius3, radius4 ;
float scaled_radius1, scaled_radius2, scaled_radius3, scaled_radius4;
float degPos1, degPos2, degPos3, degPos4;

// -----------LPS------------------------------
float x, y, z = 0; // codinate of this Tag
float max_distance[3] = {25.1, 25.1, 25.1}; //maximum distance from anchor 1, 2, 3
float min_distance[3] = {0.1, 0.1, 0.1};

byte distance_result[20];
byte elapsed_time;
byte checksum_received;  // to store ATmega serial checksum_received messge

boolean isGetPosition, song_change_flag = false;

int state_zone;
uint8_t numZone = 0; // to store zone number
uint8_t polySides[TOTAL_NUM_ZONE] = {4, 4};

struct PositionStruct {
  float _x; // it should be float
  float _y;
};

PositionStruct zonePolygon[TOTAL_NUM_ZONE][GROUP_NUM_POINT];

float zonePointX[TOTAL_NUM_ZONE][GROUP_NUM_POINT] = {
  {0, 0, 3, 3},
  {3.8, 3.8, 7, 7}
};

float zonePointY[TOTAL_NUM_ZONE][GROUP_NUM_POINT] = {
  { -0.5, 3.8, 4.1, -0.5},
  { -0.5, 4.1, 4.7, -0.5}
};

// -----------Mixer----------------------------
float mixer_gain[4];
float destination_gain[4];
unsigned long lasttime_fade_update = 0;
uint8_t fade_update_interval = 80;

// --------------------------------------------
volatile bool serialState = 0;
int action = 0;

int serToAct = 0;
long lastMillis, lastMillisImu, lastMillisImuUpdate, lastMillisSerial, calibMillis  = 0;




#if defined(__IMXRT1062__)
#define T4
#include <utility/imxrt_hw.h> // make available set_audioClock() for setting I2S freq on Teensy 4
#else
#define F_I2S ((((I2S0_MCR >> 24) & 0x03) == 3) ? F_PLL : F_CPU) // calculation for I2S freq on Teensy 3
#endif

// SC16IS750 Instance for EBIMU
SC16IS750 i2cuart = SC16IS750(SC16IS750_PROTOCOL_I2C, SC16IS750_ADDRESS_AA);

// SSD1306 Instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// Neokey Instance
Adafruit_NeoKey_1x4 neokey;

//-----------neoKey-----------------------------
boolean update_screen = true;
boolean keyPressed[4] = {false, false, false, false};
boolean lastKeyState[4] = {false, false, false, false};
bool key_A_predicate()
{
  return neokey.read() & 0x01;
}

bool key_B_predicate()
{
  return neokey.read() & 0x02;
}

bool key_C_predicate()
{
  return neokey.read() & 0x04;
}

bool key_D_predicate()
{
  return neokey.read() & 0x08;
}

// instantiate the PrediateDebouncer objects
PredicateDebouncer debouncers[4] = { PredicateDebouncer(&key_A_predicate, 5),
                                     PredicateDebouncer(&key_B_predicate, 5),
                                     PredicateDebouncer(&key_C_predicate, 5),
                                     PredicateDebouncer(&key_D_predicate, 5)
                                   };

// set a variable to store the led states
int led_states[4] = {LOW, LOW, LOW, LOW};
int neoKey_num, entry_counter = 0;
bool neoKey_enter, neoKey_exit = false;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  //////////////////////////////////////////////////////////////EEPROM
  // initialize EEPROM value for the first boot
  if (my_address > 50) EEPROM.write(EP_ADDR_MY_ADDRESS , 0); // initialize it to 0 on the first boot
  // on the first boot, eeprom is initialized as bigger than 250, so set it to 0 if it's first.
  // use devision factor of 250
  if (EEPROM.read(EP_ADDR_HEAD_OFFSET_MSB) > 1)EEPROM.write(EP_ADDR_HEAD_OFFSET_MSB, 0); // 0 1
  if (EEPROM.read(EP_ADDR_HEAD_OFFSET_LSB) > 250)EEPROM.write(EP_ADDR_HEAD_OFFSET_LSB, 0); //


  // read value from EPPROM
  my_address = EEPROM.read(EP_ADDR_MY_ADDRESS);
  heading_offset = EEPROM.read(EP_ADDR_HEAD_OFFSET_MSB) * 250 + EEPROM.read(EP_ADDR_HEAD_OFFSET_LSB);

  //////////////////////////////////////////////////////////////SERIAL
  Serial.begin(9600);
  if (DEBUG) {
    while (!Serial)
      ;
  }

  HWSERIAL.begin(58824);
  // HWSERIAL.begin(115200);
  delay(1000);
  while (!HWSERIAL);
  Serial.println("HWSerial begin");

  byte lenData = 6;
  byte s_data[lenData] = { 255, 0, 0, 0, 0, 0};
  s_data[1] = SET_DEFAULT_VALUE;
  s_data[2] = my_address;
  s_data[3] = NETWORK_ID;
  s_data[4] = TOTAL_NUM_ANCHORS;
  s_data[5] = POLL_ACK_CHECK_THRESHOLD;
  HWSERIAL.write(s_data, lenData);


  //////////////////////////////////////////////////////////////I2C
  Wire.begin();
  Wire.setClock(100000); //Increase I2C clock speed to 400kHz
  Wire2.begin();
  Wire2.setClock(100000); //Increase I2C clock speed to 100kHz
  /////////////////////////////////////////////////////////////Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  displayDataInfo();
  delay(2000);

  /////////////////////////////////////////////////////////////EBIMU
  // open connection in 921600
  i2cuart.begin(921600); // open serial connection with EBIMU
  // you need to ping it to escape from previous malfunction
  // irregular behaviours of SC16IS750 was resolved after this code
  if (i2cuart.ping() != 1) { ////////////////////// important!
    Serial.println("SC16IS750 not found");
    displayData02();
    while (1);
  } else {
    Serial.println("SC16IS750 found" );
  }

  delay(1000);
  EBimuCommand("<sor0>"); // set EBIMU to polling mode
  delay(500);
  // while (i2cuart.available())Serial.print(i2cuart.read());
  // while (1);
  if (i2cuart.read() == 60) { // see if it's answer correctly, ascii '<'
    // it's ok to send other commands
    EBimuCommand("<start>");
    Serial.println("start ebimu");
  } else {
    Serial.println("changing baudrate");
    displayData01();
    changeImuBaudrate();
  }


  //////////////////////////////////////////////////////////////NEOKEY
  // if we find neokey, go into configuration mode
  if (neokey.begin(0x30)) {     // begin with I2C address, default is 0x30
    // enter configuration mode
    Serial.println("NeoKey started!");
    state = STATE_INIT; // initialize IMU state
    cfg_mode = true;
    screen = SCREEN_MAIN_CFG_MENU;
    neoKey_num = 3 ; // 3 is the first row
    neoKeyConfigSettings();

  }

  /////////////////////////////////////////////////////////////SD
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  while (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      displayData00();
      delay(500);
    }
  }
  Serial.println("SD card checked");

  /////////////////////////////////////////////////////////////AUDIO
  fourMonoPanner1.setParamValue("Angle1", 0); // -6.283 ~ 6.283
  fourMonoPanner1.setParamValue("Radius1", 1); // 0 ~ 5
  fourMonoPanner1.setParamValue("Angle2", 1.578); // -6.283 ~ 6.283
  fourMonoPanner1.setParamValue("Radius2", 1); // 0 ~ 5
  fourMonoPanner1.setParamValue("Angle3", -1.578); // -6.283 ~ 6.283
  fourMonoPanner1.setParamValue("Radius3", 1); // 0 ~ 5
  fourMonoPanner1.setParamValue("Angle4", 1.578 * 2); // -6.283 ~ 6.283
  fourMonoPanner1.setParamValue("Radius4", 1); // 0 ~ 5

  sgtl5000_1.enable();
  sgtl5000_1.volume(0.55);
  AudioMemory(50);

  setCoordinates();

  //  while (!playWav1.isPlaying()) {
  //    playWav1.play("4ch_mono.wav");
  //    delay(10);
  //  }

  playWav1.loop(true);


/*
  player[0]->play(filename[0]);
  delay(100);
  player[0]->pause(true);
  player[0]->loop(true);
  player[0]->setPosition(4410000);
  player[0]->pause(false);
*/

  for (int i = 0; i < numFiles; i++) {
    player[i]->play(filename[i]);
    delay(100);
    player[i]->pause(true);
    player[i]->loop(true);
  }
  /////////////////////////////////////////////////////////// INITIALIZE

  state = STATE_INIT; // initialize state


  Serial.println("End of Setup");

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
float deg2rad = (3.141592654f / 180.0f);
float stereoWidth = 3.1415926f / 4.0f;  // default 4.0


void loop() {

  getImuData();

  getHWSerial();

  if (isGetPosition) {
    isGetPosition = false;
    // checkZone();

  }
  if (song_change_flag == true)playThisPlayerPauseOthers(numZone - 1);
  updateMixerGain();

  updateMixerGain();

  unsigned long currentMillis = millis();

  /*
    if (currentMillis - lastMillisImu > 50) {

      lastMillisImu = currentMillis;

      // sound sources positioned between -180 and 180 degree
      degPos1 = S1_POS_DEG - imuHead;
      degPos2 = S2_POS_DEG - imuHead;
      degPos3 = S3_POS_DEG - imuHead;
      degPos4 = S4_POS_DEG - imuHead;

      // -180 ~ 180
      if (degPos1 > 180) degPos1 -= 360;
      else if (degPos1 < -180) degPos1 += 360;

      if (degPos2 > 180) degPos2 -= 360;
      else if (degPos2 < -180) degPos2 += 360;

      if (degPos3 > 180) degPos3 -= 360;
      else if (degPos3 < -180) degPos3 += 360;

      if (degPos4 > 180) degPos4 -= 360;
      else if (degPos4 < -180) degPos4 += 360;

      fourMonoPanner1.setParamValue("Angle1", -degPos1 * deg2rad);
      fourMonoPanner1.setParamValue("Angle2", -degPos2 * deg2rad);
      fourMonoPanner1.setParamValue("Angle3", -degPos3 * deg2rad);
      fourMonoPanner1.setParamValue("Angle4", -degPos4 * deg2rad);

      // calculate gaussian volume distribution for each source
      radius1 = gaussianRadius(degPos1 * deg2rad);
      radius2 = gaussianRadius(degPos2 * deg2rad);
      radius3 = gaussianRadius(degPos3 * deg2rad);
      radius4 = gaussianRadius(degPos4 * deg2rad);

      // apply it inverse proportionaly
      scaled_radius1 = 5 * (1 - radius1);
      scaled_radius2 = 5 * (1 - radius2);
      scaled_radius3 = 5 * (1 - radius3);
      scaled_radius4 = 5 * (1 - radius4);

      fourMonoPanner1.setParamValue("Radius1", scaled_radius1); // 0 ~ 5
      fourMonoPanner1.setParamValue("Radius2", scaled_radius2);
      fourMonoPanner1.setParamValue("Radius3", scaled_radius3);
      fourMonoPanner1.setParamValue("Radius4", scaled_radius4);
      //   AudioInterrupts();
    }
  */
  if (currentMillis - lastMillisSerial > 1000) {
    lastMillisSerial = millis();
    displayData07();
    if (DEBUG) {
      Serial.print("Angle: "); Serial.print(degPos1); Serial.print(", "); Serial.print(degPos2); Serial.print(", "); Serial.print(degPos3); Serial.print(", "); Serial.println(degPos4);
      Serial.print("Radius: "); Serial.print(radius1); Serial.print(", "); Serial.print(radius2); Serial.print(", "); Serial.print(radius3); Serial.print(", "); Serial.println(radius4);
      Serial.print("Scaled_Radius: "); Serial.print(scaled_radius1); Serial.print(", "); Serial.print(scaled_radius2); Serial.print(", "); Serial.print(scaled_radius3); Serial.print(", "); Serial.println(scaled_radius4);
    }
  }

}

//float value=exp(-0.5*pow((x-mean)/sigma,2.));
//you will want to choose mean, sigma and scale the output value appropriately
float gaussianRadius(float x) {
  float gaussian_radius;
  float sigma = 1;
  gaussian_radius = exp(-0.5 * pow((x) / sigma, 2.));
  return gaussian_radius;
}

void playThisPlayerPauseOthers(int num_player) {
  for (int i = 0; i < numFiles; i++) {
    if (i == num_player) {
      if (player[i]->isPaused()){
        // 1fame = 1000 / 25ms. = 40ms. 
        //frame[i] * 40 = total_duration_in_ms
        //total_ellapsed_samples =  total_duration_in_ms * 44.1
        // so, 
        player[0]->setPosition(int(frame[i] * 40* 44.1 + 1200)); 
        player[i]->pause(false);
      }
    } else {
      if (mixer_gain[num_player] == 1.0) {
        if (player[i]->isPlaying())player[i]->pause(true);
        song_change_flag = false;
      }
    }
  }
}

void serialEvent() {
  serialState = 1;
  char serial_byte = Serial.read();

  if (DEBUG) {
    if (Serial.available()) {
      Serial.print("serial_byte = "); Serial.println(serial_byte);
    }
  }

  if (serial_byte == '1') {
    numZone = 1;
    fadeZone1();
    song_change_flag = true;

  } else if (serial_byte == '2') {
    numZone = 2;
    fadeZone2();
    song_change_flag = true;
  } else if (serial_byte == '3') {
    numZone = 3;
    fadeZone3();
    song_change_flag = true;
  } else if (serial_byte == '4') {
    fadeZone4();
    numZone = 4;
    song_change_flag = true;
  }

  Serial.clear();
  while (Serial.available()) {
    Serial.read();
  }
}
