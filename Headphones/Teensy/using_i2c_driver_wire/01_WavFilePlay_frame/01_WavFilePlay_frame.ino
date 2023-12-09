// Simple WAV file player example
//
// This example code is in the public domain.

#include <play_wav.h>
//#include <i2c_driver_wire.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>


#if defined(__IMXRT1062__)
#define T4
#include <utility/imxrt_hw.h> // make available set_audioClock() for setting I2S freq on Teensy 4
#else
#define F_I2S ((((I2S0_MCR >> 24) & 0x03) == 3) ? F_PLL : F_CPU) // calculation for I2S freq on Teensy 3
#endif

// GUItool: begin automatically generated code
AudioPlayWav             playWav1;
AudioPlayWav             playWav2;
AudioPlayWav             playWav3;

AudioMixer4              mixer1;         //xy=647,123
AudioMixer4              mixer3;         //xy=648,212
AudioOutputI2S           output;

AudioConnection          patchCord1(playWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playWav1, 1, mixer3, 0);
AudioConnection          patchCord3(playWav1, 2, mixer1, 1);
AudioConnection          patchCord4(playWav1, 3, mixer3, 1);
AudioConnection          patchCord5(playWav1, 4, mixer1, 2);
AudioConnection          patchCord6(playWav1, 5, mixer3, 2);
AudioConnection          patchCord7(playWav1, 6, mixer1, 3);
AudioConnection          patchCord8(playWav1, 7, mixer3, 3);
AudioConnection          patchCord9(mixer1, 0, output, 0);
AudioConnection          patchCord10(mixer3, 0, output, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=853,409
// GUItool: end automatically generated code

#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used

#define numFiles 3
AudioPlayWav *player[numFiles] = {
  &playWav1,  &playWav2,  &playWav3
};

const char *filename[numFiles] = {
  "BAC.WAV",
  "RBR.WAV",
  "00INTRO.WAV"

};

#ifdef T4
#else
// calculate I2S dividers for Teensy 3
uint32_t I2S_dividers( float fsamp, uint32_t nbits, uint32_t tcr2_div )
{

  unsigned fract, divi;
  fract = divi = 1;
  float minfehler = 1e7;

  unsigned x = (nbits * ((tcr2_div + 1) * 2));
  unsigned b = F_I2S / x;

  for (unsigned i = 1; i < 256; i++) {

    unsigned d = round(b / fsamp * i);
    float freq = b * i / (float)d ;
    float fehler = fabs(fsamp - freq);

    if ( fehler < minfehler && d < 4096 ) {
      fract = i;
      divi = d;
      minfehler = fehler;
      //Serial.printf("%fHz<->%fHz(%d/%d) Fehler:%f\n", fsamp, freq, fract, divi, minfehler);
      if (fehler == 0.0f) break;
    }

  }

  return I2S_MDR_FRACT( (fract - 1) ) | I2S_MDR_DIVIDE( (divi - 1) );
}
#endif


// set I2S samplerate
void setI2SFreq(int freq) {
#if defined(T4)
  // PLL between 27*24 = 648MHz und 54*24=1296MHz
  int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  int n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
  double C = ((double)freq * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);
  set_audioClock(c0, c1, c2, true);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
               | CCM_CS1CDR_SAI1_CLK_PRED(n1 - 1) // &0x07
               | CCM_CS1CDR_SAI1_CLK_PODF(n2 - 1); // &0x3f
#else
  unsigned tcr5 = I2S0_TCR5;
  unsigned word0width = ((tcr5 >> 24) & 0x1f) + 1;
  unsigned wordnwidth = ((tcr5 >> 16) & 0x1f) + 1;
  unsigned framesize = ((I2S0_TCR4 >> 16) & 0x0f) + 1;
  unsigned nbits = word0width + wordnwidth * (framesize - 1 );
  unsigned tcr2div = I2S0_TCR2 & 0xff; //bitclockdiv
  uint32_t MDR = I2S_dividers(freq, nbits, tcr2div);
  if (MDR > 0) {
    while (I2S0_MCR & I2S_MCR_DUF) {
      ;
    }
    I2S0_MDR = MDR;
  }
#endif
}

void setup() {
  Serial.begin(9600);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }


  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  sgtl5000_1.enable();
  sgtl5000_1.volume(0.55);
  AudioMemory(20);
  setI2SFreq(44100);

  player[0]->play(filename[0]);
  delay(100);
  player[0]->pause(true);
  player[0]->loop(true);
  player[0]->setPosition(4410000);
  player[0]->pause(false);

  //   playFile("BAC.WAV");
  // playFile("00INTRO.WAV");



}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);
  playWav1.play(filename);
  while (playWav1.isPlaying()) {}
}


void loop() {
  //playFile("Nums_7dot1_16_44100.wav");



}
