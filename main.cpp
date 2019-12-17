#include <Arduino.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=156,182
AudioFilterBiquad        biquad31;        //xy=385,130
AudioFilterBiquad        biquad63;        //xy=385,170
AudioFilterBiquad        biquad125;        //xy=385,210
AudioFilterBiquad        biquad250;        //xy=385,250
AudioFilterBiquad        biquad500;        //xy=385,290
AudioFilterBiquad        biquad1000;        //xy=385,330
AudioFilterBiquad        biquad2000;        //xy=385,370
AudioFilterBiquad        biquad4000;        //xy=385,410
AudioFilterBiquad        biquad8000;        //xy=385,450
AudioAnalyzeRMS          rms31;           //xy=574,130
AudioAnalyzeRMS          rms63;           //xy=574,170
AudioAnalyzeRMS          rms125;           //xy=574,210
AudioAnalyzeRMS          rms250;           //xy=574,250
AudioAnalyzeRMS          rms500;           //xy=574,290
AudioAnalyzeRMS          rms1000;           //xy=574,330
AudioAnalyzeRMS          rms2000;           //xy=574,370
AudioAnalyzeRMS          rms4000;           //xy=574,410
AudioAnalyzeRMS          rms8000;           //xy=574,450
AudioConnection          patchCord1(i2s1, 0, biquad63, 0);
AudioConnection          patchCord2(i2s1, 0, biquad125, 0);
AudioConnection          patchCord3(i2s1, 0, biquad250, 0);
AudioConnection          patchCord4(i2s1, 0, biquad500, 0);
AudioConnection          patchCord5(i2s1, 0, biquad1000, 0);
AudioConnection          patchCord6(i2s1, 0, biquad4000, 0);
AudioConnection          patchCord7(i2s1, 0, biquad8000, 0);
AudioConnection          patchCord8(i2s1, 0, biquad31, 0);
AudioConnection          patchCord9(i2s1, 0, biquad2000, 0);
AudioConnection          patchCord10(biquad63, rms63);
AudioConnection          patchCord11(biquad31, rms31);
AudioConnection          patchCord12(biquad125, rms125);
AudioConnection          patchCord13(biquad250, rms250);
AudioConnection          patchCord14(biquad500, rms500);
AudioConnection          patchCord15(biquad1000, rms1000);
AudioConnection          patchCord16(biquad2000, rms2000);
AudioConnection          patchCord17(biquad4000, rms4000);
AudioConnection          patchCord18(biquad8000, rms8000);
// GUItool: end automatically generated code

uint8_t prescale = 20;  // Prescaler for RMS detetor output
float offset = 88;    // Microphone calibration
float Awt = 0;
float AwtAccum = 0;
float LAeq = 0;
uint8_t n = 0; // number of measurements to average
unsigned long sampletime = 1000;  // Sampling interval
unsigned long starttime;

void setup(){
  AudioMemory(60);
  Serial.begin(115200);

// Setup filter center frequencies and Q (1 octave)
  biquad31.setBandpass(0, 32, 1.414);
  biquad63.setBandpass(0, 63, 1.414);
  biquad125.setBandpass(0, 125, 1.414);
  biquad250.setBandpass(0, 250, 1.414);
  biquad500.setBandpass(0, 500, 1.414);
  biquad1000.setBandpass(0, 1000, 1.414);
  biquad2000.setBandpass(0, 2000, 1.414);
  biquad4000.setBandpass(0, 4000, 1.414);
  biquad8000.setBandpass(0, 8000, 1.414);
}

void loop(){

  n = 0;
  AwtAccum = 0;
  starttime = millis();  // Get initial time

  // Loop and accumulate values during sampling interval
  while((millis() - starttime) < sampletime){

    // Wait untl data avaiable from all octave-band filters
    if (
            rms31.available() &&
            rms63.available() &&
            rms125.available() &&
            rms250.available() &&
            rms500.available() &&
            rms1000.available() &&
            rms2000.available() &&
            rms4000.available() &&
            rms8000.available()
        ){

        // Apply A-weighting and accumulate squared Vrms values from each band
        Awt = 0;
        Awt += sq(rms31.read()   * prescale * 0.01071519 );  //squared Vrms
        Awt += sq(rms63.read()   * prescale * 0.04897788 );
        Awt += sq(rms125.read()  * prescale * 0.15667510 );
        Awt += sq(rms250.read()  * prescale * 0.37153523 );
        Awt += sq(rms500.read()  * prescale * 0.69183097 );
        Awt += sq(rms1000.read() * prescale              );
        Awt += sq(rms2000.read() * prescale * 1.1481536  );
        Awt += sq(rms4000.read() * prescale * 1.1220185  );
        Awt += sq(rms8000.read() * prescale * 0.8810489  );
        // At this point, Awt is a sum of squared voltages.  Total voltage
        // would be sqrt of this value.

        AwtAccum += Awt;  // Accumulates sums of sqaured voltages from each loop
        n++;  // Increment average counter for later averaging
      }
      delay(1);  // Won't loop without this for some reason
    }

    // Power is proportional to V^2.  AwtAccum is a sum of V^2 values.
    // Average power is sigma(P)/n, equivalent to sigma(V^2)/n.
    // 10Log(sigma(V^2)/n) = average SPL
    // offset = mic calibration
    LAeq = round(10 * log10f(AwtAccum / n) + offset);
    Serial.println(LAeq,0);  // Report 0 decimal places
}
