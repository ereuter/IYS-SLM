#include <Arduino.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthNoisePink      pink1;          //xy=117,268
AudioInputI2S            i2s1;           //xy=118,193
AudioMixer4              mixer1;         //xy=260,240
AudioFilterStateVariable filter31;       //xy=456,195
AudioFilterStateVariable filter63;       //xy=456,235
AudioFilterStateVariable filter125;      //xy=456,275
AudioFilterStateVariable filter250;      //xy=456,315
AudioFilterStateVariable filter500;      //xy=456,355
AudioFilterStateVariable filter1000;     //xy=456,395
AudioFilterStateVariable filter2000;     //xy=456,435
AudioFilterStateVariable filter4000;     //xy=456,475
AudioFilterStateVariable filter8000;     //xy=459,528
AudioAnalyzeRMS          rms31;          //xy=630,195
AudioAnalyzeRMS          rms63;          //xy=630,235
AudioAnalyzeRMS          rms125;         //xy=630,275
AudioAnalyzeRMS          rms250;         //xy=630,315
AudioAnalyzeRMS          rms500;         //xy=630,355
AudioAnalyzeRMS          rms1000;        //xy=630,395
AudioAnalyzeRMS          rms2000;        //xy=630,435
AudioAnalyzeRMS          rms4000;        //xy=630,475
AudioAnalyzeRMS          rms8000;        //xy=630,515
AudioConnection          patchCord1(pink1, 0, mixer1, 1);
AudioConnection          patchCord2(i2s1, 0, mixer1, 0);
AudioConnection          patchCord3(mixer1, 0, filter31, 0);
AudioConnection          patchCord4(mixer1, 0, filter63, 0);
AudioConnection          patchCord5(mixer1, 0, filter125, 0);
AudioConnection          patchCord6(mixer1, 0, filter250, 0);
AudioConnection          patchCord7(mixer1, 0, filter500, 0);
AudioConnection          patchCord8(mixer1, 0, filter1000, 0);
AudioConnection          patchCord9(mixer1, 0, filter2000, 0);
AudioConnection          patchCord10(mixer1, 0, filter4000, 0);
AudioConnection          patchCord11(mixer1, 0, filter8000, 0);
AudioConnection          patchCord12(filter31, 1, rms31, 0);
AudioConnection          patchCord13(filter63, 1, rms63, 0);
AudioConnection          patchCord14(filter125, 1, rms125, 0);
AudioConnection          patchCord15(filter250, 1, rms250, 0);
AudioConnection          patchCord16(filter500, 1, rms500, 0);
AudioConnection          patchCord17(filter1000, 1, rms1000, 0);
AudioConnection          patchCord18(filter2000, 1, rms2000, 0);
AudioConnection          patchCord19(filter4000, 1, rms4000, 0);
AudioConnection          patchCord20(filter8000, 1, rms8000, 0);
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
  AudioMemory(20);
  Serial.begin(115200);

  pink1.amplitude(0.01);

// Reverse for internal pink noise generator
  mixer1.gain(0,1);
  mixer1.gain(1,0);


// Setup filter center frequencies and Q (1 octave)
  filter31.frequency(31.5);
  filter31.resonance(1.414);
  filter63.frequency(63);
  filter63.resonance(1.414);
  filter125.frequency(125);
  filter125.resonance(1.414);
  filter250.frequency(250);
  filter250.resonance(1.414);
  filter500.frequency(500);
  filter500.resonance(1.414);
  filter1000.frequency(1000);
  filter1000.resonance(1.414);
  filter2000.frequency(2000);
  filter2000.resonance(1.414);
  filter4000.frequency(4000);
  filter4000.resonance(1.414);
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

// Output unweighted octave-band values (debug)
        // Serial.print(20 * log10f(rms31.read())+offset);
        // Serial.print(", ");
        // Serial.print(20 * log10f(rms63.read())+offset);
        // Serial.print(", ");
        // Serial.print(20 * log10f(rms125.read())+offset);
        // Serial.print(", ");
        // Serial.print(20 * log10f(rms250.read())+offset);
        // Serial.print(", ");
        // Serial.print(20 * log10f(rms500.read())+offset);
        // Serial.print(", ");
        // Serial.print(20 * log10f(rms1000.read())+offset);
        // Serial.print(", ");
        // Serial.print(20 * log10f(rms2000.read())+offset);
        // Serial.print(", ");
        // Serial.print(20 * log10f(rms4000.read())+offset);
        // Serial.print(", ");
        // Serial.println(20 * log10f(rms8000.read())+offset);

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
