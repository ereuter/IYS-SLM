#include <Arduino.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <SmartLEDShieldV4.h>


// Setup display
#define COLOR_DEPTH 24                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint8_t kMatrixWidth = 32;        // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 16;       // known working: 16, 32, 48, 64
const uint8_t kRefreshDepth = 36;       // known working: 24, 36, 48
const uint8_t kDmaBufferRows = 2;       // known working: 2-4, use 2 to save memory, more to keep from dropping frames and automatically lowering refresh rate
const uint8_t kPanelType = SMARTMATRIX_HUB75_16ROW_MOD8SCAN; // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels, or use SMARTMATRIX_HUB75_64ROW_MOD32SCAN for common 64x64 panels
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);

const int defaultBrightness = 128;
const int backgroundBrightness = 20;

const rgb24 backgroundColor = {0x33,0x00,0x55};
const rgb24 barColor = {0x33, 0x99, 0xff};
const rgb24 barOutlineColor = {0x33, 0x99, 0xff};
const rgb24 numberColor = {0xFF,0xFF,0x66};


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

uint8_t prescale = 20;  // Prescaler for RMS detector output
float offset = 91;    // Microphone calibration
float scaledRMS[9] = {0};
float Awt[9] = {0};
float AwtAccum = 0;
float LAeq = 0;
uint8_t n = 0; // number of measurements to average
unsigned long sampletime = 1000;  // Sampling interval
unsigned long starttime;
char str[5]; // Sound level to print to display

void setup(){

  AudioMemory(20);
  Serial.begin(115200);

  matrix.addLayer(&backgroundLayer);
  matrix.addLayer(&indexedLayer);

  matrix.begin();

  pinMode(18, INPUT);
  pinMode(19, INPUT);
  CORE_PIN16_CONFIG = (PORT_PCR_MUX(2) | PORT_PCR_PE | PORT_PCR_PS);
  CORE_PIN17_CONFIG = (PORT_PCR_MUX(2) | PORT_PCR_PE | PORT_PCR_PS);

  matrix.setBrightness(defaultBrightness);
  backgroundLayer.setBrightness(backgroundBrightness);

// Pink noise generator for debug
  pink1.amplitude(0.01);
  mixer1.gain(0,1);  // microphone level
  mixer1.gain(1,0);  // pink noise level


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
  filter8000.frequency(8000);
  filter8000.resonance(1.414);
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

//  Read and scale RMS values
        scaledRMS[0] = rms31.read()   * prescale;
        scaledRMS[1] = rms63.read()   * prescale;
        scaledRMS[2] = rms125.read()  * prescale;
        scaledRMS[3] = rms250.read()  * prescale;
        scaledRMS[4] = rms500.read()  * prescale;
        scaledRMS[5] = rms1000.read() * prescale;
        scaledRMS[6] = rms2000.read() * prescale;
        scaledRMS[7] = rms4000.read() * prescale;
        scaledRMS[8] = rms8000.read() * prescale;


// Apply A-weighting and square values from each band
        Awt[0] = scaledRMS[0] * 0.01071519;  //squared Vrms
        Awt[1] = scaledRMS[1] * 0.04897788;
        Awt[2] = scaledRMS[2] * 0.15667510;
        Awt[3] = scaledRMS[3] * 0.37153523;
        Awt[4] = scaledRMS[4] * 0.69183097;
        Awt[5] = scaledRMS[5]             ;
        Awt[6] = scaledRMS[6] * 1.1481536 ;
        Awt[7] = scaledRMS[7] * 1.1220185 ;
        Awt[8] = scaledRMS[8] * 0.8810489 ;

// Accumulate sums of sqaured voltages from each loop
        for (int i = 0; i < 9; i++){
          AwtAccum += sq(Awt[i]);
        }
        n++;  // Increment average counter for later averaging

        int bar[9] = {0}; // Octave-band bargraph heights

// Calculated SPL in each band and scale for display
        for (int i = 0; i < 9; i++){
          bar[i] = int((20 * log10f(scaledRMS[i]) + offset - 15)/5);
        }

        backgroundLayer.fillScreen(backgroundColor); // Set background

// Draw rectangles for each band
        for (int b = 0; b < 8; b++){
          const int barSpacing = 4;
          const int barWidth = 3;
          int x0 = b * barSpacing;
          int y0 = kMatrixHeight;
          int x1 = b * barSpacing + barWidth - 1;
          int y1 = kMatrixHeight - bar[b];
          backgroundLayer.fillRectangle(x0, y0, x1, y1, barOutlineColor, barColor);
        }
        backgroundLayer.swapBuffers();
      }
    }

// Power is proportional to V^2.  AwtAccum is a sum of V^2 values.
// Average power is sigma(P)/n, equivalent to sigma(V^2)/n.
// 10Log(sigma(V^2)/n) = average SPL
// offset = mic calibration

    LAeq = round(10 * log10f(AwtAccum / n) + offset);
    //Serial.println(LAeq,0);  // Report 0 decimal places

    sprintf(str, "%d", int(LAeq));

    indexedLayer.fillScreen(0);
    indexedLayer.setIndexedColor(0, numberColor);
    indexedLayer.setFont(font8x13);
    if (LAeq < 100) {
      indexedLayer.drawString(12, 1, 0, str);
    } else {
      indexedLayer.drawString(4, 1, 0, str);
    }

    indexedLayer.swapBuffers();
    //Serial.println(AudioProcessorUsage());
}
