#include <Arduino.h>

#include "avr/interrupt.h"
#include "core_cm0plus.h"

// Constant for ADC sampling
constexpr int SAMPLING_RATE = 10000;
constexpr int GENERATION_RATE = 51200;
constexpr int ZERO = 511;

// Constants for low pass filter
constexpr int CUTOFF_FREQ = 80;

constexpr float RC = 1.0 / (2 * PI * CUTOFF_FREQ);

constexpr float ALPHA = (1.0 / SAMPLING_RATE) / (RC + (1.0 / SAMPLING_RATE));

// FOR THE SINE WAVE DAC TIME
constexpr int WAVE_SAMPLES_COUNT = 1024;

// Global variables
volatile int DACCounter = 0;

int sinWaveSamples[WAVE_SAMPLES_COUNT] = {0};

volatile int rawMeasurement = 0;

volatile int cumulativeRawMeasurement = 0;

volatile int filterOutput = 0;

volatile int previousFilterOutput;

volatile int sampleCounter = 0;

volatile int previousSampleCounter;

volatile int samplesPerPeriod = 0;

volatile float interpolatedZeroCrossing = 0;

volatile unsigned long previousTime = millis();

volatile float frequency = 0;

// Set up clock for ADC. ADC clock is configured to run at 48MHz
void ADCClockSetup() {

  SYSCTRL->OSC8M.reg = SYSCTRL_OSC8M_ENABLE;


  // Setup clock GCLK3 for no div factor
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(3) | GCLK_GENDIV_DIV(1);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  // configure the generator of the generic clock, which is 48MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC8M |
                       GCLK_GENCTRL_ID(3) | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  // enable clock, set gen clock number, and ID to where the clock goes (30 is
  // ADC)
  GCLK->CLKCTRL.reg |=
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(3) | GCLK_CLKCTRL_ID(30);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
}

// Set up timer to run ADC sampler
// Runs on GCLK4
void TCC0Setup() {

  // Run on general clock 4, divide by 1
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(4) | GCLK_GENDIV_DIV(1);

  // Enable, set clock to OSC8 MHz clock source, select GCLK4
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_ID(4);

  // Wait for bus synchronization
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Enable, route GCLK4 to timer 4
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;

  // Set wave generation to match frequency
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_MFRQ;

  // Wait for bus synchronization
  while (TCC0->SYNCBUSY.bit.WAVE)
    ;

  // Set counter compare value based on clock speed and sampling rate
  TCC0->CC[0].reg = 8e6 / SAMPLING_RATE;

  // Wait for bus synchronization
  while (TCC0->SYNCBUSY.bit.CC0)
    ;

  // Enable interrupt on compare match value
  TCC0->INTENSET.bit.MC0 = 1;

  // Enable TCC0
  TCC0->CTRLA.bit.ENABLE = 1;

  // Wait for bus synchronization
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ;

  // Enable IRQ for TCC0
  NVIC_EnableIRQ(TCC0_IRQn);
}

// Set up ADC. ADC Listens on port A3 (PA04)
void ADCsetup() {

  // Set up clock before anything else
  TCC0Setup();

  ADCClockSetup();

  // Wait for bus synchronization.
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Use the internal VCC reference. This is 1/2 of what's on VCCA.
  // since VCCA is typically 3.3v, this is 1.65v.
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  // Only capture one sample. The ADC can actually capture and average multiple
  // samples for better accuracy, but there's no need to do that for this
  // example.
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  // Set the clock prescaler to 16, which will run the ADC at
  // 8 Mhz / 16 = 500 kHz.
  // Set the resolution to 10bit.
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_RESSEL_10BIT;

  /* Configure the input parameters.

    - GAIN_DIV2 means that the input voltage is halved. This is important
      because the voltage reference is 1/2 of VCCA. So if you want to
      measure 0-3.3v, you need to halve the input as well.

    - MUXNEG_GND means that the ADC should compare the input value to GND.

    - MUXPOST_PIN3 means that the ADC should read from AIN3, or PA04.
      This is A2 on the Feather M0 board.
  */
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND |
                       ADC_INPUTCTRL_MUXPOS_PIN4;

  // Set PA04 as an input pin.
  PORT->Group[1].DIRCLR.reg = PORT_PA04;

  // Enable the peripheral multiplexer for PA04.
  PORT->Group[1].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;

  // Set PA04 to function B which is analog input.
  PORT->Group[1].PMUX[4].reg = PORT_PMUX_PMUXO_B;

  // Enable interrupt for ready conversion interrupt, Result Conversion Ready:
  // RESRDY
  ADC->INTENSET.reg |= ADC_INTENSET_RESRDY;
  NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupts

  // Wait for bus synchronization.
  while (ADC->STATUS.bit.SYNCBUSY)
    ;

  // Enable the ADC.
  ADC->CTRLA.bit.ENABLE = true;
}

void DACClockSetup() {
  // Enable the 48MHz internal oscillator
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;

  // Setup clock GCLK5 for no div factor
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(5) | GCLK_GENDIV_DIV(1);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  // configure the generator of the generic clock, which is 48MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M |
                       GCLK_GENCTRL_ID(5) | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  // enable clock, set gen clock number, and ID to where the clock goes (33 is
  // DAC)
  GCLK->CLKCTRL.reg |=
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(5) | GCLK_CLKCTRL_ID(33);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
}

// Set up timer to run DAC.
// This runs on GCLK6
void TCC2Setup() {

  // Run on general clock 6, divide by 1
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(6) | GCLK_GENDIV_DIV(1);

  // Enable, set clock to DFLL 48 MHz clock source, select GCL6
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(6);

  // Wait for bus synchronization
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Enable, route GCLK6 to timer TCC2
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID_TCC2_TC3;

  // Set wave generation to match frequency
  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_MFRQ;

  // Wait for bus synchronization
  while (TCC2->SYNCBUSY.bit.WAVE)
    ;

  // Set counter compare value based on clock speed and generation rate
  TCC2->CC[0].reg = 48e6 / GENERATION_RATE;

  // Wait for bus synchronization
  while (TCC2->SYNCBUSY.bit.CC0)
    ;

  // Enable interrupt on compare match value
  TCC2->INTENSET.bit.MC0 = 1;

  // Enable TCC0
  TCC2->CTRLA.bit.ENABLE = 1;

  // Wait for bus synchronization
  while (TCC2->SYNCBUSY.bit.ENABLE)
    ;

  // Enable IRQ for TCC0
  NVIC_EnableIRQ(TCC2_IRQn);
}

inline void DACOn() {
  // Enable DAC
  DAC->CTRLA.bit.ENABLE = 1;
}

inline void DACOff() {
  // Disable DAC
  DAC->CTRLA.bit.ENABLE = 0;
}

// Set up DAC
void DACSetup() {

  // Set up timer before anything else
  TCC2Setup();

  DACClockSetup();

  // Use analog voltage supply as reference selection
  DAC->CTRLB.bit.REFSEL = 1;

  // Enable output buffer
  DAC->CTRLB.bit.EOEN = 1;

  // Enable DAC
  DACOn();
}

inline float lowPassFilter(float xn, float yn1) {
  return ALPHA * xn + (1 - ALPHA) * yn1;
}

inline float calculateFrequency(float nSamples) {
  return (1.0f / ((1.0f / (float)SAMPLING_RATE) * nSamples)) /*/ 2.0f*/;
}

inline float linearInterpolation(int y, int oldY, int time) {
  return (float)time - (float)(y - ZERO) / (float)((y - ZERO) - (oldY - ZERO));
}

// This is the interrupt service routine (ISR) that is called
// if an ADC measurement falls out of the range of the window
void ADC_Handler() {

  // Save ADC measurement
  rawMeasurement = ADC->RESULT.reg;

  cumulativeRawMeasurement += rawMeasurement;

  previousFilterOutput = filterOutput;

  filterOutput =
      lowPassFilter((float)rawMeasurement, (float)previousFilterOutput);

  samplesPerPeriod++;
  if ((previousFilterOutput < ZERO && filterOutput >= ZERO)/* ||
      (previousFilterOutput > ZERO && filterOutput <= ZERO)*/) {

    interpolatedZeroCrossing = linearInterpolation(
        filterOutput, previousFilterOutput, samplesPerPeriod);

    frequency = calculateFrequency(interpolatedZeroCrossing);

    samplesPerPeriod = 0;
  }

  // Reset ADC interrupt
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
}

// This is the interrupt service routine (ISR) that is called
//
void TCC2_Handler() {

  DAC->DATA.reg = sinWaveSamples[DACCounter];
  DACCounter = (DACCounter + 1) % WAVE_SAMPLES_COUNT;

  // Reset interrupt flag
  TCC2->INTFLAG.bit.MC0 = 1;
}

// This is the interrupt service routine (ISR) that is called
// periodically by the timer, based on the sampling rate
void TCC0_Handler() {

  sampleCounter++;

  // Start ADC conversion
  ADC->SWTRIG.bit.START = 1;

  // Reset interrupt flag
  TCC0->INTFLAG.bit.MC0 = 1;
}

void generateSineWaveSamples() {
  for (int i = 0; i < WAVE_SAMPLES_COUNT; ++i) {

    // calculate value in radians for sin()
    float in = PI * 2 * (1 / (float)WAVE_SAMPLES_COUNT) * (float)i;

    // Calculate sine wave value and offset based on DAC resolution 511.5 =
    // 1023/2
    sinWaveSamples[i] = ((int)(sin(in) * 511.5 + 511.5));
  }
}

void setup() {
  Serial.begin(9600);

  //  generateSineWaveSamples();

  ADCsetup();
  //  DACSetup();
}

// int k = 0;

void loop() {

  /*
    if (k++ % 3 == 0) {
      Serial.print(frequency);
      Serial.print(',');
      Serial.print(interpolatedZeroCrossing);

      Serial.println();
    }
  */

  const int INTERVAL_MS = 1000;

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= INTERVAL_MS) {

    int diff = sampleCounter - previousSampleCounter;

    Serial.print(currentTime);
    Serial.print(" ms: counter=");
    Serial.print(sampleCounter);
    Serial.print(", diff=");
    Serial.print(diff);
    Serial.print(", avg=");
    Serial.print(cumulativeRawMeasurement / diff);
    Serial.print(", filterOutput=");
    Serial.print(filterOutput);
    Serial.print(", frequency=");
    Serial.print(frequency);
    Serial.print(", interpolatedZeroCrossing=");
    Serial.print(interpolatedZeroCrossing);

    Serial.println();

    cumulativeRawMeasurement = 0;
    previousTime = currentTime;
    previousSampleCounter = sampleCounter;
  }
}
