#include <Arduino.h>
#include <LiquidCrystal.h>
#include "avr/dtostrf.h"
#include "avr/interrupt.h"
#include "core_cm0plus.h"

// Constant for ADC sampling
constexpr int SAMPLING_RATE = 10000;
constexpr int GENERATION_RATE = 51200;
constexpr int ZERO = 511;

// Constants for low pass filter
constexpr int CUTOFF_FREQ = 50;

constexpr float RC = 1.0 / (2 * PI * CUTOFF_FREQ);

constexpr float ALPHA = (1.0 / SAMPLING_RATE) / (RC + (1.0 / SAMPLING_RATE));

// FOR THE SINE WAVE DAC TIME
constexpr int WAVE_SAMPLES_COUNT = 1024;

// Global variables
volatile int DACCounter = 0;

int sinWaveSamples[WAVE_SAMPLES_COUNT] = {0};

volatile int rawMeasurement = 0;

volatile int filterOutput = 0;
volatile int lockedFilterOutput = 0;

volatile int cumulativeFilterOutput = 0;
volatile int lockedCumulativeFilterOutput = 0;

volatile int previousFilterOutput;
volatile int lockedPreviousFilterOutput = 0;

volatile int sampleCounter = 0;

volatile int previousSampleCounter;

volatile int samplesPerPeriod = 0;
volatile int lockedSamplesPerPeriod = 0;

volatile float interpolatedZeroCrossing = 0;

volatile unsigned long previousTime = millis();

volatile float frequency = 50;

volatile float previousFrequency = 0;

volatile float xrms = 0;

volatile bool zeroCrossingFlag = false;

LiquidCrystal lcd(10, 8, 5, 4, 3, 2);

// Set up clock for ADC. ADC clock is configured to run at 48MHz
void ADCClockSetup() {
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;

  // Setup clock GCLK3 for no div factor
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(3) | GCLK_GENDIV_DIV(1);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  // configure the generator of the generic clock, which is 48MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M |
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
  NVIC_SetPriority(TCC0_IRQn, 3);
}

// Set up ADC. ADC Listens on port A3 (PA04)
void ADCsetup() {

  // Set up clock before anything else
  TCC0Setup();

  ADCClockSetup();

  // Wait for bus synchronization.
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Write the calibration data. */
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  // Use the internal VCC reference. This is 1/2 of what's on VCCA.
  // since VCCA is typically 3.3v, this is 1.65v.
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  // Only capture one sample. The ADC can actually capture and average multiple
  // samples for better accuracy, but there's no need to do that for this
  // example.
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  // Set the clock prescaler to 16, which will run the ADC at
  // 48 Mhz / 16 = 3MHz.
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
  NVIC_SetPriority(ADC_IRQn, 0);
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
  //  TCC2Setup();

  DACClockSetup();

  // Use analog voltage supply as reference selection
  DAC->CTRLB.bit.REFSEL = 1;

  // Enable output buffer
  DAC->CTRLB.bit.EOEN = 1;

  // Enable DAC
  DACOn();
}

inline float lowPassFilter(float alpha, float xn, float yn1) {
  return alpha * xn + (1 - alpha) * yn1;
}

inline float calculateFrequency(float nSamples) {
  return (1.0f / ((1.0f / (float)9968) * nSamples));
}

inline float linearInterpolation(int y, int oldY, int time) {
  return (float)time - (float)(y - ZERO) / (float)((y - ZERO) - (oldY - ZERO));
}

// This is the interrupt service routine (ISR) that is called
// if an ADC measurement falls out of the range of the window
void ADC_Handler() {

  // Save ADC measurement
  rawMeasurement = ADC->RESULT.reg;

  // DAC->DATABUF.reg=rawMeasurement;

  previousFilterOutput = filterOutput;

  filterOutput = lowPassFilter(ALPHA, (float)rawMeasurement, (float)previousFilterOutput);

  // Maybe multiply with constant 1.41 something
  cumulativeFilterOutput += ((int)(rawMeasurement)-511)*((int)(rawMeasurement)-511);

  samplesPerPeriod++;

  if ((previousFilterOutput < ZERO && filterOutput >= ZERO)) {
    zeroCrossingFlag=true;
    lockedCumulativeFilterOutput = cumulativeFilterOutput;
    lockedFilterOutput = filterOutput;
    lockedPreviousFilterOutput = previousFilterOutput;
    lockedSamplesPerPeriod = samplesPerPeriod;

    samplesPerPeriod = 0;
    cumulativeFilterOutput = 0;
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

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Frequency:");

  //  generateSineWaveSamples();

  ADCsetup();
  // DACSetup();
}

void loop() {

  char frequencyBuffer[16];
  char outputBuffer[16];
  snprintf(outputBuffer, sizeof(outputBuffer), "%s Hz", dtostrf(frequency, 7, 4, frequencyBuffer));

  lcd.setCursor(0, 1);
  lcd.print(outputBuffer);

  if (zeroCrossingFlag) {

    interpolatedZeroCrossing = linearInterpolation(lockedFilterOutput, lockedPreviousFilterOutput, lockedSamplesPerPeriod);

    previousFrequency = frequency;
    frequency = lowPassFilter(ALPHA, calculateFrequency(interpolatedZeroCrossing), previousFrequency);
    xrms = (float)sqrt((1.0 / interpolatedZeroCrossing) * (float)lockedCumulativeFilterOutput);
    xrms = (xrms / 1023.0) * 3.1;
    // xrms = (((float) maxVolt - (float) minVolt) / 2.0) / sqrt(2);
    zeroCrossingFlag = false;
  }


  const int INTERVAL_MS = 1000;

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= INTERVAL_MS) {

    int diff = sampleCounter - previousSampleCounter;

    Serial.print(currentTime);
    Serial.print(" ms: counter=");
    Serial.print(sampleCounter);
    Serial.print(", diff=");
    Serial.print(diff);
    Serial.print(", filterOutput=");
    Serial.print(filterOutput);
    Serial.print(", frequency=");
    Serial.print(frequency, 4);
    Serial.print(", interpolatedZeroCrossing=");
    Serial.print(interpolatedZeroCrossing);
    Serial.print(", cumFilterOut");
    Serial.print(lockedCumulativeFilterOutput);
    Serial.print(", XRMS=");
    Serial.print(xrms, 4);
    Serial.println();

    previousTime = currentTime;
    previousSampleCounter = sampleCounter;
  }
}
