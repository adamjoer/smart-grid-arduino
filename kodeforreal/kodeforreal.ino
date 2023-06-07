#include "Timer5.h"
#include "avr/interrupt.h"

int led = 10;
volatile int count=0;
volatile int measurement=0;
long t = millis();    // timer variable for printout
const int WAVE_SAMPLES_COUNT = 4096;

int sineWaveSamples[WAVE_SAMPLES_COUNT] = {0};

void ADCsetup() {

  /* Enable the APB clock for the ADC. */
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  /* Enable GCLK1 for the ADC */
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_ADC;

  /* Wait for bus synchronization. */
  while (GCLK->STATUS.bit.SYNCBUSY);

  /* Use the internal VCC reference. This is 1/2 of what's on VCCA.
    since VCCA is typically 3.3v, this is 1.65v.
  */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  /* Only capture one sample. The ADC can actually capture and average multiple
    samples for better accuracy, but there's no need to do that for this
    example.
  */
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  /* Set the clock prescaler to 512, which will run the ADC at
    8 Mhz / 512 = 31.25 kHz.
    Set the resolution to 10bit.
  */
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_10BIT;

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

  /* Set PA04 as an input pin. */
  PORT->Group[1].DIRCLR.reg = PORT_PA04;

  /* Enable the peripheral multiplexer for PA04. */
  PORT->Group[1].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;

  /* Set PA04 to function B which is analog input. */
  PORT->Group[1].PMUX[4].reg = PORT_PMUX_PMUXO_B;

  /* Enable interrupt for ready conversion interrupt, Result Conversion Ready:
   * RESRDY*/
  ADC->INTENSET.reg |= ADC_INTENSET_RESRDY;
  NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupts

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY) {
  };

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
}

// This is the interrupt service routine (ISR) that is called
// if an ADC measurement falls out of the range of the window
void ADC_Handler() {
  measurement = ADC->RESULT.reg;
  DAC->DATABUF.reg = measurement;
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; //Need to reset interrupt
}

void TimerSetup() {
  Serial.println("starting timer");

  // define frequency of interrupt; 1000 = approx. every 1 ms.
  MyTimer5.begin(1000);

  // Define the interrupt callback function
  MyTimer5.attachInterrupt(Timer5_IRQ);

  // Start the timer
  MyTimer5.start();
}

void DACOn() {
  // Enable DAC
  DAC->CTRLA.bit.ENABLE = 1;
}

void DACOff() {
  // Disable DAC
  DAC->CTRLA.bit.ENABLE = 0;
}

void DACSetup() {
  // Use analog voltage supply as reference selection
  DAC->CTRLB.bit.REFSEL = 0x01;

  // Enable output buffer
  DAC->CTRLB.bit.EOEN = 1;

  // Enable DAC
  DACOn();
}

void setup() {
  pinMode(led, OUTPUT);

  Serial.begin(9600);

  ADCsetup();
  DACSetup();
  TimerSetup();

  pinMode(led,OUTPUT);

  while (GCLK->STATUS.bit.SYNCBUSY);

  generateSineWaveSamples();
}

void generateSineWaveSamples() {
  const float PI2 = 3.14159 * 2;

  for (int i = 0; i < WAVE_SAMPLES_COUNT; ++i) {

    //calculate value in radians for sin()
    float in = PI2 * (1 / (float) WAVE_SAMPLES_COUNT) * (float) i;

    // Calculate sine wave value and offset based on DAC resolution 511.5 = 1023/2
    sineWaveSamples[i] = ((int) (sin(in) * 511.5 + 511.5));
  }
}

void Timer5_IRQ() {
  ADC->SWTRIG.bit.START = true;
}

void loop() {
}
