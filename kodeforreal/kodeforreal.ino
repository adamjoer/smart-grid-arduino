#include "avr/interrupt.h"

volatile int measurement = 0;

void ADCsetup() {

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
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_RESSEL_10BIT | ADC_CTRLB_FREERUN;

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
  while (ADC->STATUS.bit.SYNCBUSY);

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
}

void ADCClockSetup() {

  // Enable the 8MHz internal oscillator
  SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_ENABLE;

  // Setup clock GCLK3 for no div factor
   GCLK->GENDIV.reg |= GCLK_GENDIV_ID(3)| GCLK_GENDIV_DIV(1);
   while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  //configure the generator of the generic clock, which is 8MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_ID(3) | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  //enable clock, set gen clock number, and ID to where the clock goes (30 is ADC)
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(3) | GCLK_CLKCTRL_ID(30);
  while (GCLK->STATUS.bit.SYNCBUSY);
}

// This is the interrupt service routine (ISR) that is called
// if an ADC measurement falls out of the range of the window
void ADC_Handler() {

  // Save ADC measurement
  measurement = ADC->RESULT.reg;

  // Input ADC measurement in DAC
  DAC->DATA.reg = measurement;

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; //Need to reset interrupt
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
  Serial.begin(9600);

  ADCClockSetup();
  ADCsetup();
  DACSetup();

  while (GCLK->STATUS.bit.SYNCBUSY);
}

void loop() {
}
