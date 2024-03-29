#include <Arduino.h>

// Comment this out to disable connection to Arduino Cloud
#define IOT_ENABLED

// Comment this out to disable frequency output to LCD
#define LCD_ENABLED

#ifdef IOT_ENABLED
#include "thingProperties.h"
#endif

#ifdef LCD_ENABLED
#include <LiquidCrystal.h>
#include "avr/dtostrf.h"
#endif

#include "avr/interrupt.h"
#include "core_cm0plus.h"

// Constant for ADC sampling
constexpr int SAMPLING_RATE = 10000;
constexpr int SAMPLING_RATE_FREQ = 9975;
constexpr int ZERO = 511;

// Constants for low pass filter
constexpr int CUTOFF_FREQ = 50;

constexpr float RC = 1.0 / (2 * PI * CUTOFF_FREQ);

constexpr float ALPHA = (1.0 / SAMPLING_RATE) / (RC + (1.0 / SAMPLING_RATE));

// LED PINS
constexpr int RED_LED_PIN = 9;
constexpr int YELLOW_LED_PIN = 6;
constexpr int GREEN_LED_PIN = 7;

// H-BRIDGE MOTOR CONTROL PINS
constexpr int DC_MOTOR_CLOCKWISE_PIN = 0;
constexpr int DC_MOTOR_COUNTER_CLOCKWISE_PIN = 2;

// Global variables

#ifndef IOT_ENABLED
volatile float frequency = 50;
volatile float xrms = 1;
constexpr float lowerFrequencyThreshold = 49.9;
constexpr float upperFrequencyThreshold = 50.1;
#endif

volatile int rawMeasurement = 0;

volatile int filterOutput = 0;
volatile int lockedFilterOutput = 0;

volatile int cumulativeXSquared = 0;
volatile int lockedCumulativeXSquared = 0;

volatile int previousFilterOutput;
volatile int lockedPreviousFilterOutput = 0;

volatile int sampleCounter = 0;

volatile int samplesPerPeriod = 0;
volatile int lockedSamplesPerPeriod = 0;

volatile float interpolatedZeroCrossing = 0;

volatile float previousFrequency = 0;

volatile bool zeroCrossingFlag = false;

#ifdef LCD_ENABLED
LiquidCrystal lcd(10, 8, 5, 4, 3, 1);
#endif

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
    GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(3) | GCLK_CLKCTRL_ID(30);
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;
}

// Set up timer to run ADC sampler
// Runs on GCLK4
void TC5Setup() {

    // Run on general clock 4, divide by 1
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(4) | GCLK_GENDIV_DIV(1);

    // Enable, set clock to OSC8 MHz clock source, select GCLK4
    GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_ID(4);

    // Wait for bus synchronization
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;

    // Enable, route GCLK4 to timer 4
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TC4_TC5;

    // Set wave generation to match frequency
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_WAVEGEN_MFRQ;

    // Wait for bus synchronization
    while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY)
        ;

    // Set counter compare value based on clock speed and sampling rate
    TC5->COUNT16.CC[0].reg = 8e6 / SAMPLING_RATE;

    // Wait for bus synchronization
    while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY)
        ;

    // Enable interrupt on compare match value
    TC5->COUNT16.INTENSET.bit.MC0 = 1;

    // Enable TC5
    TC5->COUNT16.CTRLA.bit.ENABLE = 1;

    // Wait for bus synchronization
    while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY)
        ;

    // Enable IRQ for TC5
    NVIC_EnableIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 3);
}

// Set up ADC. ADC Listens on port A3 (PA04)
void ADCsetup() {

    // Set up clock before anything else
    TC5Setup();

    ADCClockSetup();

    // Wait for bus synchronization.
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;

    uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
    uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
    linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

    /* Wait for bus synchronization. */
    while (ADC->STATUS.bit.SYNCBUSY)
        ;

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
    NVIC_EnableIRQ(ADC_IRQn);// enable ADC interrupts
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
    return (1.0f / ((1.0f / (float) SAMPLING_RATE_FREQ) * nSamples));
}

inline float linearInterpolation(int y, int oldY, int time) {
    return (float) time - (float) (y - ZERO) / (float) ((y - ZERO) - (oldY - ZERO));
}

// This is the interrupt service routine (ISR) that is called
// if an ADC measurement falls out of the range of the window
void ADC_Handler() {

    // Save ADC measurement in an int
    rawMeasurement = ADC->RESULT.reg;

    previousFilterOutput = filterOutput;

    // Use lowPassFilter to smooth out the measurements
    filterOutput = lowPassFilter(ALPHA, (float) rawMeasurement, (float) previousFilterOutput);

    // each rawMeasurement minus ''zero'' to get ''negative'' values. And Multiplied by a small constant
    cumulativeXSquared += ((int) (rawMeasurement * 1.055) - ZERO) * ((int) (rawMeasurement * 1.055) - ZERO);

    samplesPerPeriod++;
    // The zero crossing logic, going from negative to positive measurements
    if ((previousFilterOutput < ZERO && filterOutput >= ZERO)) {
        zeroCrossingFlag = true;
        // lock the values to have the variables ready for
        // the calculations in main and prevent overwriting of data
        lockedCumulativeXSquared = cumulativeXSquared;
        lockedFilterOutput = filterOutput;
        lockedPreviousFilterOutput = previousFilterOutput;
        lockedSamplesPerPeriod = samplesPerPeriod;

        samplesPerPeriod = 0;
        cumulativeXSquared = 0;
    }

    // Reset ADC interrupt
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
}

// This is the interrupt service routine (ISR) that is called
// periodically by the timer, based on the sampling rate
void TC5_Handler() {

    sampleCounter++;

    // Start ADC conversion
    ADC->SWTRIG.bit.START = 1;

    // Reset interrupt flag
    TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}

void setup() {
    // Initialize serial and wait for port to open:
    Serial.begin(9600);
    // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
    delay(1500);

    // Call the LEDs and motor digital pins as outputs.
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    pinMode(DC_MOTOR_CLOCKWISE_PIN, OUTPUT);
    pinMode(DC_MOTOR_COUNTER_CLOCKWISE_PIN, OUTPUT);

#ifdef IOT_ENABLED
    // Defined in thingProperties.h
    initProperties();

    // Connect to Arduino IoT Cloud
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);

    /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
    */
    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
#endif

#ifdef LCD_ENABLED
    lcd.begin(16, 2);
#endif

    ADCsetup();
    // DACSetup();
}

void loop() {

#ifdef IOT_ENABLED
    ArduinoCloud.update();
#endif

    if (zeroCrossingFlag) {

        interpolatedZeroCrossing = linearInterpolation(lockedFilterOutput, lockedPreviousFilterOutput, lockedSamplesPerPeriod);

        previousFrequency = frequency;
        frequency = lowPassFilter(ALPHA, calculateFrequency(interpolatedZeroCrossing), previousFrequency);
        xrms = (float) sqrt((1.0 / interpolatedZeroCrossing) * (float) lockedCumulativeXSquared);
        xrms = ((xrms / 1023.0) * 3.1) * 212;

#ifdef LCD_ENABLED
        char formatNumberBuffer[16];
        char outputBuffer[16];

        snprintf(outputBuffer, sizeof(outputBuffer), "f:   %s Hz", dtostrf(frequency, 5, 3, formatNumberBuffer));
        lcd.setCursor(0, 0);
        lcd.print(outputBuffer);

        snprintf(outputBuffer, sizeof(outputBuffer), "RMS: %s V", dtostrf(xrms, 5, 2, formatNumberBuffer));
        lcd.setCursor(0, 1);
        lcd.print(outputBuffer);
#endif

        int redLedValue = LOW;
        int yellowLedValue = LOW;
        int greenLedValue = LOW;

        int dcMotorClockwiseValue = LOW;
        int dcMotorCounterClockwiseValue = LOW;

        if (frequency < lowerFrequencyThreshold) {
            redLedValue = HIGH;
            dcMotorClockwiseValue = HIGH;

#ifdef IOT_ENABLED
            isFrequencyWithinThresholds = false;
#endif

        } else if (frequency > upperFrequencyThreshold) {
            yellowLedValue = HIGH;
            dcMotorCounterClockwiseValue = HIGH;

#ifdef IOT_ENABLED
            isFrequencyWithinThresholds = false;
#endif

        } else {
            greenLedValue = HIGH;

#ifdef IOT_ENABLED
            isFrequencyWithinThresholds = true;
#endif
        }

        digitalWrite(RED_LED_PIN, redLedValue);
        digitalWrite(YELLOW_LED_PIN, yellowLedValue);
        digitalWrite(GREEN_LED_PIN, greenLedValue);

        digitalWrite(DC_MOTOR_CLOCKWISE_PIN, dcMotorClockwiseValue);
        digitalWrite(DC_MOTOR_COUNTER_CLOCKWISE_PIN, dcMotorCounterClockwiseValue);

        zeroCrossingFlag = false;
    }
}
