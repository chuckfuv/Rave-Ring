/*
 * CISC 340: Team Seven, Project One
 * 
 * This module is responsible for taking sound coming from the microphone 
 * and then sending it out to a Bluetooth 4.0 (BT LE) capable host for more 
 * complex signal processing that would be well beyond the capabilities of 
 * the Arduino Microcontroller.
 * 
 * This Code Requires the following hardware components:
 *    - Any ATmega 328P arduino Board or Better
 *    - Bluetooth LE module
 *    - Adafruit Electret Microphone Amplifier
 * 
 * Original Authors: Rocky Petkov, Jack Qiao
 */

#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Adafruit_BLE.h>
#include <SPI.h>
#include <Adafruit_ATParser.h>

#include "Codebase_Arduino.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


#define FACTORYRESET_ENABLE 1 // Allows us to do a factory reset to put device in known good state!
#define FFT_N 128             // Not really using the library so i'll just define a constant to save memory.
#define MIC_ADC_CHANNEL 6     // Testing this with ADC Channel 6 (Note: Unable to find mapping for Flora Series)

#define PIXEL_PIN 12          // Neo Pixels are plugged into the D11 pin
#define NUM_COLOURS 1         // We display a maximum of one colour at a time. 
#define PIXEL_COUNT 24        // Number of pixels on our ring!

#define BLUEFRUIT_HWSERIAL_NAME Serial1;

// A couple of fields used when sampling incomming data
int16_t capture[FFT_N];                   // Audio Capture Buffers
volatile byte samplePos = 0; // Position Counter for the Buffer

// A couple globals which are useful for processing incomming colours. 
const char *delimiters = {"[,]"};                    // Delimiters for splitting strings
char       *pch;                                     // Pointer to the lead character in our string
uint8_t colour[3 * NUM_COLOURS] = {0, 0, 0};         // Three values for each colour. Initialise it to 0.
uint8_t previousColour[3 * NUM_COLOURS] = {0, 0, 0}; // Previous colour
uint8_t    pulseDimMode = DIM_MODE;                  // If we are pulsing set it to dim!


// Initialise our NeoPixel object
Adafruit_NeoPixel ring = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
 
// Initialise bluetooth function
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);


/*
 * A small helper to let us know if there is an error
 */
 void error(const __FlashStringHelper *err) {
  Serial.println(err);
  while (1);
 }

/*
 * Initialising the bluetooth connection as well as the neopixels!
 *
 */
void setup() {


   // Waiting for the serial link to be established
   while(!Serial);
   delay(500);
  
   // Start Serial communications so we can do output. 
   Serial.begin(115200);
   Serial.println(F("Welcome to the Rave Light Show!"));
   
  // Factory reset the bluetooth module so that everything is in a safe place
  // First need to begin verbose mode, or factory reset wont work.
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  
  Serial.println( F("OK!") );

  if (FACTORYRESET_ENABLE)
  {
    Serial.println("Resetting Bluetooth Module:");
    if (!ble.factoryReset()) {
      error(F("Unable to factory reset"));
    }
  }
  /* Disable command echo from Bluefruit 
     Need to disable this to establish connection. */
  ble.echo(false);
  ble.verbose(false);  // debug info is a little annoying after this point!
  Serial.println("Requesting Bluefruit info:");
  ble.info();

   Serial.println(F("Initialising the lights"));
  ring.begin();
  ring.show();

  // Wait for the incomming bluetooth connexion
  Serial.println("Waiting for connection...");
  while (! ble.isConnected()) {
    Serial.println(F("Stallin like Stalin!"));
    delay(500);
  }

  Serial.println("Initialising Random Number Generator");
  randomSeed(analogRead(6));  // Read A6 for a random number!

    Serial.println("Enabling Free Run Mode");
  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
  ADMUX  = MIC_ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz ((SAMPLE RATE))
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << MIC_ADC_CHANNEL; // Turn off digital input for ADC pin
  TIMSK0 = 0;                // Timer0 off

sei(); // Enabling interrupts. 
  
  Serial.println("\n\nBeginning Main Loop...");
}

/*
 * This loop works by task switching between handling input and output.
 * Currently I am taking the naive approach where I simply switch between tasks 
 * with out using any of the fancier constructs of the language. 
 * 
 * Steps:
 *  1) Wait on input from Microphone
 *  2) Check Bluetooth Pin, if there's data, receive and store
 *  3) Send data recorded from microphone
 *  4) Display lights based upon the data received
 *  
 *  Things to be Wary Of:
 *    1) The fact that we are dealing with an interrupt might complicate things.
 *      still, the code I'm working from does a DFT in the time between interrupts 
 *      so we should be good
 *      
 *    2) Memory
 *      We have somewhat limited memory to work with. This is unfortunate. 
 */
void loop() {
  
    int i, j;          // Loop counters
    int randomPattern;

    uint8_t bytesRead;                              // No. of bytes read to the buffer

  
    while(ADCSRA & _BV(ADIE));  // Wait for audio sampling to finish
    samplePos = 0;              // Reset Sample Counter
    ADCSRA |= _BV(ADIE); // Resumes the sampling interrupt
  
    // Copy current colour to previous colour buffer
    memcpy(previousColour, colour, 3 * NUM_COLOURS);  
  
    // Check for incoming characters from Bluefruit
    ble.println("AT+BLEUARTRX");
    ble.readline();
    Serial.println(ble.buffer);
    if (strcmp(ble.buffer, "OK") == 0) {
    }
    else {
        
      // Converting byte string into integers
      pch = strtok(ble.buffer, delimiters);
      i = 0;

     
        // While loop terminates when we run out of tokens or we have read more than 3 tokens
        while (pch != NULL && i < (3 * NUM_COLOURS)) {
            colour[i] = atoi(pch);    // Translate pch into an int and store in colour
            //Serial.println(pch);
            pch = strtok(NULL, delimiters);
            //Serial.println(colour[i]);
            i++;
        }
    ble.waitForOK();
    }
    
    // Here we do some boolean comparisons and branch off to do an appropriate pattern. 
    if (matchingColours(colour, previousColour, 3 * NUM_COLOURS) && random(0, 100) <= 30) {
      pulsePattern();   // Pulse Pattern and Return
      return;   
    }
    // We didn't pulse so we should set pulse to dim mode to be dim
    pulseDimMode = DIM_MODE;

    randomPattern = random(0, 100);

    /*
     * Randomly Choose if we want to do the block pattern or the pair pattern.
     * 65% chance for block pattern
     */
    if (randomPattern <= 70) {
      blockPattern();
    }
    else {
      pairPattern();
    }
    
    Serial.println("Sending...");
    // Sending one number at a time. 
    for (j = 0; j < FFT_N; j++) {
        ble.print("AT+BLEUARTTX=");
        ble.print(capture[j]);    // Print the ith value of capture
        ble.println(",");           // Add in a comma for easy parsing on the python end. 

        // Waiting to ensure that we are okay w.r.t. seding our data
        if (! ble.waitForOK()) {
            Serial.println(F("Failed to send buffer."));
        }   
    }
}  

/*
 * A simple method which compares two colour arrays for equality
 */
boolean matchingColours(uint8_t *colourArrayOne, uint8_t *colourArrayTwo, int arraySize) {
  int i;
  for (i=0 ; i < arraySize; i++) {
    if (colourArrayOne[i] != colourArrayTwo[i]) {
      return false;     // If a mismatch return false
    }
  }
  return true;  // If we emerge we have the same colour
}

/*
 * The simplest pattern. Naively displays the same colour 
 * for all of the lights in the ring.
 */
void blockPattern() {
    Serial.println("Block");
  
  int light;
  for(light = 0; light < PIXEL_COUNT; light++) {
    // Set the ring colour
    ring.setPixelColor(light, colour[0], colour[1], colour[2]);
  }
  ring.show();  // And latch them in!
  delay(20);
}

/*
 * A pattern which upon successive calls will change alternate a colour
 * between light and dark. 
 */
void pulsePattern() {
  int light;

  Serial.println("Pulse");

  // If we have the pulse on
  if (!pulseDimMode) {
    for (light = 0; light < PIXEL_COUNT; light++) {
      ring.setPixelColor(light, colour[0] / 2, colour[1] / 2, colour[2] / 2);
    }
  }
  else {
    // We want to multiply all values by 2. for a brighter version.
    for (light = 0; light < PIXEL_COUNT; light++) {
      ring.setPixelColor(light, colour[0] * 2, colour[1] * 2, colour[2] * 2); 
    }
  }
  ring.show();  // Propagate the changes
  delay(20);    // Delay for effect
  pulseDimMode = !pulseDimMode; // Flip the pulse pattern

  
}

/*
 * Moves a pair of dark pixels around the ring moving out from centre.
 */
void pairPattern() {
  int8_t darkPixels[2];      // Dark pixels are initialised. Pos 0 moves clockwise. Pos 1 moves counter clockwise. 
  int loopCount = 3;         // Three loops around with our pixel count
  int i, light;

  Serial.println("Pair");

  for (i = 0; i < loopCount; i++) {
    darkPixels[0] = 0;
    darkPixels[1] = PIXEL_COUNT; // PIXEL_COUNT % PIXEL_COUNT == 0.

    for (light = 0; light < PIXEL_COUNT; light++) {
      // If it's a light we wish to have off
      if (light == darkPixels[0] || light == darkPixels[1]) {
        ring.setPixelColor(light, 0, 0, 0); // Set it to black
      }
      else {  // Else we want to have the light on
        ring.setPixelColor(light, colour[0], colour[1], colour[2]);
      }
      ring.show();
      darkPixels[0]++;   // Increment the first pixel
      darkPixels[1]--;   // Decrement the second pixel
    }
  } 
}

/*
 * Generates a complimentary colour and displays it in alternating positions
 */
void complimentaryPattern() {
  int i, light;
  int loopCount = 5;
  int8_t *compliment;
  int modeCounter = 0;

  Serial.println("Complimentary");
  // Computing complimentary colours. 
  compliment[0] = sqrt(pow(255, 2) - pow(colour[0], 2));
  compliment[1] = sqrt(pow(255, 2) - pow(colour[1], 2));
  compliment[2] = sqrt(pow(255, 2) - pow(colour[2], 2));

  for (i = 0;i < loopCount; i++) {
    for (light = 0; light < PIXEL_COUNT / 2; light++) {
      if (modeCounter = 0) {  // Show sent colour
        ring.setPixelColor(light, colour[0], colour[1], colour[2]);
        ring.setPixelColor(light + 1, colour[0], colour[1], colour[2]);
      }
      else {  // Show compliment
        ring.setPixelColor(light, compliment[0], compliment[1], compliment[2]);
        ring.setPixelColor(light + 1, compliment[0], compliment[1], compliment[2]);
      }
      modeCounter = ~modeCounter;
    }
    ring.show();
    delay(30);
    modeCounter + ~modeCounter; // Flip for next time. 
  }
}

/*
 * Below is an interrupt service routine
 * which takes 128 audio samples and normalises them
 * around 0 for the DFT.
 */
ISR(ADC_vect) { 
    static const int16_t noiseThreshold = 4;  // We can play around with this. We don't want to have lights run on ambient nothing!
    int16_t sample = ADC; // Raw voltage over the wire. 0 corresponds to 0 volts. 1023 corresponds to ~5v.
    
    // Normalise our samples around 0 for the DFT
    capture[samplePos] = 
      ((sample > (512 - noiseThreshold)) &&
      sample < (512 + noiseThreshold)) ? 0 : 
      sample - 512;  

      // Here we check if our buffer is full. If so we turn off our interrupt. 
      if(++samplePos >= FFT_N) {
          ADCSRA &= ~_BV(ADIE);
      }
}

