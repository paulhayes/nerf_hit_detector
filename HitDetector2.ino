
#include <oledDirect.h>
#include <SD.h>

#ifndef __AVR__
#include <SPI.h>
#endif

#define CS	8
#define DC	9
#define RESET	7

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

int ledPin = 13;

const unsigned int numSensors = 2;
const unsigned int numSamples = 120;

const unsigned int yHeight = 128 / numSensors;
const int yHalfHeight = yHeight >> 1;
const float yScale = 1.0 * yHeight / 2048;

unsigned long sampleInterval = 16;
unsigned long lastSample;

int valA = 0;
int valB = 0;
int valC = 0;
int valD = 0;
int valE = 0;

int statePin = LOW;

int THRESHOLD = 250;

long accum = 0;

volatile bool readingAnalog = false;

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

unsigned long sensorTimes[numSensors];
volatile int sensorValues[numSensors][numSamples];  
uint8_t sensorFlags = 0;
uint8_t sensorMask = (1<<numSensors)-1;
volatile uint8_t readFlag;
volatile int analogVal;
volatile uint8_t samplesComplete;

uint8_t analogInputs[5] = { 0,2,2,3,4 };

volatile byte currentAnalogIndex = 0;
volatile byte currentSampleIndex = 0;

SSD1351 *oled;

void setup() {
 oled = new SSD1351(CS, DC, RESET);
 oled->clearScreen(Colour::Black);
 
 pinMode(ledPin, OUTPUT); 



   // set up the ADC
  //ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  //ADCSRA |= PS_16;    // set our own prescaler to 64 
  
  setupFreeRunningAnalog();
  
  //Serial.println(freeRam());
}

// Initialization
void setupFreeRunningAnalog(){
  
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
  
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= B11000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
  
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
    
  ADMUX |= 0;
  // ADMUX |= B00001000; // Binary equivalent
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
  
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= B00100000;
  
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= B11111000;
  
  // Set the Prescaler to 4 (16000KHz/ = 125MHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= B00000111 ;
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;
  
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();
  
  // Kick off the first ADC
  readFlag = 0;
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;
}

void loop() {
  
  if (readFlag == 1){
    //turn off internupt; 
    //ADCSRA &= ~B00001000;

    // Perform whatever updating needed
    byte analogHitIndex = currentAnalogIndex;
    // Move to next analog input

    /*
    if( sensorMask != sensorFlags ){
      currentAnalogIndex++;
      if(currentAnalogIndex>=numSensors) currentAnalogIndex=0;
      while(sensorFlags&(1<<currentAnalogIndex)){
        currentAnalogIndex++;
        if(currentAnalogIndex>=numSensors) currentAnalogIndex=0;
      }
    }
      
      

      ADMUX &= B11110000;
      ADMUX |= analogInputs[currentAnalogIndex];
      */
      
      //Serial.println( analogVal );
      //delay(1000);
      //if( analogVal > 0 ) hitDetected(analogHitIndex);
    
      readFlag = 0;
    }
    /*
    if( samplesComplete ){
      samplesComplete = 0;
      oled->clearScreen(Colour::Black);
	
      for(uint8_t i=0;i<numSamples;i++){
        int height = 64;
        int halfHeight = 32;
        oled->drawPixel( i, CLAMP( sensorValues[0][i], -halfHeight, halfHeight ) + halfHeight, Colour::White );
      }
      
    }
    */

}

/*
void hitDetected(byte index){
  
  sensorTimes[index] = micros();
  //sensorValues[index][] = analogVal;
  sensorFlags |= 1<<index;
  uint8_t i;
  uint8_t lowestIndex = 0;

  
  Serial.print("hit detected: ");
  Serial.print(index);
  Serial.print(" ");
  Serial.println( analogVal );
  
  
  for( i=1;i<numSensors;i++){
    if( sensorTimes[lowestIndex] > sensorTimes[i] ){
        lowestIndex = i;
    }
  }
  
  if( sensorMask == sensorFlags ){
      Serial.print("hit: ");

       Serial.print( sensorValues[lowestIndex] );
       Serial.print(" ");
      for( i=0;i<numSensors;i++){
        if( i==lowestIndex ) continue;
        Serial.print( sensorTimes[i] - sensorTimes[lowestIndex] );
        Serial.print(" ");
        Serial.print( sensorValues[i] );
        Serial.print(" ");
      }
      Serial.println(analogVal);
      
      sensorFlags = 0;
  }
  

}
*/

ISR(ADC_vect){
  if( readingAnalog ) return;
  readingAnalog = true;
  
  /*
  if( currentAnalogIndex == 0 ){
    unsigned long currentTime = micros();
    if( currentTime < lastSample ) lastSample = currentTime;
    if( currentTime - lastSample < sampleInterval ){
        return;
    }
  
  }
  */
    
  // Must read low first
  analogVal = ADCL | (ADCH << 8);

  // Done reading
  readFlag = 1;
  
  // Not needed because free-running mode is enabled.
  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion
  // ADCSRA |= B01000000;
  int oldVal = sensorValues[currentAnalogIndex][currentSampleIndex];
  sensorValues[currentAnalogIndex][currentSampleIndex] = analogVal;
  byte oldAnalogIndex = currentAnalogIndex;
  currentAnalogIndex++;
  if(currentAnalogIndex==numSensors) currentAnalogIndex=0;
  
  /* switch to pin 6 */
  ADMUX = ( ADMUX & B11110000 ) | 6;   
  delayMicroseconds(50000);
  ADMUX = ( ADMUX & B11110000 ) | analogInputs[currentAnalogIndex];   

  //if we've gone back to the first sensor, increment the sample counter
  
  int yOffset = yHeight * oldAnalogIndex;
  if( oldVal != analogVal ){
    oled->drawPixel( currentSampleIndex, yOffset + CLAMP( oldVal * yScale, -yHalfHeight, yHalfHeight ) + yHalfHeight, Colour::Black );
    oled->drawPixel( currentSampleIndex, yOffset + CLAMP( analogVal * yScale, -yHalfHeight, yHalfHeight ) + yHalfHeight, Colour::White );
  }
  
  if(currentAnalogIndex==0){
    currentSampleIndex++;
    /*
    //turn off internupt; 
    ADCSRA &= ~B00001000;
    delayMicroseconds(100);
    //turn on internupt; 
    ADCSRA |= B00001000;
    */
    if( currentSampleIndex==numSamples ){
      currentSampleIndex = 0;
      samplesComplete = 1; 
    }
  }
  
  readingAnalog = false;
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
