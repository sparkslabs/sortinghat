#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000

#include "griffindor.h"
#include "hufflepuff.h"
// #include "ravenclaw.h"
// #include "slytherin.h"


// ###########################################################################################################################################################################################
// ##### AUDIO PLAYBACK (start) ##############################################################################################################################################################
// ###
// ###    Audio Samples will be sent to to the speaker Pin
// ###



int speakerPin = 11;
volatile uint16_t sample;
byte lastSample;

int sounddata_length = hufflepuff_length;
signed char *sounddata_data = hufflepuff_data;
int sampleToPlay = 1;


// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= sounddata_length) {
        if (sample == sounddata_length + lastSample) {
            stopPlayback();
        }
        else {
            // Ramp down to zero to reduce the click at the end of playback.
            OCR2A = sounddata_length + lastSample - sample;
        }
    }
    else {
        OCR2A = pgm_read_byte(&sounddata_data[sample]);
    }

    ++sample;
}

// ###
// ###
// ###
// ##### AUDIO PLAYBACK (end) ################################################################################################################################################################
// ###########################################################################################################################################################################################



void startPlayback()
{
    sounddata_length = hufflepuff_length;
    sounddata_data = hufflepuff_data;
    pinMode(speakerPin, OUTPUT);
    
    // Set up Timer 2 to do pulse width modulation on the speaker
    // pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

    // Do non-inverting PWM on pin OC2A (p.155)
    // On the Arduino this is pin 11.
    TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
    TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));

    // No prescaler (p.158)
    TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set initial pulse width to the first sample.
    OCR2A = pgm_read_byte(&sounddata_data[0]);

    // Set up Timer 1 to send a sample every interrupt.
    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]);
    sample = 0;
    sei();
}

void stopPlayback()
{
    // Disable playback per-sample interrupt.
    TIMSK1 &= ~_BV(OCIE1A);

    // Disable the per-sample timer completely.
    TCCR1B &= ~_BV(CS10);

    // Disable the PWM timer.
    TCCR2B &= ~_BV(CS10);

    digitalWrite(speakerPin, LOW);
    // Loop for test purposes
//    startPlayback();
}


int ledPin = 5;
int buttonPin = A5;
int lightSensorPin = A6;
int light_threshold = 5;

void setup()
{
  pinMode(ledPin, OUTPUT);  // set pin as output
  //Button
  pinMode(buttonPin, INPUT);  // set the pin as an input
  digitalWrite(buttonPin, HIGH);  // enable the pull-up resistor
//  startPlayback();
}

void loop()
{ 
  //Buzzer
  if(digitalRead(buttonPin) == 0)
  {  // if you press the button, make a short buzz
    light_threshold = analogRead(lightSensorPin);
    startPlayback();
//    digitalWrite(ledPin, HIGH);
  }
  else  // If the button is not pressed go in here
  {
     digitalWrite(ledPin, LOW);
    ///LIGHT TEST
    if(analogRead(lightSensorPin) > light_threshold){
      //if its dark, turn on all white leds...
      digitalWrite(ledPin, HIGH);
    }
    else{
//      digitalWrite(ledPin, LOW);
    }
  }
}

