// Main program
#include <FastLED.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Pin definitions
#define DATA_PIN 19 // Pin for the LED strip
#define stepPin 3 // Pin for the stepper motor direction
#define dirPin 4 // Pin for the stepper motor step
static const uint8_t PIN_MP3_TX = 26
static const uint8_t PIN_MP3_RX = 27

// Constants for the DFPlayer Mini
SoftwareSerial mySoftwareSerial(PIN_MP3_RX, PIN_MP3_TX); // RX, TX

// Create the DFPlayerMini object
DFRobotDFPlayerMini player;
w
// Constants for LED
#define NUM_LEDS 300
#define GRB CRGB(10,0,0)    // color of the LED
#define HSV CHSV(43.64, 99.18, 47.84)
#define START 278

CRGB leds[NUM_LEDS];

// Constants for the stepper motor
#define stepsPerRevolution 200

// Constants for the RGB LED
#define RGB_BRIGHTNESS 1
#define RGB_BUILTIN 2

#define BLINK_SPD

/*
Sequence of events
1. Button pressed
2. Send signal to stepper motor to move
3. Send signal to LED strip to light up
4. Send signal to DFPlayer Mini to play music
5. Stop music
6. Keep light on
7. Keep moving
8. Stop moving
9. Stop light
10. Rocket return to start
*/

void setup() {
    // Declare pins as output
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(RGB_BUILTIN, OUTPUT);
    Serial.begin(9600);

    rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    delay(BLINK_SPD);
    rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
    delay(BLINK_SPD);
    rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
    delay(BLINK_SPD);
    rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    delay(BLINK_SPD);
    rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
    
}

void loop(){

}
