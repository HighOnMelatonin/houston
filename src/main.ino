// Main program
#include <FastLED.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Pin definitions
static const uint8_t PIN_MP3_RX = 18
static const uint8_t PIN_MP3_TX = 19
#define buttonPin 20 // Pin for the button
#define LED_PIN1 21 // Pin for the LED
#define LED_PIN2 22 // Pin for the LED
#define LED_PIN3 23 // Pin for the LED
#define LED_PIN4 24 // Pin for the LED
#define LED_PIN5 25 // Pin for the LED
#define LED_PIN6 26 // Pin for the LED
#define LED_PIN7 27 // Pin for the LED
#define LED_PIN8 28 // Pin for the LED
#define stepPin 3 // Pin for the stepper motor direction
#define dirPin 2 // Pin for the stepper motor step

// Constants for the DFPlayer Mini
#define moveTime 5000  // Duration of the audio
SoftwareSerial mySoftwareSerial(PIN_MP3_RX, PIN_MP3_TX); // RX, TX
// Create the DFPlayerMini object
DFRobotDFPlayerMini player;

// Constants for LED
#define NUM_LEDS 18    // 18 LEDs per pin
#define num_strips 8    // 8 pins of LEDs
#define GRB CRGB(10,0,0)    // color of the LED
#define HSV CHSV(43.64, 99.18, 47.84)
#define START 278

CRGB leds[NUM_LEDS];

// Constants for the stepper motor
#define stepsPerRevolution 200
#define revolutions 10

// Constants for the RGB LED
#define RGB_BRIGHTNESS 1
#define RGB_BUILTIN 2

#define BLINK_SPD

/*
Sequence of events
1. Button pressed
2. Send signal to LED strip to light up
3. Send signal to stepper motor to move
4. Send signal to DFPlayer Mini to play music
5. Stop music (music will play for as long as the stepper motor is moving)
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
    pinMode(buttonPin, INPUT);

    // Init USB serial port for debugging
    Serial.begin(115200);
    // Init serial port for DFPlayer Mini
    mySoftwareSerial.begin(9600);
    if (!player.begin(mySoftwareSerial)) {
        Serial.println("Unable to begin");
        return;
    }
    else {
        Serial.println("DFPlayer Mini online");
    }

    FastLED.addLeds<WS2812, LED_PIN1, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN2, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN3, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN4, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN5, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN6, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN7, GRB>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN8, GRB>(leds, NUM_LEDS);

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
    // Read input from button
    int buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
        // Send signal to LED strip to light up
        lightUpLED();
        // Send signal to stepper motor to move
        moveStepperMotor();
        // Send signal to DFPlayer Mini to play music
        playMusic();
        // Stop moving
        stopMoving();
        // Stop light
        stopLight();
        // Rocket return to start
        returnToStart();
    }
}

void lightUpLED() {
    Serial.println("Lighting up LED");
    for (int i = 0; i < num_strips; i++) {
        for (int j = 0; j < NUM_LEDS; j++) {
            leds[j] = HSV;
            FastLED.show();
        }
    }
}

void moveStepperMotor() {
    // Set the direction counterclockwise
    digitalWrite(dirPin, LOW);
    // Step the motor
    for (int i = 0; i < stepsPerRevolution * revolutions; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
        // Light up LED while moving
        lightUpLED();
    }
}

void playMusic() {
    Serial.println("Playing music");
    mySoftwareSerial.begin(9600);

    player.volume(20);
    player.play(1);
    delay(moveTime);
}
