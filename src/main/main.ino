// Main program
#include <FastLED.h>

// Pin definitions
#define buttonPin 20 // Pin for the button
#define LED_PIN1 15 // Pin for the LED
#define LED_PIN2 2 // Pin for the LED
#define LED_PIN3 0 // Pin for the LED
#define LED_PIN4 4 // Pin for the LED
#define LED_PIN5 16 // Pin for the LED
#define LED_PIN6 17 // Pin for the LED
#define LED_PIN7 5 // Pin for the LED
#define LED_PIN8 18 // Pin for the LED
#define stepPin 3 // Pin for the stepper motor direction
#define dirPin 1 // Pin for the stepper motor step


// Constants for LED
#define NUM_LEDS 18    // 18 LEDs per pin
#define num_strips 8    // 8 pins of LEDs
#define GRB CRGB(10,0,0)    // color of the LED
#define HSV CHSV(43.64, 99.18, 47.84)
#define START 278

CRGB leds[NUM_LEDS];

// Constants for the stepper motor
#define stepsPerRevolution 200
#define revolutions 10  // Change this to change height of rocket

// Constants for the RGB LED
#define RGB_BRIGHTNESS 1
#define RGB_BUILTIN 2

#define BLINK_SPD 10

/*
Sequence of events
1. Button pressed
2. Send signal to LED strip to light up
3. Send signal to stepper motor to move
4. Keep light on
5. Keep moving
6. Stop moving
7. Stop light
8. Rocket return to start
*/

void setup() {
    // Declare pins as output
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(RGB_BUILTIN, OUTPUT);
    pinMode(buttonPin, INPUT);

    // Init USB serial port for debugging
    Serial.begin(115200);

    FastLED.addLeds<WS2812, LED_PIN1>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN2>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN3>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN4>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN5>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN6>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN7>(leds, NUM_LEDS);
    FastLED.addLeds<WS2812, LED_PIN8>(leds, NUM_LEDS);

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
        // Stop moving
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

void stopLight() {
    Serial.println("Stopping light");
    for (int i = 0; i < num_strips; i++) {
        for (int j = 0; j < NUM_LEDS; j++) {
            leds[j] = CRGB::Black;
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

void returnToStart() {
    // Set the direction clockwise
    digitalWrite(dirPin, HIGH);
    // Step the motor
    for (int i = 0; i < stepsPerRevolution * revolutions; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
    }
}
