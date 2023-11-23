/* MTRAS SUMO Code
Needs work

*/

#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "pitches.h"


// Display
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 0         // Reset pin
#define SCREEN_ADDRESS 0x3C  // 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

// Built in LED
int LED_BUILTIN = 2;  // Pin the on board LED is connected to

//Piezo
int piezoPin = 4;  // Pin the piezo is connected to

// TOF Sensors
VL53L0X sensor;
VL53L0X sensor2;
int fr;  //Front Right Sensor
int fl;  //Front Left Sensor


// Floor Sensors
const int ffl = 13;  //Front Left Floor pin
const int ffr = 34;  //Front Right Floor pin
int potValueFL = 0;  //Variable to store floor sensor output
int potValueFR = 0;  //Variable to store floor sensor output




//Motor Controller Setup
#include <ESP32MX1508.h>

#define PINA 17  //Right motor Pin
#define PINB 16  //Right motor Pin

#define PINC 18  //Left motor Pin
#define PIND 19  //Left motor Pin


#define CH1 9   // 16 Channels (0-15) are availible
#define CH2 10  // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)

#define CH3 11  // 16 Channels (0-15) are availible
#define CH4 12  // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)

// Optional Parameters
#define RES 8      // Resolution in bits:  8 (0-255),  12 (0-4095), or 16 (0-65535)
#define FREQ 5000  // PWM Frequency in Hz


MX1508 motorR(PINA, PINB, CH1, CH2, RES, FREQ);  // Specify resolution and frequency

MX1508 motorL(PINC, PIND, CH3, CH4, RES, FREQ);  // Specify resolution and frequency


//Motor Speeds
int rspeed = 90;
int lspeed = 90;
int rspeedf = 90;
int lspeedf = 90;


// Button
const int buttonSelect = 35;  //Pushbutton pin

int buttonSelectState = 0;  // Variable for reading the pushbutton status

// Notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(buttonSelect, INPUT);

  //TOF Sensor Addressing

  pinMode(32, OUTPUT);

  pinMode(33, OUTPUT);


  digitalWrite(33, LOW);

  digitalWrite(32, LOW);


  delay(500);
  Wire.begin();


  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }




  Serial.begin(115200);

  display.setRotation(2);  //rotates text on OLED 1=90 degrees, 2=180 degrees


  const unsigned char MTRAS_SUMO[] PROGMEM =  // Logo on OLED

    {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x07, 0xf1, 0xfc, 0x20, 0xbf, 0xdf, 0xf0, 0xfe, 0x30, 0x00, 0xff, 0x61, 0x9f, 0x98, 0x30, 0x00,
      0x07, 0xf3, 0xfe, 0x71, 0xbf, 0xdf, 0xf1, 0xfe, 0x30, 0x00, 0xff, 0x61, 0x9f, 0xf8, 0x30, 0x00,
      0x0f, 0xf7, 0xfe, 0x71, 0xbf, 0xdf, 0xfb, 0xff, 0x70, 0x01, 0xff, 0xe3, 0x9f, 0xb8, 0x70, 0x00,
      0x0f, 0xe7, 0xfe, 0x71, 0xbf, 0x9f, 0xf3, 0xff, 0x70, 0x01, 0xfe, 0xe3, 0xbf, 0xbc, 0x70, 0x00,
      0x0c, 0x06, 0x06, 0x79, 0xb6, 0x18, 0x37, 0x03, 0x60, 0x01, 0xb0, 0xc3, 0xb0, 0x3c, 0xf0, 0x00,
      0x1c, 0x0e, 0x07, 0x79, 0xae, 0x38, 0x36, 0x03, 0x60, 0x00, 0x30, 0xc3, 0x30, 0x3c, 0xf0, 0x00,
      0x18, 0x0c, 0x06, 0xfb, 0x8e, 0x3b, 0xf6, 0x03, 0x60, 0x00, 0x30, 0xc3, 0x70, 0x3d, 0xf0, 0x00,
      0x18, 0x0c, 0x06, 0xfb, 0x8c, 0x37, 0xf6, 0x03, 0x60, 0x00, 0x30, 0xc3, 0x60, 0x3f, 0xf0, 0x00,
      0x38, 0x0c, 0x06, 0xdf, 0x0c, 0x37, 0xe6, 0x03, 0x60, 0x00, 0x30, 0xc7, 0x60, 0x7f, 0xe0, 0x00,
      0x30, 0x0c, 0x06, 0xdf, 0x0c, 0x37, 0xee, 0x03, 0xe0, 0x00, 0x71, 0xff, 0xff, 0x7f, 0x60, 0x00,
      0x30, 0x1c, 0x06, 0xdf, 0x0c, 0x37, 0x0e, 0x07, 0xe0, 0x00, 0x71, 0xff, 0xff, 0x77, 0x60, 0x00,
      0x38, 0x1c, 0x0e, 0xcf, 0x0c, 0x37, 0x0e, 0x07, 0xf0, 0x00, 0x71, 0xff, 0xff, 0x76, 0x60, 0x00,
      0x3c, 0x1c, 0x0e, 0xcf, 0x1c, 0x73, 0x8e, 0x06, 0xf0, 0x00, 0x71, 0xc6, 0xf0, 0x70, 0x60, 0x00,
      0x3c, 0x1c, 0x0f, 0xcf, 0x1c, 0x73, 0x8e, 0x06, 0xf0, 0x00, 0x71, 0xc6, 0xf0, 0x70, 0xe0, 0x00,
      0x3c, 0x1c, 0x0d, 0xc7, 0x1c, 0x73, 0x8e, 0x06, 0xf0, 0x00, 0x71, 0xc6, 0xf0, 0x70, 0xe0, 0x00,
      0x38, 0x1c, 0x0d, 0xc6, 0x1c, 0x71, 0x8e, 0x0e, 0xf0, 0x00, 0x71, 0xc6, 0xf0, 0xf0, 0xc0, 0x00,
      0x38, 0x1c, 0x1d, 0xc6, 0x1c, 0x71, 0xce, 0x0e, 0xf0, 0x00, 0xf3, 0xce, 0xe0, 0xe0, 0xcc, 0xee,
      0x3f, 0x9f, 0xf9, 0xc6, 0x1c, 0x71, 0xcf, 0xfc, 0x7f, 0x00, 0xe3, 0x8e, 0xfe, 0xe0, 0xde, 0xee,
      0x3f, 0xdf, 0xf9, 0xc6, 0x1c, 0x71, 0xcf, 0xfc, 0x7f, 0x00, 0xe3, 0x8e, 0xfe, 0xe0, 0xdc, 0xee,
      0x3f, 0xcf, 0xf1, 0xc6, 0x1c, 0xf0, 0xcf, 0xf8, 0x7f, 0x00, 0xe3, 0x8c, 0xfe, 0xe0, 0xdc, 0xce,
      0x3f, 0x8f, 0xe1, 0x84, 0x18, 0x60, 0xc7, 0xf0, 0x7f, 0x00, 0xe3, 0x8c, 0x7e, 0xe0, 0x80, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

  const unsigned char Version[] PROGMEM =  // Logo on OLED

    {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x00,
      0x00, 0x00, 0xfe, 0x1f, 0xcf, 0xe3, 0xf8, 0xff, 0x8f, 0xc0, 0xff, 0x61, 0x9f, 0xb0, 0x60, 0x00,
      0x00, 0x01, 0xff, 0x1f, 0xcf, 0xe7, 0xfc, 0xff, 0x9f, 0xc1, 0xff, 0xe3, 0x9f, 0xf0, 0x60, 0x00,
      0x00, 0x01, 0xff, 0x3f, 0x8f, 0xcf, 0xfc, 0xff, 0x9f, 0xc1, 0xff, 0xe3, 0xbf, 0xb8, 0xe0, 0x00,
      0x00, 0x00, 0x07, 0x3f, 0x1c, 0x0e, 0x1d, 0xc1, 0x9f, 0x81, 0xb8, 0xc3, 0xbf, 0x39, 0xc0, 0x00,
      0x00, 0x01, 0x86, 0x30, 0x18, 0x1c, 0x0d, 0xc1, 0xb8, 0x01, 0xb0, 0xc3, 0x30, 0x39, 0xc0, 0x00,
      0x00, 0x01, 0x8e, 0x70, 0x18, 0x18, 0x0d, 0x81, 0xb0, 0x00, 0x30, 0xc3, 0x70, 0x1b, 0x80, 0x00,
      0x00, 0x01, 0x9e, 0x70, 0x38, 0x18, 0x0d, 0x9f, 0xb0, 0x00, 0x30, 0xc3, 0x70, 0x1f, 0x80, 0x00,
      0x00, 0x03, 0xbc, 0x60, 0x30, 0x18, 0x0d, 0xbf, 0x70, 0x00, 0x30, 0xc3, 0x60, 0x1f, 0x00, 0x00,
      0x00, 0x03, 0xfc, 0xff, 0x3f, 0xb8, 0x0d, 0xbf, 0x7f, 0x00, 0x71, 0xff, 0xff, 0x1f, 0x00, 0x00,
      0x00, 0x03, 0x7e, 0xff, 0x7f, 0xf8, 0x1d, 0xbe, 0x7f, 0x80, 0x71, 0xff, 0xff, 0x0e, 0x00, 0x00,
      0x00, 0x03, 0x7e, 0xff, 0x7f, 0xb0, 0x1f, 0xb8, 0x7f, 0x80, 0x61, 0xff, 0xff, 0x0e, 0x00, 0x00,
      0x00, 0x03, 0x7f, 0xff, 0x7f, 0xb8, 0x1b, 0xb8, 0x7f, 0x00, 0x71, 0xfe, 0xff, 0x0e, 0x00, 0x00,
      0x00, 0x03, 0x86, 0xf0, 0x70, 0x38, 0x1b, 0x98, 0x70, 0x00, 0x71, 0xc6, 0xf0, 0x0e, 0x00, 0x00,
      0x00, 0x03, 0x86, 0xf0, 0x70, 0x38, 0x1b, 0x9c, 0x70, 0x00, 0x71, 0xc6, 0xf0, 0x0e, 0x00, 0x00,
      0x00, 0x07, 0x86, 0xf0, 0x70, 0x38, 0x1b, 0x9c, 0x70, 0x00, 0x71, 0xc6, 0xf0, 0x1e, 0x00, 0x00,
      0x00, 0x07, 0x06, 0xf0, 0xf0, 0x38, 0x3b, 0x9c, 0x70, 0x00, 0xf3, 0xce, 0xe0, 0x1c, 0x00, 0x00,
      0x00, 0x07, 0x06, 0xe0, 0xe0, 0x38, 0x73, 0x8e, 0x70, 0x00, 0xe3, 0x8e, 0xe0, 0x1c, 0x00, 0x00,
      0x00, 0x07, 0xfe, 0xfe, 0xe0, 0x3f, 0xf7, 0x8e, 0x7f, 0x00, 0xe3, 0x8e, 0xfe, 0x1c, 0x00, 0x00,
      0x00, 0x07, 0xfc, 0xfe, 0xe0, 0x3f, 0xe7, 0x0e, 0x7f, 0x00, 0xe3, 0x8c, 0xfe, 0x1c, 0x00, 0x00,
      0x00, 0x07, 0xfc, 0xfe, 0xe0, 0x1f, 0xc7, 0x06, 0x7f, 0x00, 0xe3, 0x8c, 0xfe, 0x1c, 0x00, 0x00,
      0x00, 0x07, 0xf8, 0x7c, 0x40, 0x0f, 0x82, 0x04, 0x3e, 0x00, 0x41, 0x00, 0x7c, 0x18, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

  const unsigned char ControlThem[] PROGMEM =  // Logo on OLED

    {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0xfe, 0x3f, 0x0c, 0x37, 0xfb, 0xfc, 0x3f, 0x8c, 0x00, 0x20, 0xc7, 0xe1, 0x82, 0x63, 0x30,
      0x00, 0xfe, 0x7f, 0x8c, 0x37, 0xff, 0xfe, 0x7f, 0x8c, 0x00, 0x70, 0xcf, 0xf1, 0x83, 0x77, 0x70,
      0x01, 0xfe, 0xff, 0xcc, 0x3f, 0xff, 0xfe, 0x7f, 0xcc, 0x00, 0x71, 0xdf, 0xf9, 0x87, 0x77, 0x70,
      0x01, 0xfc, 0xff, 0xce, 0x7f, 0xf7, 0xfe, 0xff, 0xdc, 0x00, 0x71, 0xdf, 0xf9, 0x86, 0xf7, 0x70,
      0x01, 0x80, 0xc1, 0xde, 0x7d, 0x86, 0x0e, 0xc0, 0xdc, 0x00, 0x33, 0x98, 0x3b, 0x86, 0xe7, 0x70,
      0x03, 0x81, 0xc0, 0xde, 0x6d, 0x86, 0x0d, 0xc0, 0xd8, 0x00, 0x3b, 0xb8, 0x1b, 0x06, 0xef, 0x70,
      0x03, 0x01, 0x80, 0xdf, 0x61, 0x86, 0x7d, 0x80, 0xd8, 0x00, 0x3f, 0x30, 0x1b, 0x06, 0xee, 0x70,
      0x03, 0x01, 0x80, 0xdf, 0x61, 0x86, 0xfd, 0x80, 0xd8, 0x00, 0x3f, 0x30, 0x1b, 0x06, 0xee, 0xe0,
      0x07, 0x01, 0x80, 0xdf, 0x61, 0x8f, 0xfd, 0x80, 0xd8, 0x00, 0x1e, 0x30, 0x1b, 0x0e, 0xee, 0xe0,
      0x06, 0x01, 0x80, 0xdb, 0xe3, 0x8f, 0xf9, 0x80, 0xd8, 0x00, 0x1c, 0x30, 0x1b, 0x0e, 0xcc, 0xe0,
      0x06, 0x03, 0x81, 0xdb, 0xe3, 0x8c, 0xc1, 0x80, 0xf8, 0x00, 0x1c, 0x70, 0x3b, 0x0d, 0xcc, 0xc0,
      0x07, 0x03, 0x81, 0xfb, 0xe3, 0x0c, 0xe3, 0x81, 0xfc, 0x00, 0x1c, 0x70, 0x3f, 0x0d, 0xcc, 0xc0,
      0x07, 0x03, 0x81, 0xb9, 0xc3, 0x8e, 0xe3, 0x81, 0xfc, 0x00, 0x1c, 0x70, 0x37, 0x0d, 0xdc, 0xc0,
      0x07, 0x83, 0x81, 0xb9, 0xc3, 0x8e, 0xe3, 0x81, 0xbc, 0x00, 0x1c, 0x70, 0x37, 0x0d, 0x9c, 0xc0,
      0x07, 0x03, 0x81, 0xb9, 0xc3, 0x8e, 0x63, 0x81, 0xbc, 0x00, 0x1c, 0x70, 0x37, 0x0d, 0x88, 0x80,
      0x07, 0x03, 0x83, 0xb9, 0xc3, 0x9e, 0x73, 0x81, 0x9c, 0x00, 0x1c, 0x70, 0x77, 0x1c, 0x00, 0x00,
      0x07, 0x03, 0x83, 0xb8, 0xc7, 0x9c, 0x73, 0x83, 0x9c, 0x00, 0x3c, 0x70, 0x77, 0x1d, 0x98, 0x80,
      0x07, 0xf3, 0xff, 0x39, 0xc7, 0x1c, 0x73, 0xff, 0x1f, 0xc0, 0x38, 0x7f, 0xe7, 0xfb, 0x99, 0xc0,
      0x07, 0xfb, 0xff, 0x71, 0xc7, 0x1c, 0x33, 0xff, 0x1f, 0xc0, 0x38, 0x7f, 0xe7, 0xfb, 0xb9, 0xc0,
      0x07, 0xf9, 0xfe, 0x71, 0x87, 0x1c, 0x31, 0xfe, 0x1f, 0xc0, 0x38, 0x3f, 0xc7, 0xf3, 0xbb, 0x80,
      0x07, 0xf1, 0xfc, 0x71, 0x87, 0x18, 0x31, 0xfc, 0x1f, 0xc0, 0x38, 0x3f, 0x83, 0xe3, 0xb9, 0x80,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

  const unsigned char Before_They[] PROGMEM =  // Logo on OLED

    {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x0c, 0x06, 0xff, 0xbf, 0xf0, 0xfc, 0x3f, 0xc0, 0x3f, 0xec, 0x0d, 0x80, 0xc3, 0xf8, 0x00,
      0x00, 0x0c, 0x0e, 0xff, 0xbf, 0xf1, 0xfc, 0x7f, 0xe0, 0x7f, 0xec, 0x0d, 0xc1, 0xc7, 0xfc, 0x00,
      0x00, 0x0e, 0x0f, 0xff, 0xbf, 0xf3, 0xfe, 0x7f, 0xe0, 0x7f, 0xec, 0x1d, 0xc1, 0xcf, 0xfc, 0x00,
      0x00, 0x1e, 0x1f, 0xff, 0x7f, 0xf3, 0xfe, 0xff, 0xc0, 0xff, 0xcc, 0x1d, 0xc3, 0xdf, 0xfe, 0x00,
      0x00, 0x1e, 0x3d, 0xd8, 0x70, 0x37, 0x8e, 0xe0, 0x00, 0xe0, 0x1c, 0x1f, 0xc3, 0xdc, 0x0e, 0x00,
      0x00, 0x1f, 0x3d, 0xb8, 0x70, 0x77, 0x0e, 0xe0, 0x00, 0xe0, 0x1c, 0x1b, 0xe7, 0x98, 0x0e, 0x00,
      0x00, 0x1f, 0x7c, 0x38, 0x70, 0x76, 0x0e, 0xe0, 0x00, 0xe0, 0x1c, 0x1b, 0xef, 0xb8, 0x0e, 0x00,
      0x00, 0x1f, 0x7c, 0x38, 0x67, 0xf6, 0x0e, 0xf0, 0x00, 0xf0, 0x18, 0x1b, 0xef, 0xb8, 0x0e, 0x00,
      0x00, 0x1f, 0xfc, 0x38, 0x6f, 0xe6, 0x0e, 0xf0, 0x00, 0x78, 0x18, 0x3b, 0xff, 0xb8, 0x0e, 0x00,
      0x00, 0x1b, 0xfc, 0x30, 0x6f, 0xee, 0x7e, 0x78, 0x00, 0x78, 0x18, 0x3b, 0x7f, 0xb0, 0x0e, 0x00,
      0x00, 0x3b, 0xdc, 0x30, 0xef, 0xce, 0xfc, 0x3c, 0x00, 0x3c, 0x18, 0x3b, 0x7b, 0xb0, 0x0e, 0x00,
      0x00, 0x3b, 0xd8, 0x30, 0xee, 0x0f, 0xfc, 0x3c, 0x00, 0x3c, 0x38, 0x3f, 0x7b, 0xb0, 0x0c, 0x00,
      0x00, 0x3b, 0x98, 0x70, 0xe7, 0x0f, 0xfc, 0x1e, 0x00, 0x1e, 0x38, 0x37, 0x33, 0x70, 0x0c, 0x00,
      0x00, 0x39, 0x18, 0x70, 0xe7, 0x0e, 0xfc, 0x1e, 0x00, 0x0f, 0x38, 0x37, 0x23, 0x70, 0x0c, 0x00,
      0x00, 0x38, 0x38, 0x78, 0xe7, 0x0e, 0x1c, 0x0f, 0x00, 0x0f, 0x38, 0x37, 0x03, 0x78, 0x1c, 0x00,
      0x00, 0x38, 0x38, 0x70, 0xe7, 0x0e, 0x1c, 0x07, 0x00, 0x07, 0xb8, 0x77, 0x07, 0x70, 0x1c, 0x00,
      0x00, 0x38, 0x38, 0x70, 0xe3, 0x9e, 0x1c, 0x07, 0x80, 0x07, 0xb8, 0x77, 0x07, 0x70, 0x1c, 0x00,
      0x00, 0x78, 0x38, 0x71, 0xe3, 0x9e, 0x19, 0xe3, 0x01, 0xe3, 0xb8, 0x77, 0x07, 0x70, 0x18, 0x00,
      0x00, 0x78, 0x30, 0x71, 0xe3, 0x9e, 0x1b, 0xf3, 0x03, 0xf3, 0x38, 0xef, 0x07, 0x70, 0x38, 0x00,
      0x00, 0x78, 0x30, 0xf1, 0xe3, 0xdc, 0x1b, 0xff, 0x03, 0xff, 0x3f, 0xef, 0x06, 0x7f, 0xf8, 0x00,
      0x00, 0x70, 0x30, 0xf1, 0xc1, 0xdc, 0x3b, 0xff, 0x03, 0xff, 0x3f, 0xef, 0x06, 0x7f, 0xf0, 0x00,
      0x00, 0x70, 0x70, 0xf1, 0xc1, 0xdc, 0x3b, 0xfe, 0x03, 0xfe, 0x3f, 0xce, 0x06, 0x3f, 0xe0, 0x00,
      0x00, 0x70, 0x20, 0x61, 0xc1, 0x98, 0x33, 0xfe, 0x03, 0xfe, 0x1f, 0x8e, 0x06, 0x1f, 0xc0, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

  const unsigned char Control_You[] PROGMEM =  // Logo on OLED

    {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x13, 0xcf, 0xc7, 0xc0, 0x7c, 0x42, 0x0c, 0x03, 0xc0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x06, 0x13, 0xef, 0xef, 0xe8, 0xfe, 0x42, 0x0c, 0x07, 0xe0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x04, 0x37, 0xcf, 0xef, 0xc8, 0xfe, 0x66, 0x0c, 0x0f, 0xf0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x04, 0x36, 0x18, 0x68, 0x09, 0x86, 0x66, 0x04, 0x08, 0x70, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x04, 0x24, 0x18, 0x68, 0x09, 0x02, 0xe6, 0x04, 0x18, 0x70, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x04, 0x2c, 0x1b, 0xcc, 0x19, 0x02, 0xf4, 0x0c, 0x10, 0xf0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x0c, 0x2c, 0x13, 0xcc, 0x1b, 0x02, 0xf4, 0x0c, 0x11, 0xb0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x0c, 0x2f, 0x97, 0xc6, 0x1b, 0x06, 0xbc, 0x0c, 0x11, 0xa0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x0c, 0x6f, 0xd2, 0x07, 0x12, 0x06, 0x9c, 0x08, 0x13, 0x20, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x0e, 0x5f, 0x93, 0x03, 0x13, 0x06, 0x9c, 0x0c, 0x16, 0x20, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x06, 0xcc, 0x33, 0x03, 0x9b, 0x04, 0x9c, 0x0c, 0x3e, 0x20, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x06, 0x8c, 0x33, 0x01, 0x9b, 0x05, 0x8c, 0x0c, 0x3c, 0x60, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x07, 0x8c, 0x31, 0x00, 0xb3, 0x05, 0x88, 0x0c, 0x38, 0x60, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x07, 0x0c, 0x31, 0xbc, 0xb3, 0x0d, 0x88, 0x19, 0xb8, 0xc0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x03, 0x0f, 0xb1, 0xbf, 0xb3, 0xf9, 0x88, 0x19, 0xbf, 0xc0, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x02, 0x0f, 0xb1, 0xbf, 0xb3, 0xf9, 0x88, 0x19, 0x1f, 0x80, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x02, 0x0f, 0x20, 0x3f, 0x21, 0xe1, 0x00, 0x18, 0x0f, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

  //The following displays the start up text upon power up.
  display.drawBitmap(0, 0, MTRAS_SUMO, 128, 64, WHITE);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.drawBitmap(0, 0, Version, 128, 64, WHITE);
  display.display();
  delay(750);
  display.clearDisplay();
  display.drawBitmap(0, 0, ControlThem, 128, 64, WHITE);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.drawBitmap(0, 0, Before_They, 128, 64, WHITE);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.drawBitmap(0, 0, Control_You, 128, 64, WHITE);
  display.display();
  delay(1000);
  display.clearDisplay();

  // Starts up TOF Sensors

  digitalWrite(33, HIGH);  //A
  delay(150);
  sensor.init(true);
  Serial.println("03");
  delay(100);
  sensor.setAddress((uint8_t)02);
  Serial.println("04");


  digitalWrite(32, HIGH);  //B
  delay(150);
  Serial.println("07");
  sensor2.init(true);

  Serial.println("08");
  delay(100);
  sensor2.setAddress((uint8_t)04);
  Serial.println("09");



  Serial.println("addresses set");
  // Start reading TOF Sensors continuously
  sensor.startContinuous();
  sensor2.startContinuous();

  // Go to portion of code to wait on a button press
  selectButton();
}


void loop() {
  // read the state of the pushbutton value:
  int buttonSelectState = HIGH;

  while (buttonSelectState == HIGH) { //While the button is not pressed do the following
    buttonSelectState = digitalRead(buttonSelect);


    readsensors();
    delay(20);

    if ((fl < 600 && fl > 20) && (fr < 600 && fr > 20)) {
      forward();
    }

    if ((fl < 600 && fr < 600)) {
      forward();
    }

    else {
      if (fl < 600 && fl > 20) {
        right();
        //forward();
      }
      if (fr < 600 && fr > 20) {
        left();
        //forward();
      }
    }
    if (potValueFL < 1500) {
      reverseR();
    }

    if (potValueFR < 1500) {
      reverseL();
    }
    readsensors();
  }
  //Button was pressed so top motors and go back to waiting for a button press
  motorR.motorStop();
  motorL.motorStop();

  selectButton();
  
}



void forward() {
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("FORWARD"));
  display.display();
  motorR.motorGo(160);  //Forward
  motorL.motorGo(160);
  delay(25);
  //readsensors();
}

void reverseR() {
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("REVERSE"));
  display.display();
  motorR.motorRev(135);  // Reverse
  motorL.motorRev(135);
  delay(700);
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("RIGHT"));
  display.display();
  motorR.motorRev(135);  //Right Left
  motorL.motorGo(135);
  delay(50);
  // readsensors();
}

void reverseL() {
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("REVERSE"));
  display.display();
  motorR.motorRev(135);  // Reverse
  motorL.motorRev(135);
  delay(700);
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("LEFT"));
  display.display();
  motorR.motorGo(135);  //Right Turn
  motorL.motorRev(135);
  delay(50);
  //readsensors();
}

void left() {
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("LEFT"));
  display.display();
  motorR.motorGo(135);  //Right Turn
  motorL.motorRev(135);
  delay(50);
  // readsensors();
}

void right() {
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("RIGHT"));
  display.display();
  motorR.motorRev(135);  //Right Left
  motorL.motorGo(135);
  delay(50);
  // readsensors();
}


void readsensors() {
  //TOF Sensors
  Serial.print(" TOF Front Right: ");
  fr = sensor.readRangeContinuousMillimeters();
  Serial.print(fr);
  delay(10);


  Serial.print("  TOF Front Left: ");
  fl = sensor2.readRangeContinuousMillimeters();
  Serial.print(fl);
  delay(10);


  // Floor Sensors
  Serial.print(" Line Sensor Left: ");
  potValueFL = analogRead(ffl);
  Serial.print(potValueFL);

  Serial.print(" Line Sensor Right: ");
  potValueFR = analogRead(ffr);
  Serial.print(potValueFR);

  Serial.println("  ");
  //delay (200);
}
void countdown() {
  digitalWrite(LED_BUILTIN, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(22, 10);            // Start at top-left corner
  display.println(F(" FIVE "));
  display.display();
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(22, 10);            // Start at top-left corner
  display.println(F(" FOUR "));
  display.display();
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(18, 10);            // Start at top-left corner
  display.println(F(" THREE "));
  display.display();
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(32, 10);            // Start at top-left corner
  display.println(F(" TWO "));
  display.display();
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH);
  tone(piezoPin, 1000, 1000);
  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(32, 10);            // Start at top-left corner
  display.println(F(" ONE "));
  display.display();
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  tone(piezoPin, 0, 500);
  delay(500);
  display.clearDisplay();

  display.clearDisplay();
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("FIGHT!"));
  display.println(F("ME!"));
  /*
  // This portion of code is if you have a drop down scoop. 
   motorR.motorRev(rspeed);  // Reverse
  motorL.motorRev(lspeed);
  delay(250);
  motorR.motorStop();  // Reverse
  motorL.motorStop();
  */

  display.display();
  readsensors();

  //tone(piezoPin, 1000, 500);
  //delay(1000);
  right();
}

void selectButton() {
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("    Push Select"));
  display.display();
  display.setCursor(0, 16);  // Start at top-left corner
  display.println(F("     to START"));
  display.display();
  delay(1000);
  int buttonSelectState = HIGH;
  // read the state of the pushbutton value:

  while (buttonSelectState == HIGH) {
    buttonSelectState = digitalRead(buttonSelect);
  }
  countdown();
}
