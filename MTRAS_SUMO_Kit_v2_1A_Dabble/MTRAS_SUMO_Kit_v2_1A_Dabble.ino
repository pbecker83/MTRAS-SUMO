/* MTRAS Kit v2.A SUMO Code 2/21/2024
More info @ MTRAS.COM/SUMO
*/

//All the libraries used.
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "pitches.h"
#include <Arduino.h>
#include <IRremoteESP8266.h>  //Future use.
#include <IRrecv.h>           //Future use.
#include <IRutils.h>          //Future use.
#include <ESP32MX1508.h>
#include "MTRAS_Boot_Screen.cpp"


// Display Setup
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 0         // Reset pin
#define SCREEN_ADDRESS 0x3C  // 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

/////////////////////////////////////////////////
//Robot adjustments.
char name[] = "Robot";  //Change "Robot name" text to your name etc.
int tofsens = 850;          // Change this value to increase or decrease the TOF Front Sensor object detection ranges.
int turnspeed = 65;//50          //Adjust for turning speed.
// Adjust for Reverse speed. Allows for adjustment of each motor. This makes it possible to adjust to how straight the robot backs up.
int rspeedRev = 135;//135
int lspeedRev = 135;//135
// Adjust for Forward speed. Allows for adjustment of each motor. This makes it possible to adjust to how straight the robot goes forward.
int rspeedFor = 75;   //Right motor speed
int irspeedFor = 75;  //Initial right motor speed
int lspeedFor = 75;   //Left motor speed
int ilspeedFor = 75;  //Initial left motor speed
///////////////////////////////////////////////////

// Built in LED
int ledPin = 2;  // Pin the on board LED is connected to
//Piezo
int piezoPin = 4;  // Pin the piezo is connected to

// TOF Sensors Ie: Front sensors.
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

//Right Motor
int SpeedupR;  //Right motor ramp up variable

//Left Motor
int SpeedupL;  //Left motor ramp up variable




// Start Button
const int buttonSelect = 35;  //Start button pin

int buttonSelectState = 0;  // Variable for reading the start button status

// Notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void setup() {
  Dabble.begin("Patricks SUMO");       //set bluetooth name of your robot
  pinMode(ledPin, OUTPUT);
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

  // Starts up TOF Sensors

  digitalWrite(32, HIGH);  //A
  delay(150);
  sensor.init(true);
  Serial.println("03");
  delay(100);
  sensor.setAddress((uint8_t)02);
  Serial.println("04");

  digitalWrite(33, HIGH);  //B
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


//Set default motor speed
irspeedFor = rspeedFor;
ilspeedFor = lspeedFor;

  //This portion of code allows you to see the sensor outputs via the Arduino "Serial Monitor"
  //Press the start button before power up and release when the display shows "Reading sensors"
  //To exit press the start button again.
  buttonSelectState = digitalRead(buttonSelect);
  if (buttonSelectState == LOW) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Reading sensors"));
    display.display();
    delay(3000);
    printsensors();
  }

  /*
  bootscreen();    // Display start screen
  selectButton();  //Go to wait for start button press
  */
   display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Bluetooth Control"));
    display.display();
    delay(1000);
}


void loop() {
  Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
  Serial.print("KeyPressed: ");
  if (GamePad.isUpPressed())
  {
    forwardBT();
  }
  if (GamePad.isDownPressed())
  {
    reverse();
  }
  if (GamePad.isLeftPressed())
  {
    leftBT();
  }
  if (GamePad.isRightPressed())
  {
   rightBT();
  }
   if (GamePad.isStartPressed())
  {
stop(); 
 }

}
  /*
  SpeedupR = irspeedFor;
  SpeedupL = ilspeedFor;
  readsensors();
  delay(20);
  if ((fl < tofsens && fl > 20) && (fr < tofsens && fr > 20)) {
    motorR.motorGo(irspeedFor);  //Forward
    motorL.motorGo(ilspeedFor);
    forward();
    SpeedupR = irspeedFor;
    SpeedupL = ilspeedFor;

  } else {
    if ((fl < tofsens && fl > 20)) {
      right();
    }
    if (fr < tofsens && fr > 20) {
      left();
    }
  }
  if ((potValueFL < 3000) && (potValueFL > 100)) {
    reverseR();
  }
  if ((potValueFR < 3000) && (potValueFR > 100)) {
    reverseL();
  }
  
}
*/

void forward() {
  irspeedFor = rspeedFor;
  ilspeedFor = lspeedFor;
  display.clearDisplay();
  display.println(F("FORWARD"));
  display.display();
  readsensors();
  //while ((fl < tofsens && fl > 20) && (fr < tofsens && fr > 20)) {
  while ((potValueFL > 2500) && (potValueFR > 2500)) {
    buttonSelectState = digitalRead(buttonSelect);

    if (SpeedupR > 250) {
      SpeedupR = (250);
    }
    if (SpeedupL > 250) {
      SpeedupL = (250);
    }
    if ((potValueFL < 2500) && (potValueFL > 100)) {
      reverseR();
    }
    if ((potValueFR < 2500) && (potValueFR > 100)) {
      reverseL();
    }
    SpeedupR = (irspeedFor++);  //Ramps up speed every loop if both front sensors see something
    SpeedupL = (ilspeedFor++);  //Ramps up speed every loop if both front sensors see something
    motorR.motorGo(SpeedupR);   //Forward
    motorL.motorGo(SpeedupL);
    delay(50);
    readsensors();
  }
}
void forwardBT() {  //Reverse and turn left
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("FORWARD"));
  display.display();
   motorR.motorGo(rspeedFor);   //Forward
    motorL.motorGo(lspeedFor);
  delay(10);
}

void reverse() {  //Reverse and turn left
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("REVERSE"));
  display.display();
  motorR.motorRev(rspeedRev);
  motorL.motorRev(lspeedRev);
  delay(10);
}

void stop(){
   display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("STOP"));
  display.display();
    motorR.motorStop();
    motorL.motorStop();
    delay(100);
    }

void reverseR() {  //Reverse and turn right
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("REVERSE"));
  display.display();
  motorR.motorRev(rspeedRev);
  motorL.motorRev(lspeedRev);
  delay(500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("RIGHT"));
  display.display();
  motorR.motorRev(turnspeed);  //Turn Right
  motorL.motorGo(turnspeed);
  //delay();
}

void reverseL() {  //Reverse and turn left
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("REVERSE"));
  display.display();
  motorR.motorRev(rspeedRev);
  motorL.motorRev(lspeedRev);
  delay(500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("LEFT"));
  display.display();
  motorR.motorGo(turnspeed);  //Turn Left
  motorL.motorRev(turnspeed);
  //delay();
}

void left() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("LEFT"));
  display.display();
  motorR.motorGo(turnspeed);  //Turn Left
  motorL.motorRev(turnspeed);
  readsensors();
  if (fr < tofsens && fr > 20) {
    //right();
    forward();
  }
  //delay();
}

void leftBT() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("LEFT"));
  display.display();
  motorR.motorGo(turnspeed);  //Turn Left
  motorL.motorRev(turnspeed);
  delay(10);
}

void right() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("RIGHT"));
  display.display();
  motorR.motorRev(turnspeed);  //Turn Right
  motorL.motorGo(turnspeed);
  readsensors();

  if ((fl < tofsens && fl > 20)) {
    // left();
    forward();
  }
  //delay();
}

void rightBT() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("RIGHT"));
  display.display();
  motorR.motorRev(turnspeed);  //Turn Right
  motorL.motorGo(turnspeed);
delay(10);
}

void readsensors() {
  // Button State
  buttonSelectState = digitalRead(buttonSelect);
  //TOF Sensors
  fl = sensor.readRangeContinuousMillimeters();
  fr = sensor2.readRangeContinuousMillimeters();
  //Floor Sensors
  potValueFL = analogRead(ffl);
  potValueFR = analogRead(ffr);
  //printsensors();
  if (buttonSelectState == LOW) {
    motorR.motorStop();
    motorL.motorStop();
    delay(1500);
    selectButton();
  }
}

void printsensors() {
  int buttonSelectState = HIGH;
  buttonSelectState = HIGH;

  // read the state of the pushbutton value:
  buttonSelectState = digitalRead(buttonSelect);
  while (buttonSelectState == HIGH) {
    buttonSelectState = digitalRead(buttonSelect);
    //TOF Sensors
    fl = sensor.readRangeContinuousMillimeters();
    fr = sensor2.readRangeContinuousMillimeters();
    //Floor Sensors
    potValueFL = analogRead(ffl);
    potValueFR = analogRead(ffr);
    //TOF Sensors
    Serial.print(" TOF Front Left: ");
    Serial.print(fr);
    Serial.print("  TOF Front Right: ");
    Serial.print(fl);
    //Floor Sensors
    Serial.print(" Line Sensor Left: ");
    Serial.print(potValueFL);
    Serial.print(" Line Sensor Right: ");
    Serial.print(potValueFR);
    Serial.println("  ");

    display.clearDisplay();
    display.setTextSize(1);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, 0);              // Start at top-left corner
    display.println(F("Lft  Rht  Blft Brht"));
    display.setCursor(0, 15);  // Start at top-left corner
    display.println(fl);
    display.setCursor(30, 15);  // Start at top-left corner
    display.print(fr);
    display.setCursor(60, 15);  // Start at top-left corner
    display.print(potValueFL);
    display.setCursor(90, 15);  // Start at top-left corner
    display.print(potValueFR);
    display.display();
    delay(200);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Exiting"));
  display.display();
  delay(3000);
  display.clearDisplay();
}

void countdown() {
  digitalWrite(ledPin, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(22, 10);
  display.println(F(" FIVE "));
  display.display();
  digitalWrite(ledPin, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);
  digitalWrite(ledPin, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(22, 10);
  display.println(F(" FOUR "));
  display.display();
  digitalWrite(ledPin, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);
  digitalWrite(ledPin, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(18, 10);
  display.println(F(" THREE "));
  display.display();
  digitalWrite(ledPin, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);
  digitalWrite(ledPin, HIGH);
  tone(piezoPin, 500, 500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(32, 10);
  display.println(F(" TWO "));
  display.display();
  digitalWrite(ledPin, LOW);
  delay(500);
  tone(piezoPin, 0, 500);
  delay(500);
  digitalWrite(ledPin, HIGH);
  tone(piezoPin, 1000, 1000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(32, 10);
  display.println(F(" ONE "));
  display.display();
  digitalWrite(ledPin, LOW);
  delay(1000);
  tone(piezoPin, 0, 500);
  delay(500);
  display.clearDisplay();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  // Start at top-left corner
  display.println(F("FIGHT!"));
  display.println(F("ME!"));
  display.display();
  buttonSelectState = HIGH;
  readsensors();
  left();  // Omit for pounce mode. Initial turning direction to start scanning for enemy
  loop();
}

void displaylcd() {
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("TOF Front"));
  display.setCursor(0, 15);  // Start at top-left corner
  display.println(F(fl));
  display.display();
  delay(200);
}

void selectButton() {
  buttonSelectState = HIGH;
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("    Push Select"));
  display.display();
  display.setCursor(0, 16);  // Start at top-left corner
  display.println(F("     to START"));
  display.display();
  delay(1500);
  int buttonSelectState = HIGH;
  // read the state of the pushbutton value:
  while (buttonSelectState == HIGH) {
    buttonSelectState = digitalRead(buttonSelect);
  }
  countdown();
}

void bootscreen() {
  display.clearDisplay();
  //The following displays the start up text upon power up.
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(13, 0);             
  display.println(F(name));
  delay(2000);
  display.clearDisplay();
  display.drawBitmap(0, 0, ControlThem, 128, 64, WHITE);  //Control them
  display.display();
  delay(1000);
  display.clearDisplay();
  display.drawBitmap(0, 0, BeforeThey, 128, 64, WHITE);  //Before they
  display.display();
  delay(750);
  display.clearDisplay();
  display.drawBitmap(0, 0, ControlYou, 128, 64, WHITE);  //Control you
  display.display();
  delay(1000);
  display.clearDisplay();
  display.drawBitmap(0, 0, MTRAS_SUMO, 128, 64, WHITE);  //MTRAS SUMO
  display.display();
  delay(1000);
  display.clearDisplay();
  display.drawBitmap(0, 0, Version, 128, 64, WHITE);  //Version
  display.display();
  delay(1000);
  display.clearDisplay();
}

void BT_Control(){
}