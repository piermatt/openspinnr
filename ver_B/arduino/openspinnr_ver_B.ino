/*****************************************************************************
  https://github.com/piermatt/openspinnr - Open-source electrospinning apparatus
******************************************************************************
  This is the arduino code for the electrospinner ver_B

  The two stepper motors are connected trough two ULN2003 modules and controlled by the AccelStepper library
  IN1-IN3-IN2-IN4 for proper step sequence
  stepper1(FULLSTEP, 9, 11, 10, 12); (syringe slider)
  stepper2(HALFSTEP, 5, 7, 6, 8); (syringe pump)

  The SSD1306 OLED is connected via I2C and controlled by the Adafruit library

  Input is provided by a rotary encoder:
  -First hardware interrupt pin is digital pin 2;
  -Second hardware interrupt pin is digital pin 3;
  -Pin 4 is the select switch for the encoder.
  Interrupt-based Rotary Encoder Sketch has been adapted from the work of Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt, Steve Spence
  http://exploreembedded.com/wiki/Interactive_Menus_for_your_project_with_a_Display_and_an_Encoder

*/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Include the AccelStepper Library
#include <AccelStepper.h>

// Define step constants
#define FULLSTEP 4
#define HALFSTEP 8

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper stepper1(FULLSTEP, 9, 11, 10, 12);
AccelStepper stepper2(HALFSTEP, 5, 7, 6, 8);

static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
static int selectSwitch = 4; //The select switch for our encoder.
int   option = 0;
int   syringe = 0;
long travel_lenght;
int travel_speed, syringe_speed;
unsigned long myTime;

volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile uint16_t encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile uint16_t oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

void setup() {

  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(selectSwitch, INPUT_PULLUP);
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  Serial.begin(9600); // start the serial monitor link
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  stepper1.setMaxSpeed(900.0);
  stepper1.setAcceleration(100.0);

  stepper2.setMaxSpeed(300.0);
  stepper2.setAcceleration(90.0);
}
void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void loop() {

  if (option == 0)
  {
    /// SET ORIGIN TRAVELER
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("<<Set ORIGIN<<");
    display.display();
    display.setTextSize(2);      // Normal 1:1 pixel scale

    if (oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
    }

    display.print(encoderPos * 2); display.print(" mm");
    display.display();
    delay(100);

    // The select switch is pulled high, hence the pin goes low if the switch is pressed.
    if (digitalRead(selectSwitch) == 0)
    {
      Serial.println("Key Pressed");
      delay(50); // wait for debounce to get over
      stepper1.setCurrentPosition(0);
      stepper1.moveTo(-(oldEncPos * 5000L));
      while (stepper1.currentPosition() != -(oldEncPos * 5000L)) // Full speed basck to 0
        stepper1.run();
      stepper1.stop(); // Stop as fast as possible: sets new target
      stepper1.runToPosition();
      stepper1.setCurrentPosition(0);
      display.print("OK!");
      display.display();
      delay(2000);
      option = 1;
    }
  }

  if (option == 1)
  {
    /// SET lenght TRAVELER
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("<>Set LENGTH<>");
    display.display();
    display.setTextSize(2);      // Normal 1:1 pixel scale

    if (oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
    }

    display.print(encoderPos * 2); display.print(" mm");
    display.display();
    delay(100);

    // The select switch is pulled high, hence the pin goes low if the switch is pressed.
    if (digitalRead(selectSwitch) == 0)
    {
      Serial.println("Key Pressed");
      delay(50); // wait for debounce to get over
      travel_lenght = oldEncPos * 5000L;
      display.print("OK!");
      display.display();
      delay(2000);
      option = 2;
    }
  }

  if (option == 2)
  {
    /// SET  speed
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("<<Set SPEED>>");
    display.display();
    display.setTextSize(2);      // Normal 1:1 pixel scale

    if (oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
    }

    display.print(encoderPos * 2); display.print(" mm");
    display.display();
    delay(100);

    // The select switch is pulled high, hence the pin goes low if the switch is pressed.
    if (digitalRead(selectSwitch) == 0)
    {
      Serial.println("Key Pressed");
      delay(50); // wait for debounce to get over
      travel_speed = oldEncPos * 10;
      //      stepper1.setMaxSpeed(travel_speed);
      display.print("OK!");
      display.display();
      delay(2000);
      option = 3;
    }
  }

  if (option == 3)
  {
    /// OPEN syringe
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("<<OPEN syringe<<");
    display.display();
    display.setTextSize(2);      // Normal 1:1 pixel scale

    if (oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
    }

    display.print(encoderPos * 2); display.print(" mm");
    display.display();
    delay(100);

    // The select switch is pulled high, hence the pin goes low if the switch is pressed.
    if (digitalRead(selectSwitch) == 0)
    {
      Serial.println("Key Pressed");
      delay(50); // wait for debounce to get over
      stepper2.setCurrentPosition(0);
      stepper2.moveTo((oldEncPos * 550L));
      while (stepper2.currentPosition() != (oldEncPos * 550L)) // Full speed basck to 0
        stepper2.run();
      stepper2.stop(); // Stop as fast as possible: sets new target
      stepper2.runToPosition();
      stepper2.setCurrentPosition(0);
      display.print("OK!");
      display.display();
      delay(2000);
      option = 4;
    }
  }

  if (option == 4)  {
    /// OPEN syringe
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("<<CLOSE syringe<<");
    display.display();
    display.setTextSize(2);      // Normal 1:1 pixel scale

    if (oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
    }

    display.print(encoderPos * 2); display.print(" mm");
    display.display();
    delay(100);

    // The select switch is pulled high, hence the pin goes low if the switch is pressed.
    if (digitalRead(selectSwitch) == 0)
    {
      Serial.println("Key Pressed");
      delay(50); // wait for debounce to get over
      stepper2.setCurrentPosition(0);
      stepper2.moveTo(-(oldEncPos * 550L));
      while (stepper2.currentPosition() != -(oldEncPos * 550L)) // Full speed basck to 0
        stepper2.run();
      stepper2.stop(); // Stop as fast as possible: sets new target
      stepper2.runToPosition();
      stepper2.setCurrentPosition(0);
      display.print("OK!");
      display.display();
      delay(2000);
      option = 5;
    }
  }

  if (option == 5)  {
    /// SET Syringe flow<>
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("<>Syringe flow<>");
    display.display();
    display.setTextSize(2);      // Normal 1:1 pixel scale

    if (oldEncPos != encoderPos) {
      Serial.println(encoderPos);
      oldEncPos = encoderPos;
    }

    display.print(encoderPos); display.print(" u");
    display.display();
    delay(100);

    // The select switch is pulled high, hence the pin goes low if the switch is pressed.
    if (digitalRead(selectSwitch) == 0)
    {
      Serial.println("Key Pressed");
      delay(50); // wait for debounce to get over
      syringe_speed = oldEncPos * 1;
      //      stepper1.setMaxSpeed(syringe_speed);
      display.print("OK!");
      display.display();
      delay(2000);
      option = 6;
    }

  }

  if (option == 6)  {

    /// routine
    Serial.print("current position, distancetogo, travel lenght");
    stepper1.setCurrentPosition(0);
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("spinning..yeeeeah!");
    display.display();
    display.setTextSize(2);      // Normal 1:1 pixel scale

    stepper1.setSpeed(travel_speed);
    stepper2.setSpeed(syringe_speed);
    stepper1.moveTo(travel_lenght);
    stepper2.moveTo(-5500);

    while (stepper2.distanceToGo() != 0) {
      if (stepper1.distanceToGo() == 0) {
        if (stepper1.currentPosition() == 0)
          stepper1.moveTo(travel_lenght);
        else
          stepper1.moveTo(0);


      }
      //      Serial.print(stepper1.currentPosition()); Serial.print(" "); Serial.print(stepper1.distanceToGo()); Serial.print(" "); Serial.println(travel_lenght);
      stepper1.run();
      stepper2.runSpeed();


      // The select switch is pulled high, hence the pin goes low if the switch is pressed.
      if (digitalRead(selectSwitch) == 0)
      {
        display.clearDisplay();
        display.println("pause!");
        display.display();
        stepper1.stop(); // Stop as fast as possible: sets new target
        delay(10000);

      }
    }

  }


}
