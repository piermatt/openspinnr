/*****************************************************************************
https://github.com/piermatt/openspinnr - Open-source electrospinning apparatus
******************************************************************************
 This is the arduino code for the electrospinner ver_A

 The two stepper motors are connected trough two easydriver modules and controlled by the AccelStepper library
              pin
 STEPPER 1   9, 8     (syringe pump)
 STEPPER 2  12, 13    (syringe slider)

          potentiometer     switch
             (speed)    (fwd/stop/bwd)
 syringe       A13            A8 
 slider        A14            A9
 
 The SSD1306 OLED is connected via I2C and controlled by the Adafruit library

 ****************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#include <math.h>

#include <AccelStepper.h>

int sy_dir, x_dir, y_dir, sy_pot, x_pot, sy_speed, x_speed, x_sign, sy_sign, sy_avg;
int period_display = 1000;
unsigned long time_now = 0;
AccelStepper stepper1(AccelStepper::DRIVER, 9, 8);
AccelStepper stepper2(AccelStepper::DRIVER, 12, 13);

// Define our maximum and minimum speed in steps per second (scale pot to these)
#define  MAX_SPEED 5000
#define  MIN_SPEED 0.02


void setup()
{
  // The only AccelStepper value we have to set here is the max speeed, which is higher than we'll ever go
  stepper1.setMaxSpeed(10000.0);
  stepper2.setMaxSpeed(10000.0);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();


  for (int16_t i = 0; i < max(display.width(), display.height()) / 2; i += 5) {
    display.drawTriangle(
      display.width() / 2  , display.height() / 2 - i,
      display.width() / 2 - i, display.height() / 2 + i,
      display.width() / 2 + i, display.height() / 2 + i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

}

void loop()
{

  static int analog_read_counter = 1000;    // Counts down to 0 to fire analog read
  static char sign = 0;                     // Holds -1, 1 or 0 to turn the motor on/off and control direction

  //syringe direction
  if (sy_dir > 600) {
    sy_sign = 0;
  }
  else if (sy_dir > 300) {
    sy_sign = -1;
  }
  else {
    sy_sign = +1;
  }
  
  //slider direction
  if (x_dir > 600) {
    x_sign = 0;
  }
  else if (x_dir > 300) {
    x_sign = -1;
  }
  else {
    x_sign = +1;
  }


  if (analog_read_counter > 0) {
    analog_read_counter--;
  }
  else {
    analog_read_counter = 3000;
    //syr

    stepper1.runSpeed();
    int    avg_count = 9;
    if (avg_count > 0) {
      avg_count--;
      sy_pot = analogRead(A13);
      sy_avg = sy_pot + sy_avg;
    }
    sy_avg = sy_avg / 9;
    sy_dir = analogRead(A8);
    //  And scale the pot's value from min to max speeds
    if (sy_pot > 600) {
      sy_speed = sy_sign * (((sy_pot / 676.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED);
    } else {
      sy_speed = sy_sign * (((sy_pot / 676.0) * (MAX_SPEED / 25 - MIN_SPEED)) + MIN_SPEED);
    }//lineare
    //    sy_speed = sy_sign * (pow(1.012, sy_avg)); //exp
    // Update the stepper to run at this new speed
    stepper1.setSpeed(sy_speed);
    Serial.print(sy_pot); Serial.print(" "); Serial.print(sy_speed); Serial.print(" _ ");

    //slider
    x_pot = analogRead(A14);
    stepper2.runSpeed();
    x_dir = analogRead(A9);
    //  And scale the pot's value from min to max speeds
    x_speed = x_sign * (((x_pot / 676.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED);
    // Update the stepper to run at this new speed
    stepper2.setSpeed(x_speed);
    Serial.print(x_pot); Serial.print(" "); Serial.print(x_speed); Serial.println("");
    //

  }

  // This will run the stepper at a constant speed
  stepper1.runSpeed();
  stepper2.runSpeed();

  if (millis() >= time_now + period_display) {
    time_now += period_display;
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.print("<==> "); display.print(x_speed * 0.0645); display.println(" mm/min");
    display.print("|[2ml]- "); display.print(sy_speed * 0.1024); display.println(" ml/h");
    display.print("|[5ml]- "); display.print(sy_speed * 0.2039); display.println(" ml/h");
    display.display();

  }
}
