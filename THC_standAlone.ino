

/*/Pin allocation
   A0 = arcVoltageInput// voltage input from voltage divider in plasma machine.
   A2 = Stepper speed, ie, period between pulses.
   A3 = setPointInput //pot to adjust voltage setpoint.
   A4 = SDA LCD
   A5 = SCL LCD
   D2 = interruptPin //For reading pulses from controller.
   D4 = arcOkInput //Arc OK signal from plasma machine.
   D5 = dir //Directly connected to stepper driver.
   D6 = step //Directly connected to stepper driver.
   D7 = grblDirPin//dir signal from controller.
   D8 = lowLedPin //independant led pin
   D9 = stepPin //to stepper controller
   D10 = arcOkLedPin // independant led pin
   D11 = engageThcSwitch //passthrough or thc control.

*/
#include "Arduino.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

#include <TimerOne.h>

namespace
{
constexpr int arcVoltageInput = A0;
constexpr int pulseIntervalInput = A2;
constexpr int setPointInput = A3;
constexpr int interruptPin = 2;
constexpr int arcOkInput = 4;
constexpr int dirPin = 5;
constexpr int highLedPin = 6;
constexpr int grblDirPin = 7;
constexpr int lowLedPin = 8;
constexpr int stepPin = 9;
constexpr int arcOkLedPin = 10;
constexpr int engageThcSwitch = 11;

constexpr unsigned deadband = 5;
int arcVoltage;
int setPoint;
long pulseInterval = 800;
bool arcOk;
bool cw = true;
bool arcVHigh;
bool arcVLow;
bool engageThc = HIGH;
}

void readInputs()
{
  arcVoltage = analogRead(arcVoltageInput);
  setPoint = map(analogRead(setPointInput), 474, 1024, 778, 614);//Will need conversion to arcVoltage range.
  pulseInterval = map(analogRead(pulseIntervalInput), 0, 1024, 0, 10000);//In microseconds.
  engageThc = !digitalRead(engageThcSwitch);
  arcOk = !digitalRead(arcOkInput);//Switches THC on only when arcOk signal from plasma machine.
}


void setup() {
  pinMode(arcOkInput, INPUT_PULLUP);
  pinMode(dirPin, OUTPUT);
  pinMode(grblDirPin, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(stepPin, OUTPUT);
  pinMode(engageThcSwitch, INPUT_PULLUP);
  pinMode(highLedPin, OUTPUT);
  pinMode(lowLedPin, OUTPUT);
  pinMode(arcOkLedPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), passThrough, FALLING);
  Timer1.initialize();
  //Timer1.pwm(stepPin, 126);

  Serial.begin(115200);
  Serial.println("Stand alone THC starting");
  lcd.init();                      // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("GRBL THC");
  delay(1000);
  lcd.clear();


}

void loop() {
  readInputs();
  updateLcd();
  arcOk = !digitalRead(arcOkInput);//Switches THC on only when arcOk signal from plasma machine.
  arcVHigh = arcVoltage > setPoint + deadband;
  arcVLow = arcVoltage < setPoint - deadband;

  if (engageThc) {
    digitalWrite(arcOkLedPin, HIGH);
    if (arcOk) {
      if (arcVHigh) {
        digitalWrite(dirPin, HIGH);
        digitalWrite(highLedPin, HIGH);
        digitalWrite(lowLedPin, LOW);
        Timer1.pwm(stepPin, 512, pulseInterval);
      }
      else if (arcVLow) {
        digitalWrite(dirPin, LOW);
        digitalWrite(lowLedPin, HIGH);
        digitalWrite(highLedPin, LOW);
        Timer1.pwm(stepPin, 512, pulseInterval);
      }
      else {
        //analogWrite(stepPin, 255);
        digitalWrite(lowLedPin, LOW);
        digitalWrite(highLedPin, LOW);
        Timer1.pwm(stepPin, 1024, pulseInterval);
      }
    }

    else {      

      stop();
    }
  }
  else {
    stop();
    digitalWrite(arcOkLedPin, LOW);
  }
}
void passThrough()
{
  digitalWrite(stepPin, LOW);
  digitalWrite(stepPin, HIGH);
}

void stop()
{
  Timer1.stop();
  Timer1.disablePwm(stepPin);
  digitalWrite(dirPin, digitalRead(grblDirPin));
        digitalWrite(lowLedPin, LOW);
        digitalWrite(highLedPin, LOW);  
}
void updateLcd()
{
  long previousMillis;
  if (previousMillis + 500 <= millis()) {
    if (arcOk) {
      lcd.setCursor(0, 0);
      lcd.print("THC Active  ");

      if (arcVHigh) {
        lcd.setCursor(0, 1);
        lcd.print("Dn");
      }
      else if (arcVLow) {
        lcd.setCursor(0, 1);
        lcd.print("Up");
      }
      else {
        lcd.setCursor(0, 1);
        lcd.print("OK");
      }
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("THC inactive");
      lcd.setCursor(0, 1);
      lcd.print("   ");
    }
    //lcd.clear();
    lcd.setCursor(6, 1); lcd.print("ArcV ");
    int V = ((50 * (float(setPoint) / 10)) / 205) * 10;
    lcd.print(V);

    previousMillis = millis();
  }
}
