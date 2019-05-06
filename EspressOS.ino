/* Thomas Jeffries 2019 - thomasejeffries@gmail.com -
   Free to use and share under MIT license (see end of file for license blurb) */

/*  Thermister (brew) ==>     A0
    Pressure (steam)  ==>     A1
    Steam water Level ==>     A2   */

/* I2C LCD Module
      Gnd             ==>     Gnd
      Vcc             ==>     Vcc (5v)
      SDA             ==>     A4
      SCL             ==>     A5      */

/* Rotary Encoder
      SW              ==>     D2
      DTA             ==>     D3
      CLK             ==>     D4
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

/*   SSR #1 (brew)    ==>     D5   */
/*   SSR #2 (steam)   ==>     D6   */

/*  Pump switch down  ==>     D7
    Pump switch up    ==>     D8
    brew solenoid     ==>     D9
    steam solenoid    ==>     D10
    pump relay        ==>     D11 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); //usually 0x27 for 16x2; use I2C scanner to determine address

//I/O pin definitions:
#define ROTA_SW    2  //rotary encoder push switch
#define ROTA_DT    3  //rotary encoder data line
#define ROTA_CLK   4  //rotary encoder clock line
#define BREW_COIL  5  //Pin for signal to SSR #1 (brew)
#define STEAM_COIL 6  //Pin for signal to SSR #2 (steam)
//switches and relays
#define PUMP_SW_DN     7
#define PUMP_SW_UP     8
#define PUMP_RELAY     9
#define BREW_SOLENOID  10
#define STEAM_SOLENOID 11
//boiler sensor inputs
#define BREW_TEMP    A0
#define STEAM_PRESS  A1
#define STEAM_LEVEL  A2

//thermocouple constants
const float thermoCal1 = -0.4;

//Brew boiler (boiler #1) temperature variables
float setTemp = 95; //Default brew temperature setpoint (Celsius)
float curTemp, prevTemp, tempErr, prevTempErr, PIDTemp;

//Steam boiler (boiler #2) temperature variables
float setPress = 1.5; //Default steam pressure setpoint (Bar)
float curPress, prevPress, pressErr, prevPressErr, PIDPress;

//boiler elapsed time variables
float curTime, prevTime;

//display state variables
int buttonPressed, menuState;

//Rotary encoder state variables
volatile uint8_t clkState, lastState, dtState, swPinMask, dtPinMask, clkPinMask;
volatile uint8_t *swPinPort, *dtPinPort, *clkPinPort;

//PID constants///////////////////////////////////////
float kp1 = 1.0, ki1 = 0.0, kd1 = -1.0;
float kp2 = 2.0, ki2 = 0.0, kd2 = -4.0;

//Boiler duty cycle settings (0-10, x10%)
int brewDuty, steamDuty;

///////////////////CONTROL/INPUT//////////////
volatile int rotorValue;
volatile uint8_t rotorOffset = 0, buttonCount = 0;

//Cycle variables
uint8_t loopCount;

//Menu enum
enum menu : int {
  controlLoop = 0,
  brewSet = 1,
  steamSet = 2,
  preInfusionSet = 3,
};

/*-------------------------------------------------------------*/
void setup() {
  //DEBUG/TESTING
  Serial.begin(9600);

  //setup IO pins
  pinMode(BREW_COIL, OUTPUT);
  pinMode(STEAM_COIL, OUTPUT);
  //set relay pins base state to high (off) to prevent flutter on startup
  pinMode(BREW_SOLENOID, INPUT_PULLUP);
  pinMode(STEAM_SOLENOID, INPUT_PULLUP);
  pinMode(PUMP_RELAY, INPUT_PULLUP);
  pinMode(BREW_SOLENOID, OUTPUT);
  pinMode(STEAM_SOLENOID, OUTPUT);
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(PUMP_SW_DN, INPUT_PULLUP);//brew
  pinMode(PUMP_SW_UP, INPUT_PULLUP);//fill steam boiler

  curTime = millis();

  //register fast-access ports and corresponding masks
  swPinMask = digitalPinToBitMask(ROTA_SW);
  *swPinPort = portInputRegister(digitalPinToPort(ROTA_SW));
  dtPinMask = digitalPinToBitMask(ROTA_DT);
  *dtPinPort = portInputRegister(digitalPinToPort(ROTA_DT));
  clkPinMask = digitalPinToBitMask(ROTA_CLK);
  *clkPinPort = portInputRegister(digitalPinToPort(ROTA_CLK));

  pinMode(ROTA_DT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ROTA_SW), buttonInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ROTA_DT), rotateInterrupt, RISING);

  // read the pin
  //uint8_t swValue = (*swPinPort & swPinMask) != 0;

  rotorValue = 0;
  loopCount = 0;
  menuState = 0;

  brewDuty = 0;
  steamDuty = 0;

  lcd.init();
  lcd.backlight();
}


void loop() {
  //update menu state based on button click count
  menuState += buttonCount;
  buttonCount = 0;

  //perform PID calculation and set boiler duty cycle every 1s
  if ((loopCount % 10) == 0) {
    calculatePID();
  }

  //update display every 500ms
  if ((loopCount % 5) == 0) {

    getBrewTemp();
    getSteamPress();

    lcd.setCursor(0, 0);
    lcd.print(curTemp, 2);
    lcd.setCursor(8, 0);
    lcd.print(curPress, 2);
    lcd.setCursor(0, 1);
    lcd.print(brewDuty);
    lcd.setCursor(8, 1);
    lcd.print(steamDuty);

//    char out[32];
//    int curTrunk1 = curTemp1/TEMP_AVG_COUNT, curDecimal1 = (curTemp1*10-curTrunk1*10)/(10*TEMP_AVG_COUNT);
//    int curTrunk2 = curTemp2/TEMP_AVG_COUNT, curDecimal2 = (curTemp2*10-curTrunk2*10)/(10*TEMP_AVG_COUNT);
//    sprintf(out, "brew: %d.%d; steam: %d.%d\n", curTrunk1, curDecimal1, curTrunk2, curDecimal2);
//    Serial.print(out);
  }

  setBoilerStates();
  setSwitchStates();

  if (menuState != 0) {
    menu();
    return;
  }

  //loop delay & counting
  delay(50);
  loopCount = (++loopCount) % 200;
}


void getBrewTemp(){
  uint8_t brewVoltage = analogRead(BREW_TEMP);
}


void getSteamPress(){
  uint8_t steamVoltage = analogRead(STEAM_PRESS);
}


void getWaterLevel(){
  uint8_t levelVoltage = analogRead(STEAM_LEVEL);
}


const uint8_t CLOCK_MULTIPLE = 1;
//duty cycle implementation method
void setBoilerStates() {
  uint8_t slice = loopCount % (10 * CLOCK_MULTIPLE);
  if (slice == 0) {
    digitalWrite(BREW_COIL, HIGH);
    digitalWrite(STEAM_COIL, LOW);
  }

  if (slice >= (brewDuty * CLOCK_MULTIPLE)) {
    digitalWrite(BREW_COIL, LOW);
    if (slice >= ((10-steamDuty) * CLOCK_MULTIPLE)) { //nested to prevent combined 3kW load :P
      digitalWrite(STEAM_COIL, HIGH);
    }
  }
}


//reads pump switch position and sets relay states accordingly
void setSwitchStates() {
  uint8_t swDn = digitalRead(PUMP_SW_DN);
  uint8_t swUp = digitalRead(PUMP_SW_UP);
  digitalWrite(BREW_SOLENOID, !swDn);
  digitalWrite(STEAM_SOLENOID, !swUp);
  digitalWrite(PUMP_RELAY, (swDn | swUp));
}


//breakout method for menu state functionality
void menu() {

}

/* float setTemp2 = 80;
   float curTemp2, prevTemp2, error2, prevError2, elapsedTime2,
        time2, timePrev2, PIDValue2;      */
//performs PID calculation and sets boiler duty cycle globals (0-10, proportion of time on)
void calculatePID() {
    brewDuty = 5;//SSR test routine
    steamDuty = 5;
    return;

//  //perform PID math here
//  float p1, i1, d1, p2, i2, d2;
//
//  //setup PID vars
//  prevTime = curTime; curTime = millis();
//  error1 = setTemp1 - (curTemp1 / (TEMP_AVG_COUNT + 0.0));
//  error2 = setTemp2 - (curTemp2 / (TEMP_AVG_COUNT + 0.0));
//
//  //calculate P values
//  p1 = kp1 * error1;
//  p2 = kp2 * error2;
//
//  //calculate I values
//  i1 = ki1 * (error1 + prevError1) / 2.0 * ((curTime - prevTime) / 1000.0);
//  i2 = ki2 * (error2 + prevError2) / 2.0 * ((curTime - prevTime) / 1000.0);
//
//  //calculate D values
//  d1 = kd1 * ((error1 - prevError1) / ((curTime - prevTime) / 1000.0));
//  d2 = kd2 * ((error2 - prevError2) / ((curTime - prevTime) / 1000.0));
//
//  //reset vars for next cycle
//  prevTemp1 = curTemp1; prevError1 = error1;
//  prevTemp2 = curTemp2; prevError2 = error2;
//
//  //sum and set PID values
//  PIDValue1 = p1; // + i1 + d1;
//  PIDValue2 = p2; // + i2 + d2;
//
//  //  lcd.setCursor(8,1);
//  //  lcd.print(p1);
//  //  lcd.print(i1);
//  //  lcd.print(d1);
//  //lcd.print(p1+" "+i1+" "+d1);
//
//  //round (add .5 and truncate to int) to get duty cycle
//  boilerDuty1 = (PIDValue1 + 0.5);
//  boilerDuty2 = (PIDValue2 + 0.5);
}


//interrupt routine performed when rotary encoder data pin RISING (to high state)
void rotateInterrupt() {
  if ((*clkPinPort & clkPinMask) == 0) {
    rotorOffset++;
  } else {
    rotorOffset--;
  }
}


//interrupt routine performed on rotary encoder button press
void buttonInterrupt() {
  buttonCount++;
}



/*
  Copyright (c) 2019 Thomas Jeffries

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
