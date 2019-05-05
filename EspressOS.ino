/* Thomas Jeffries 2019 - thomasejeffries@gmail.com -
   Free to use and share under MIT license (see end of file for license blurb) */

/*  Max6675 #1 (brew) ==>   Arduino
      SCK             ==>     D6
      CS              ==>     D7
      SO              ==>     D8
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

/*   SSR #1 (brew)    ==>     D5   */

/*  Max6675 #2 (steam)==>   Arduino
      SCK             ==>     D10
      CS              ==>     D11
      SO              ==>     D12
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

/*   SSR #2 (steam)   ==>     D9   */

/*    i2c LCD Module  ==>   Arduino
      SCL             ==>     A5
      SDA             ==>     A4
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

/*    Rotary Encoder  ==>   Arduino
      SW              ==>     D2
      DTA             ==>     D3
      CLK             ==>     D4
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

/*  Pump switch down  ==>    D13
    Pump switch up    ==>    A0
    brew solenoid     ==>    A1
    steam solenoid    ==>    A2
    pump relay        ==>    A3 */

#include <Wire.h>
#include <MAX6675_Thermocouple.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); //usually 0x27 for 16x2; use I2C scanner to determine address
MAX6675_Thermocouple* thermo1;
MAX6675_Thermocouple* thermo2;

//I/O pin definitions:
#define ROTA_SW    2  //rotary encoder push switch
#define ROTA_DT    3  //rotary encoder data line
#define ROTA_CLK   4  //rotary encoder clock line
#define BOILER_1   5  //Pin for signal to SSR #1 (brew)
#define BOILER_2   9  //Pin for signal to SSR #2 (steam)
//MAX6675 #1 (brew temp) SPI pins
#define TEMP1_SCK  6
#define TEMP1_CS   7
#define TEMP1_SO   8
//MAX6675 #2 (steam temp) SPI pins
#define TEMP2_SCK  10
#define TEMP2_CS   11
#define TEMP2_SO   12
//switch input and relay outputs
#define PUMP_SW_DN 13
#define PUMP_SW_UP A0
#define BREW_SOLENOID A1
#define STEAM_SOLENOID A2
#define PUMP_RELAY A3

//thermocouple constants
const float thermoCal1 = -0.4;
const float thermoCal2 = -1.0;
const int TEMP_AVG_COUNT = 5;

//Brew boiler (boiler #1) temperature variables
float setTemp1 = 95; //Default brew temperature setpoint
float curTemp1, prevTemp1, error1, prevError1, PIDValue1;
uint8_t tempIndex1 = 0;
float tempArray1[TEMP_AVG_COUNT];

//Steam boiler (boiler #2) temperature variables
float setTemp2 = 133; //Default steam temperature setpoint
float curTemp2, prevTemp2, error2, prevError2, PIDValue2;
uint8_t tempIndex2 = 0;
float tempArray2[TEMP_AVG_COUNT];

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
int boilerDuty1, boilerDuty2;

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
  pinMode(BOILER_1, OUTPUT);
  pinMode(BOILER_2, OUTPUT);
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

  //setup both MAX6675 modules:
  thermo1 = new MAX6675_Thermocouple(TEMP1_SCK, TEMP1_CS, TEMP1_SO);
  thermo2 = new MAX6675_Thermocouple(TEMP2_SCK, TEMP2_CS, TEMP2_SO);


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

  boilerDuty1 = 0;
  boilerDuty2 = 0;

  lcd.init();
  lcd.backlight();
}


void loop() {
  //update menu state base on button click count
  menuState += buttonCount;
  buttonCount = 0;

  //perform PID calculation and set boiler duty cycle every 1s
  if ((loopCount % 10) == 0) {
    calculatePID();
  }

  //update display every 500ms
  if ((loopCount % 5) == 0) {
    //subtract oldest sample from average
    curTemp1 -= tempArray1[tempIndex1];
    curTemp2 -= tempArray2[tempIndex2];
    //save temp samples in averaging array
    tempArray1[tempIndex1] = thermo1->readCelsius() + thermoCal1;
    tempArray2[tempIndex1] = thermo2->readCelsius() + thermoCal2;
    //add newest sample to average
    curTemp1 += tempArray1[tempIndex1];
    curTemp2 += tempArray2[tempIndex2];
    //calculate new temp array indices
    tempIndex1 = (++tempIndex1) % TEMP_AVG_COUNT;
    tempIndex2 = (++tempIndex2) % TEMP_AVG_COUNT;

    lcd.setCursor(0, 0);
    lcd.print(curTemp1 / (TEMP_AVG_COUNT + 0.0), 2);
    lcd.setCursor(8, 0);
    lcd.print(curTemp2 / (TEMP_AVG_COUNT + 0.0), 2);
    lcd.setCursor(0, 1);
    lcd.print(boilerDuty1);
    lcd.setCursor(8, 1);
    lcd.print(boilerDuty2);

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

const uint8_t CLOCK_MULTIPLE = 1;
//duty cycle implementation method
void setBoilerStates() {

  if (loopCount % (10 * CLOCK_MULTIPLE) == 0) {
    digitalWrite(BOILER_1, HIGH);
    digitalWrite(BOILER_2, HIGH);
  }

  if (loopCount % (10 * CLOCK_MULTIPLE) >= (boilerDuty1 * CLOCK_MULTIPLE)) {
    digitalWrite(BOILER_1, LOW);
  }

  if (loopCount % (10 * CLOCK_MULTIPLE) >= (boilerDuty2 * CLOCK_MULTIPLE)) {
    digitalWrite(BOILER_2, LOW);
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
  //  boilerDuty1 = 5;//SSR test routine
  //  boilerDuty2 = 5;
  //  return;

  //perform PID math here
  float p1, i1, d1, p2, i2, d2;

  //setup PID vars
  prevTime = curTime; curTime = millis();
  error1 = setTemp1 - (curTemp1 / (TEMP_AVG_COUNT + 0.0));
  error2 = setTemp2 - (curTemp2 / (TEMP_AVG_COUNT + 0.0));

  //calculate P values
  p1 = kp1 * error1;
  p2 = kp2 * error2;

  //calculate I values
  i1 = ki1 * (error1 + prevError1) / 2.0 * ((curTime - prevTime) / 1000.0);
  i2 = ki2 * (error2 + prevError2) / 2.0 * ((curTime - prevTime) / 1000.0);

  //calculate D values
  d1 = kd1 * ((error1 - prevError1) / ((curTime - prevTime) / 1000.0));
  d2 = kd2 * ((error2 - prevError2) / ((curTime - prevTime) / 1000.0));

  //reset vars for next cycle
  prevTemp1 = curTemp1; prevError1 = error1;
  prevTemp2 = curTemp2; prevError2 = error2;

  //sum and set PID values
  PIDValue1 = p1; // + i1 + d1;
  PIDValue2 = p2; // + i2 + d2;

  //  lcd.setCursor(8,1);
  //  lcd.print(p1);
  //  lcd.print(i1);
  //  lcd.print(d1);
  //lcd.print(p1+" "+i1+" "+d1);

  //round (add .5 and truncate to int) to get duty cycle
  boilerDuty1 = (PIDValue1 + 0.5);
  boilerDuty2 = (PIDValue2 + 0.5);
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
