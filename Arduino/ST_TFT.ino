
#include <SPI.h>
#include <stdlib.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
#include "Weller.c"

// Rotary encoder declarations
static int pinA = 2;              // Our first hardware interrupt pin is digital pin 2
static int pinB = 3;              // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0;          // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0;          // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0;     //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int oldEncPos = 0;      //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0;        //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

// Button reading, including debounce without delay function declarations
const byte buttonPin = 4;        // this is the Arduino pin we are connecting the push button to
byte oldButtonState = HIGH;       // assume switch open because of pull-up resistor
const unsigned long debounceTime = 50;  // milliseconds
unsigned long buttonPressTime=0;    // when the switch last changed state
boolean buttonPressed = 0;        // a flag variable
boolean lastbuttonPressed = 0;
boolean buttonPressed_OLD = 0;

//Sense pin to put the solder iron off when in the docking position
static int SensePin = 5;

#define dc 8
#define rst 12
#define cs_tft 10
Adafruit_ST7735 tft = Adafruit_ST7735(cs_tft, dc, rst); // Invoke custom library

#define DISPLAY_REFRESH 200
#define BUTTON_REFRESH 2000

int tempSOLL_OLD,tempSOLL=0;
int SetTemp=200;
int SetTemp_OLD=0;
int MaxTemp=420;
int MinTemp=10;
int analogPin = 3;
unsigned long lastTime,thisTime,lastButtonTime=0;
bool STANDBY=false;      //when power on set it to standby to allow setting of temp before it warms
bool ManualSTANDBY=true;
bool OldManualSTANDBY=false;
bool ERROR=false;
byte PWM=128;
byte  SenseInput;

//Menu system
bool GraphMenu=false;
bool StandardMenu=true;
bool Refresh=true;
int PosX=0;

//EEPROM
int eeAddress = 0;   //Location we want the data to be put.
int eeLength;

//PWM
#define PWM_DIV 1024 /// Note that the base frequency for pins 5 and 6 is 62500 Hz, giving 62500/1024=61 Hz
#define PWMpin 9

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=20, Ki=1, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  
  //Serial.begin(9600);     // DEBUGGING: opens serial port, sets data rate to 9600 bps
  
  //Rotary encoder section of setup
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  // button section of setup
  pinMode (buttonPin, INPUT_PULLUP); // setup the button pin

//EEPROM read last Setpoint
  SetTemp=EEPROMReadInt(eeAddress);

  encoderPos=SetTemp;

//SENSE
  pinMode(SensePin,INPUT);

//PWM
  pinMode(PWMpin, OUTPUT);
  setPwmFrequency(PWMpin, PWM_DIV);

//PID
  Input = analogRead(analogPin);
  Setpoint = SetTemp;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100); //default is 100, we have 16ms pwm frequency, thus approx 6 pwm periods per update

//Init display
  tft.initR(INITR_GREENTAB);

  tft.setRotation(2); // 0 - Portrait, 1 - Lanscape
  tft.fillScreen(ST7735_BLACK);
  tft.setTextWrap(false);

  tft.setFont(&FreeSans9pt7b);
  tft.drawBitmap(0,0,Weller,128,42,ST7735_CYAN);
  tft.drawBitmap(0,42,Weller2,128,86,ST7735_WHITE);
  
  delay(3000);
  tft.fillScreen(ST7735_BLACK);
}

//---------------------------------Main loop--------------------------------------------------
void loop(){

  rotarySwitch();

  thisTime = millis();
  
  if(buttonPressed != buttonPressed_OLD) {         //Switch standby state by pressing the key
    //Serial.println("Button pressed");// DEBUGGING.
    buttonPressed_OLD=buttonPressed;
    if (buttonPressed == 1) {
      STANDBY=not(STANDBY);
      Refresh=true;
    }
  }

  if(thisTime - lastButtonTime >= BUTTON_REFRESH)   //Hold button in for more than a second
  {
    lastButtonTime = thisTime;
    if (buttonPressed == 1) {
      if(lastbuttonPressed==buttonPressed)
      {
        GraphMenu=not(GraphMenu);
        //StandardMenu=false;
        Refresh=true;
      }
    }
  }
  else {lastbuttonPressed=buttonPressed;}


  if(STANDBY|ManualSTANDBY) {
      Setpoint=0;
      ERROR=true;
    }
  else {
      Setpoint=SetTemp;
      ERROR=false;
    }
 
  if(thisTime - lastTime >= DISPLAY_REFRESH)                    //Update display at set interval and switch between menus
  {
    lastTime = thisTime;
    //Serial.println(Output);// DEBUGGING.
        
    //if(tempSOLL<150) {ERROR=true;}
    //else {ERROR=false;}

    if (!GraphMenu) {
      DisplayText();
    }
    else {
      DisplayGraph();
    }
    //Serial.println(PWM);// DEBUGGING.

    SenseInput = digitalRead (SensePin);                    //Put it here to avoil debounce - function to disable heating when in solder station
    if(SenseInput==LOW) 
    {
      ManualSTANDBY=true;
      }
    else
    {
      ManualSTANDBY=false;
    }

  if(OldManualSTANDBY!=ManualSTANDBY)
  {
    OldManualSTANDBY=ManualSTANDBY;
    Refresh=true;
  }
    
  }

  
//PID loop  
  myPID.SetMode(MANUAL);
  analogWrite(PWMpin, 0);
  delay(7);
  Input = analogRead(analogPin)*0.42+30;
  tempSOLL=Input;
  myPID.SetMode(AUTOMATIC);
  PWM=Output;
  analogWrite(PWMpin, PWM);

  myPID.Compute();
}

//--------------------------------Display Graph routine-----------------------------------------------
unsigned long DisplayGraph() {
  PosX++;
  if (PosX>(128-28)) {
    PosX=0;
    Refresh=true;
  }
  
  if (Refresh==true) {
    tft.fillScreen(ST7735_BLACK);
    tft.drawLine(27,0,27,100,ST7735_WHITE);
    tft.drawLine(27,100,127,100,ST7735_WHITE);
    tft.setFont();
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(27,110);
    tft.print("0    5   10   15");
    tft.setCursor(0,0);
    tft.print("400");
    tft.setCursor(0,25);
    tft.print("300");
    tft.setCursor(0,50);
    tft.print("200");
    tft.setCursor(0,75);
    tft.print("100");

    if(ERROR) { tft.fillCircle(10,115, 3, ST7735_RED);}
    else { tft.fillCircle(10,115, 3, ST7735_GREEN);}
  }

  tft.writePixel(28+PosX,100-tempSOLL/4,ST7735_YELLOW);
  tft.setCursor(80,75);
  tft.print("ACT");
  tft.setCursor(80,85);
  tft.print("SET");
  tft.fillRect(110,65,127-65,30,ST7735_BLACK);
  tft.setCursor(110,75);
  tft.print(tempSOLL);
  tft.setCursor(110,85);
  tft.print(SetTemp);

  Refresh=false;
}

//--------------------------------Display routine-----------------------------------------------
unsigned long DisplayText() {

  if (Refresh==true) {
    tft.fillScreen(ST7735_BLACK);
  }

  tft.setTextColor(ST7735_WHITE);
  tft.setFont(&FreeSans9pt7b);

  //tft.setTextScale(2);
  tft.setTextSize(1);
  tft.setCursor(0, 25);
  tft.print("SET");
  tft.setCursor(0, 70);
  tft.print("ACT");

  tft.setCursor(115, 10);
  tft.print("o");
  tft.setCursor(115, 55);
  tft.print("o");
  
  tft.setFont();
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);

  int x=100;
  tft.setCursor(x, 115);
  tft.print(" PWM");
  //tft.fillRect(20,95,107,35,ST7735_BLACK);
  int Prosent=0.4*PWM;
  //tft.setCursor(20, 120);
  //tft.print("PWM " + String(Prosent)+" %   ");
  tft.fillRect(x, 125,(128-x)*Prosent/100 , 3, ST7735_WHITE); //(PWM/256)*(128-x)
  tft.fillRect((128-x)*Prosent/100+x, 125,(128-x)*(1-Prosent/100) , 3, ST7735_BLACK);

  if(ERROR) { tft.fillCircle(10,115, 3, ST7735_RED);}
  else { tft.fillCircle(10,115, 3, ST7735_GREEN);}

//----------------------------------Lodde temp------------------------------------------------

  tft.setFont();
  tft.setTextSize(4);
  if ((abs(tempSOLL_OLD-tempSOLL)>1)|(Refresh))
  {
    tft.setCursor(40, 45);
    tft.setTextColor(ST7735_BLACK);
    if ((tempSOLL_OLD / 100) != (tempSOLL / 100))
    {
      tft.print(tempSOLL_OLD / 100);
    }
    else
    {
      tft.print(" ");
    }
    if (((tempSOLL_OLD / 10) % 10) != ((tempSOLL / 10) % 10))
    {
      tft.print((tempSOLL_OLD / 10) % 10);
    }
    else
    {
      tft.print(" ");
    }
    
    if ((tempSOLL_OLD % 10) != (tempSOLL % 10))
    {
      tft.print(tempSOLL_OLD % 10);
    }

    //Neuen Wert in Weiß schreiben
    tft.setCursor(40, 45);

    if (abs(tempSOLL-SetTemp)>20) { tft.setTextColor(ST7735_RED);}
    else if (abs(tempSOLL-SetTemp)>5) { tft.setTextColor(ST7735_YELLOW);}
    else { tft.setTextColor(ST7735_GREEN);}
    
    if(STANDBY|ManualSTANDBY) {tft.setTextColor(ST7735_BLUE);}

    if (tempSOLL < 100)
    {
      tft.print(" ");
    }
    if (tempSOLL < 10)
    {
      tft.print(" ");
    }
    tft.print(tempSOLL);
    tempSOLL_OLD = tempSOLL;
  }
  
//-------------------------------------SET temp------------------------------------------------  
  if ((SetTemp_OLD != SetTemp)|(Refresh))
  {
    tft.setCursor(40, 0);
    tft.setTextColor(ST7735_BLACK);
    if (((SetTemp_OLD / 100) != (SetTemp / 100)|(Refresh)))
    {
      tft.print(SetTemp_OLD / 100);
    }
    else
    {
      tft.print(" ");
    }
    if (((SetTemp_OLD / 10) % 10) != ((SetTemp / 10) % 10))
    {
      tft.print((SetTemp_OLD / 10) % 10);
    }
    else
    {
      tft.print(" ");
    }
    
    if ((SetTemp_OLD % 10) != (SetTemp % 10))
    {
      tft.print(SetTemp_OLD % 10);
    }

    //Neuen Wert in Weiß schreiben
    tft.setCursor(40, 0);
    tft.setTextColor(ST7735_CYAN);

    if (SetTemp < 100)
    {
      tft.print(" ");
    }
    if (SetTemp < 10)
    {
      tft.print(" ");
    }
    tft.print(SetTemp);
    SetTemp_OLD = SetTemp;
  }
  Refresh=false;
}

void rotarySwitch() { //This handles the bulk of the menu functions without needing to install/include/compile a menu library
  if(oldEncPos != encoderPos) { 
    //Serial.println(encoderPos);// DEBUGGING. Sometimes the serial monitor may show a value just outside modeMax due to this function. The menu shouldn't be affected.
    if (encoderPos > MaxTemp) {encoderPos = MaxTemp; }// check we haven't gone out of bounds below 0 and correct if we have
    else if (encoderPos < MinTemp) { encoderPos = 10;} // check we haven't gone out of bounds above modeMax and correct if we have
    else {
      SetTemp=encoderPos;
      EEPROMWriteInt(eeAddress,SetTemp);
    }
    oldEncPos = encoderPos;
  }
  // Button reading with non-delay() debounce - thank you Nick Gammon!
  byte buttonState = digitalRead (buttonPin); 
  if (buttonState != oldButtonState){
    if (millis () - buttonPressTime >= debounceTime){ // debounce
      buttonPressTime = millis ();  // when we closed the switch 
      oldButtonState =  buttonState;  // remember for next time 
      if (buttonState == LOW){
        buttonPressed = 1;
        //Serial.println ("Button closed"); // DEBUGGING: print that button has been closed
      }
      else {
        buttonPressed = 0;  
        //Serial.println ("Button opened"); // DEBUGGING: print that button has been opened
      }  
    }  // end if debounce time up
  } // end of state change
}

//Rotary encoder interrupt service routine for one encoder pin
void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

//Rotary encoder interrupt service routine for the other encoder pin
void PinB(){
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

void EEPROMWriteInt(int address, int value)
{
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);
  
  EEPROM.update(address, two);
  EEPROM.update(address + 1, one);
}
 
int EEPROMReadInt(int address)
{
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);
 
  return ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);
}


//----------------------------------Solder Station code---------------------------------------
void setPwmFrequency(int pin, int divisor)
{
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    switch (divisor)
    {
    case 1:
      mode = 0x01;
      break;
    case 8:
      mode = 0x02;
      break;
    case 64:
      mode = 0x03;
      break;
    case 256:
      mode = 0x04;
      break;
    case 1024:
      mode = 0x05;
      break;
    default:
      return;
    }

    if (pin == 5 || pin == 6)
    {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else
    {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11)
  {
    switch (divisor)
    {
    case 1:
      mode = 0x01;
      break;
    case 8:
      mode = 0x02;
      break;
    case 32:
      mode = 0x03;
      break;
    case 64:
      mode = 0x04;
      break;
    case 128:
      mode = 0x05;
      break;
    case 256:
      mode = 0x06;
      break;
    case 1024:
      mode = 0x07;
      break;
    default:
      return;
    }

    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

// end of sketch!
