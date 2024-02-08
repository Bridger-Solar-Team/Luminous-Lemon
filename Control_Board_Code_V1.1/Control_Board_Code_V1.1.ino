#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

#define CHIP 0x40       // The chip's address (set by pins 4 & 5)
#define CHIP_READ 0x41
#define IO_DIR_REG 0x00 // The Input/Output Register
#define GPIO_REG 0x09   // The GPIO Register
#define GPPU_REG  0x06

byte PortExByte;



int RightOut = 1,ContactorOut = 17,BPLOut = 0;
int ExPin = 9, PotPin = 10;

int  RightSig =5, BPSig = 2,  LeftSig = 4,DisplayTogSig = 8,CruiseSig = 7,AccPotSig = A6,DDCSig = A0,DOCSig = A1,SOCSig = A2,ConverterSig = A7,CurSpeedSig = 3,BrakeSig = 6; 
int RightVal = 0,LeftVal  = 0, HazardVal = 0, DisplayTogVal = 0,CruiseVal = 0,DigiPotVal = 0,DDCVal = 0, DOCVal = 0, SOCVal = 0,CurSpeedVal = 0,PulseFreq = 0;
int MainPowerVal = 0,EmergencyVal = 1,BrakeVal = 0;
float AccPotFloat = 0, DDCFloat = 0, DOCFloat = 0, SOCFloat = 0,ConverterFloat = 0,SpeedFloat = 0,ClockSpeed = 15625,SpeedPeriodDiff = 0,ConvFactor = 17.6,DigiPotFloat = 0;
int DisplayTogRecord = 0;
volatile int SpeedPeriodCur = 0, SpeedPeriodPrev = 0,LeftPrev = 0,RightCur = 0,LeftCur = 0,BPSCur = 1,ThrottlePercent = 0;
int  WheelCirc = 66,PulseNum = 16,CruiseHold = 0,ThrottleHold = 0,CruiseStop = 0,BrakeHold = 0;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);


void setup() {
  Serial.begin(9600);
  pinMode(RightOut, OUTPUT);
  pinMode(BPLOut, OUTPUT);
  pinMode(ContactorOut, OUTPUT);
//input setup

  pinMode(DisplayTogSig,INPUT);
  pinMode(LeftSig, INPUT);
  pinMode(RightSig, INPUT);
  pinMode(CurSpeedSig, INPUT_PULLUP);
  pinMode(RightSig, INPUT);
  pinMode(BrakeSig, INPUT);

//initial output values
  digitalWrite(RightOut, LOW);
  digitalWrite(BPLOut, LOW);
  digitalWrite(ContactorOut, LOW);
  
  //SPI Setup
  pinMode(PotPin,OUTPUT);
  digitalWrite(PotPin,HIGH);
  pinMode(ExPin, OUTPUT);
  digitalWrite(ExPin,HIGH);
  SPI.begin();
  SPIWrite(IO_DIR_REG,0b11111110); // Set pins 1-7 as input, pin 0 as input
  SPIWrite(GPIO_REG,0x00);         // Sets pin 0 to low
  SPIWrite(GPPU_REG,0b11111110);   //turns on internal pullups for all pins but 0

//Timer Setup 
  cli();//stop intturupts
//Timer inturrupt setup for blinking lights 
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR1A = 10000;//sets time to go off every .2 seconts, as per formula sun regulations
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS01 and CS00 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
//timer setup for measuring vehicle speed
  sei();//allow interrupts



  //lcd setup
  lcd.init();
  lcd.backlight();
  Wire.setClock(60000);//lowers baud rate to reduce interference over long wires, should not go below 50000

    //Hardware Inturrupt Setup
  attachInterrupt(digitalPinToInterrupt(CurSpeedSig),SpeedRead,FALLING);
  pinMode(BPSig,INPUT);
  //attachInterrupt(digitalPinToInterrupt(BPSig),ESTOP,FALLING);


 }

void loop() {
  ReadAccPot();
  InputCheck();
  DigitalOut();
  LCD();
  CalculateSpeed();
}

void CalculateSpeed(){
  SpeedPeriodDiff = abs(SpeedPeriodCur-SpeedPeriodPrev);
  CurSpeedVal = int(((WheelCirc/PulseNum)/(SpeedPeriodDiff/ClockSpeed))/ConvFactor);
  if(CurSpeedVal < 3) CurSpeedVal = 0;
}

//ISR that iturrupts on falling edge of every pulse from motor controller, records timer value to caculate speed
void SpeedRead(){
  SpeedPeriodPrev = SpeedPeriodCur;
  SpeedPeriodCur = TCNT1;
}
//ISR pin fuction, runs if if Inturrupt pin D2 is pulled high
void ESTOP(){
  Emergency();
}
//Emergency  function that runs in event of battery issue. Disconnects Main Battery Pack and switches low voltage electronics to backup battery
void Emergency(){
  EmergencyVal = 0;  
//  digitalWrite(ContactorOut, LOW);//Turns off contactor, board must be reset to turn contactor back on  

}

void DigitalOut(){
  if(LeftCur == 1){
    SPIWrite(GPIO_REG, 0x01);
  }
  else{
    SPIWrite(GPIO_REG,0x00);
  }
  digitalWrite(RightOut,RightCur);
  digitalWrite(BPLOut,BPSCur);
  if(EmergencyVal == 1){
    digitalWrite(ContactorOut,MainPowerVal);//toggles system connection to main battery pack
  }
  else{
    digitalWrite(ContactorOut,LOW);
  }


}

void InputCheck(){
  LeftVal = digitalRead(LeftSig);
  BrakeVal = digitalRead(BrakeSig);
  PortExByte = SPIRead(GPIO_REG);//Reads all values of port expander
  if((PortExByte & 0b10000000) == 0b10000000){//Reads hazard value
    HazardVal = 1;
  }
  else{
    HazardVal = 0;
  }
  if((PortExByte & 0b01000000)==0b01000000){//Reads Main Power switch, In this case Spare1 on the Interface Control Board
    MainPowerVal = 1;
    Serial.println(PortExByte);
  }
  else{
    MainPowerVal = 0;
    Serial.println(PortExByte);
  } 
  DisplayTogVal = digitalRead(DisplayTogSig);
  RightVal = digitalRead(RightSig);
  CruiseVal = digitalRead(CruiseSig);
  
  if(BrakeVal == 1) CruiseStop = 1;
  else if(CruiseVal == 1)CruiseStop = 0;
  if(CruiseStop == 1)CruiseVal = 1; //these three if statements turn of Cruise control if break is applied, and does not turn back on until Cruise switch is reset
  EmergencyVal = digitalRead(BPSig);

}

void DigiPotWrite(int value){
    //writes to the  Digipot
    digitalWrite(PotPin,LOW);
    SPI.transfer(0);
    SPI.transfer(value);
    digitalWrite(PotPin,HIGH);
}

void ReadAccPot(){
  AccPotFloat = analogRead(AccPotSig); //takes in signal from pot on steering wheel
  DigiPotVal = int(round(((AccPotFloat-475)/400)*255));//brings steering pot value to an int between 0 and 255, adjusts input for limited range of pot. should be between 475 and 875
  if(DigiPotVal < 0){
    DigiPotVal = 0;
  }
  if(DigiPotVal > 255){
    DigiPotVal = 255;
  }
  DigiPotVal = abs(DigiPotVal- 255);//flips the value, otherwise the car would be at full throttle when the throttle is released
  DigiPotFloat = DigiPotVal;// probably could have casted to float in below eq, but I dont want to mess with casting in C if I dont have to
  ThrottlePercent = int(100 * (DigiPotFloat/255));//Determines throttle percentage to be sent to display
  if(CruiseVal == LOW){
    DigiPotVal = CruiseHold;
    ThrottlePercent = ThrottleHold;
  }
  else{
    CruiseHold = DigiPotVal;
    ThrottleHold = ThrottlePercent;
  }
  DigiPotWrite(DigiPotVal);//writes acceleration value value to digital pot
}

void SPIWrite(byte spiRegister, byte value){
  digitalWrite(ExPin,LOW);
  SPI.transfer(CHIP);
  SPI.transfer(spiRegister);
  SPI.transfer(value);
  digitalWrite(ExPin,HIGH);
}
byte SPIRead(byte address){
  digitalWrite(ExPin,LOW);
  SPI.transfer(CHIP_READ);
  SPI.transfer(address);
  byte retVal = SPI.transfer(0x00);
  digitalWrite(ExPin,HIGH);
  return retVal;
}

ISR(TIMER1_COMPA_vect){
  
    if(HazardVal == HIGH){  
      if(LeftVal == HIGH){
        LeftCur = 0;  //I do this because aruduino does not seem to like writing Via spi in a timer inturrupt, switch lights in DigitalOut instead
      }
      else{
        LeftCur = !LeftCur;
      }
      if(RightVal == HIGH){
        RightCur = 0;
      }
      else{
        RightCur = !RightCur;
      }
    }
    else{
      if(LeftCur != RightCur){
        LeftCur = 0;  //ensures lights cant alternate flashing while hazards are engaged
        RightCur = 0;
      }
      LeftCur = !LeftCur;
      RightCur = !RightCur;
    }
    if(EmergencyVal == HIGH){
      BPSCur = 0;
    }
    else{
      BPSCur = !BPSCur;
    }
}
void LCD(){
  if(DisplayTogVal == HIGH){
    //clears display if button was just hit
    if(DisplayTogRecord == LOW){
      lcd.clear();
      DisplayTogRecord = HIGH;
    }
    lcd.setCursor(0, 0); 
    lcd.print("Speed:");
    lcd.print(CurSpeedVal);
    if(CurSpeedVal < 10) lcd.print(" ");
    if(CurSpeedVal < 100) lcd.print(" "); 
    
    lcd.setCursor(9,0);
    lcd.print("Crs:");
    if(CruiseVal == 0) lcd.print("On ");
    else lcd.print("Off");
    lcd.setCursor(0,1);
    
    lcd.print("Vltg:");
    lcd.print(DigiPotVal);
    if(DigiPotVal < 100) lcd.print(" ");
    if(DigiPotVal<10)lcd.print(" ");//if statements ensure that all blank characters are clared from display
    
    lcd.setCursor(8,1);
    lcd.print("Acc%:");
    lcd.print(ThrottlePercent);
    if(ThrottlePercent < 100) lcd.print(" ");
    if(ThrottlePercent < 10)lcd.print(" ");//if statements ensure that all blank characters are clared from display
  }
  else{
    //clears display if button was just hit 
    if(DisplayTogRecord == HIGH){
      lcd.clear();
      DisplayTogRecord = LOW;
    }
    lcd.setCursor(0, 0);//prints DOC value to lcd 
    lcd.print("DOC:"); 
    lcd.print(DOCVal);
    if(DOCVal<100)lcd.print(" ");
    if(DOCVal<10) lcd.print(" ");//if statements ensure that all blank characters are clared from display
    lcd.print("%");
    
    lcd.setCursor(8, 0); //prints DDC value to LCD
    lcd.print("DDC:");
    lcd.print(DDCVal);
    if(DDCVal<100)lcd.print(" ");
    if(DDCVal<10) lcd.print(" ");//if statements ensure that all blank characters are clared from display
    lcd.print("%");
    
    lcd.setCursor(0,1); //prints SOC value to LCD
    lcd.print("SOC:");
    lcd.print(SOCVal);
    if(SOCVal<100)lcd.print(" ");
    if(SOCVal<10) lcd.print(" ");//if statements ensure that all blank characters are clared from display
    lcd.print("%");
  }
}
