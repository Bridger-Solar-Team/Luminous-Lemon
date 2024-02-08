#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

#define CHIP 0x40       // The chip's address (set by pins 4 & 5)
#define CHIP_READ 0x41
#define IO_DIR_REG 0x00 // The Input/Output Register
#define GPIO_REG 0x09   // The GPIO Register
#define GPPU_REG  0x06

#define EX_PIN 9 
#define POT_PIN 10
#define AccPotSig A6
#define BrakeSig 6
#define CONTACTOR_OUT 17
#define CRUISE_PIN 7

byte PortExByte;

float accel_pot_raw = 0;
bool hazard_val = 1;
bool brake_raw = 0;
int digi_pot_val = 0;
int throttle_percent = 0;
bool main_power = 0;
bool cruise_val = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  spi_setup();
  pinMode(CONTACTOR_OUT, OUTPUT);
  digitalWrite(CONTACTOR_OUT, LOW);
}

void loop() {
  read_inputs();
  move_car();
  // run_lights();
  // update_display();
  // send_telemetry();
  Serial.println();
}

void read_inputs() {
  //Read port expander
  PortExByte = SPIRead(GPIO_REG);

  //takes in signal from potentionmeter on steering wheel
  accel_pot_raw = analogRead(AccPotSig); 
  Serial.print("Acceleration pot: ");
  Serial.print(accel_pot_raw);
  Serial.print(" ");

  cruise_val = digitalRead(CRUISE_PIN);

  Reads Main Power switch, In this case Spare1 on the Interface Control Board
  if((PortExByte & 0b01000000)==0b01000000){
    main_power = 1;
    Serial.print("Main Power On ");
  }
  else{
    main_power = 0;
    Serial.print("Main Power Off ");
  } 

  //Reads the brake signal, high if brake is pressed(i assume) - Tristan, 12/6/2023
  brake_raw = digitalRead(BrakeSig);
  if(brake_raw) {
    Serial.print("Brake Engaged ");
  }
}

void move_car() {
  //Turn on contactors if main power switch is on
  if(main_power) {
    digitalWrite(CONTACTOR_OUT, HIGH);
    Serial.print("Contactors Powered! ");
  } else {
    digitalWrite(CONTACTOR_OUT, LOW);
    Serial.print("Contactors Unpowered ");
  }

  if(!brake_raw){
    digi_pot_val = calculate_digi_pot(accel_pot_raw);
    DigiPotWrite(digi_pot_val);//writes acceleration value value to digital pot
  }
  Serial.print("DigiPot value: ");
  Serial.print(digi_pot_val);
  Serial.print(" ");
}

int calculate_digi_pot(float accel) {
  //brings steering pot value to an int between 0 and 255, adjusts input for limited range of pot. should be between 475 and 875
  digi_pot_val = int(round(((accel_pot_raw-475)/400)*255));
  if(digi_pot_val < 0){
    digi_pot_val = 0;
  }
  if(digi_pot_val > 255){
    digi_pot_val = 255;
  }

  //flips the value, otherwise the car would be at full throttle when the throttle is released
  digi_pot_val = abs(digi_pot_val- 255);

  return digi_pot_val;
}

void DigiPotWrite(int value){
    //writes to the  Digipot
    digitalWrite(POT_PIN,LOW);
    SPI.transfer(0);
    SPI.transfer(value);
    digitalWrite(POT_PIN,HIGH);
}

byte SPIRead(byte address){
  digitalWrite(EX_PIN,LOW);
  SPI.transfer(CHIP_READ);
  SPI.transfer(address);
  byte retVal = SPI.transfer(0x00);
  digitalWrite(EX_PIN,HIGH);
  return retVal;
}

void spi_setup() {
  //DigiPot SPI setup
  pinMode(POT_PIN,OUTPUT);
  digitalWrite(POT_PIN,HIGH);

  //Port expander SPI setup
  pinMode(EX_PIN, OUTPUT);
  digitalWrite(EX_PIN,HIGH);
  delay(100);
  SPI.begin();
  SPIWrite(IO_DIR_REG,0b11111110); // Set pins 1-7 as INPUT, pin 0 as OUTPUT
  SPIWrite(GPIO_REG,0x00);         // Sets pin 0 to low
  SPIWrite(GPPU_REG,0b11111110);   //turns on internal pullups for all pins but 0
}

void SPIWrite(byte spiRegister, byte value){
  digitalWrite(EX_PIN,LOW);
  SPI.transfer(CHIP);
  SPI.transfer(spiRegister);
  SPI.transfer(value);
  digitalWrite(EX_PIN,HIGH);
}