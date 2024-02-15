#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "Constants.h"

byte PortExByte;

float accel_pot_raw = 0;

int digi_pot_val = 0;
int throttle_percent = 0;

bool hazard_val = 0;
bool brake_raw = 0;
bool main_power = 0;
bool left_turn_raw = 0;
bool right_turn_raw = 0;
bool cruise_raw = 0;
bool hazard_raw = 0;
bool display_tog_raw = 0;


void setup() {
  //Begin debugging serial port
  Serial.begin(9600);
  //Begin the SPI for port expander and digital potentiometer
  spi_setup();
  //Safeguard the contactor states on first power-up
  pinMode(CONTACTOR_OUT, OUTPUT);
  digitalWrite(CONTACTOR_OUT, LOW);
  pinMode(LEFT_TURN_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(RIGHT_TURN_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(CRUISE_PIN, INPUT_PULLUP);
  pinMode(DISPLAY_TOG_SIGNAL_PIN, INPUT);
}

void loop() {
  read_inputs(); //TODO(debug): hazzard & display_toggle
  move_car();
  // run_lights();
  // update_display();
  // send_telemetry();
  debug();
}

void debug() {
  Serial.print("Acceleration pot: ");
  Serial.print(accel_pot_raw);
  Serial.print(" ");

  if(main_power) {
    Serial.print("Main_Power_On ");
  }
  else {
    Serial.print("Main_Power_Off ");
  }

  if(brake_raw) {
    Serial.print("Brake_Engaged ");
  }
  
  if(!right_turn_raw) {
    Serial.print("Turning_Right! ");
  }
  else if(!left_turn_raw) {
    Serial.print("Turning_Left! ");
  }
  
  if(!cruise_raw) {
    Serial.print("Cruizin! ");
  }
  
  if(main_power) {
    Serial.print("Contactors Powered! ");
  } else {
    Serial.print("Contactors Unpowered ");
  }

  if(display_tog_raw) {
    Serial.print("get_TOGGLED! ");
  }
  
  if(hazard_raw) {
    Serial.print("Hazards! ");
  } 
}

void read_inputs() {
  //Read port expander (Currently not working)
  PortExByte = SPIRead(GPIO_REG);

  //takes in signal from potentionmeter on steering wheel
  accel_pot_raw = analogRead(ACC_POT_PIN); 

  // Reads Main Power switch, In this case Spare1 on the Interface Control Board
  if((PortExByte & 0b01000000)==0b01000000){
    main_power = 1;
  }
  else {
    main_power = 0;
  } 

  //Reads the brake signal, high if brake is pressed(i assume) - Tristan, 12/6/2023
  brake_raw = digitalRead(BRAKE_PIN);

  //Left and right turn signals
  left_turn_raw = digitalRead(LEFT_TURN_SIGNAL_PIN);
  right_turn_raw = digitalRead(RIGHT_TURN_SIGNAL_PIN);

  //Cruise control button (it's inverted, so out is true and in is false)
  cruise_raw = digitalRead(CRUISE_PIN);

  //TODO (Does not work)
  // display_tog_raw = digitalRead(DISPLAY_TOG_SIGNAL_PIN);

  //TODO (Does Not Work)
  // if((PortExByte & 0b10000000) == 0b10000000){//Reads hazard value 
  //   hazard_raw = 1;
  // } 
  // else {
  //   hazard_raw = 0;
  // }
}

void move_car() {
  //Turn on contactors if main power switch is on
  if(main_power) {
    digitalWrite(CONTACTOR_OUT, HIGH);
  } else {
    digitalWrite(CONTACTOR_OUT, LOW);
  }

  if(!brake_raw){
    digi_pot_val = calculate_digi_pot(accel_pot_raw);
    DigiPotWrite(digi_pot_val);//writes acceleration value to digital potentiometer
  }
}

int calculate_digi_pot(float accel) {
  //converts the acceleration potentionmeter value (between 475 and 875) to a 0-255 range for SPI, and flips it because of the potentiometer mounting
  if(digi_pot_val < 50) {
    digi_pot_val = ACCEL_ZERO_POSITION;
  }
  digi_pot_val = int(map(accel_pot_raw, ACCEL_MAX_POSITION, ACCEL_ZERO_POSITION, 255, 0));

  //truncate the value to between 0 and 255
  digi_pot_val = max(digi_pot_val, 0);
  digi_pot_val = min(digi_pot_val, 255);

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
  byte retrieved_Val = SPI.transfer(0x00);
  digitalWrite(EX_PIN,HIGH);
  Serial.print("Expander: ");
  Serial.print(retrieved_Val);
  Serial.print(". ");
  return retrieved_Val;
}

void SPIWrite(byte spiRegister, byte value){
  digitalWrite(EX_PIN,LOW);
  SPI.transfer(CHIP_WRITE);
  SPI.transfer(spiRegister);
  SPI.transfer(value);
  digitalWrite(EX_PIN,HIGH);
}

void spi_setup() {
  //DigiPot SPI setup
  pinMode(POT_PIN,OUTPUT);
  digitalWrite(POT_PIN,HIGH);

  //Port expander SPI setup
  pinMode(EX_PIN, OUTPUT);
  digitalWrite(EX_PIN, HIGH);
  delay(100);                      //This may cause issues?
  SPI.begin();
  SPIWrite(IO_CON_REG, 0b00111110); //This is an untested fix for the port expander always returning 0 (11am feb 15th 2024)
  SPIWrite(IO_DIR_REG, 0b11111110); // Set pins 1-7 as INPUT, pin 0 as OUTPUT
  SPIWrite(GPIO_REG, 0x00);         // Sets pin 0 to low
  SPIWrite(GPPU_REG, 0b11111110);   //turns on internal pullups for all pins but 0
}