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

bool right_turn = 0;
bool left_turn = 0;
bool display_tog_record = 0;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);

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
  //pinMode(DISPLAY_TOG_SIGNAL_PIN, INPUT); Break in the wire somewhere, not sure if need pullup
  pinMode(RIGHT_TURN_LIGHT_PIN, OUTPUT);
  digitalWrite(RIGHT_TURN_LIGHT_PIN, LOW);

  lcd_setup();
}

void lcd_setup() {
  //lcd setup
  lcd.init();
  lcd.backlight();
  Wire.setClock(60000);//lowers baud rate to reduce interference over long wires, should not go below 50000
}

void loop() {
  read_inputs(); //TODO(debug): hazzard & display_toggle
  move_car();
  // run_lights();
  update_display();
  // send_telemetry();
  debug(); //Comment this out to disable printing debug values
}

void update_display() {
  if(display_tog_raw){
    //clears display if button was just hit
    if(display_tog_record == LOW){
      lcd.clear();
      display_tog_record = HIGH;
    }
    lcd.setCursor(0, 0);

    //Turn signals, right, left, hazard. hazard overrides right and left
    lcd.print("T");
    if(hazard_raw) {
      lcd.print("H");
    }
    else if(right_turn) {
      lcd.print("R");
    } else if(left_turn) {
      lcd.print("L");
    } else {
      lcd.print("0");
    }

    //Digital potentiometer output, aka motor power. 0-255 value
    lcd.print(" M");
    if(digi_pot_val < 100){
      lcd.print("0");
    }
    if(digi_pot_val < 10) {
      lcd.print("0");
    }
    lcd.print(digi_pot_val);
    
    //Cruise control switch. 0 when out, 1 when in
    lcd.print(" C");
    if(cruise_raw) {
      lcd.print("0");
    }
    else {
      lcd.print("1");
    }

    //Main power switch. 0 when off, 1 when on
    lcd.print(" P");
    if(main_power) {
      lcd.print("1");
    }
    else {
      lcd.print("0");
    }

    //Brake pedal. 0 when not braking, 1 when braking
    lcd.print(" B");
    if(brake_raw) {
      lcd.print("1");
    }
    else {
      lcd.print("0");
    }

  }
  else {
    //clears display if button was just hit 
    if(display_tog_record == HIGH){
      lcd.clear();
      display_tog_record = LOW;
    }
  }
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
  
  if(right_turn) {
    Serial.print("Turning_Right! ");
  }

  if(left_turn) {
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

  Serial.println();
}

void run_lights() {
  //Neither turn signal work
    if(left_turn){
    SPIWrite(GPIO_REG, 0x01);
  }
  else{
    SPIWrite(GPIO_REG,0x00);
  }

  if(right_turn) {
    digitalWrite(RIGHT_TURN_LIGHT_PIN, HIGH);
  }
  else {
    digitalWrite(RIGHT_TURN_LIGHT_PIN, LOW);
  }

}

void read_inputs() {
  //Read port expander (Fixed)
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

  //Reads the brake signal, high if brake is pressed
  brake_raw = digitalRead(BRAKE_PIN);

  //Left and right turn signals
  left_turn_raw = digitalRead(LEFT_TURN_SIGNAL_PIN);
  right_turn_raw = digitalRead(RIGHT_TURN_SIGNAL_PIN);

  if(!right_turn_raw) {
    right_turn = 1;
  }
  else if(!left_turn_raw) {
    left_turn = 1;
  }
  else {
    right_turn = 0;
    left_turn = 0;
  }

  //Cruise control button (it's inverted, so out is true and in is false)
  cruise_raw = digitalRead(CRUISE_PIN);

  // TODO (Does not work, there is a break in the wire)
  display_tog_raw = digitalRead(DISPLAY_TOG_SIGNAL_PIN);

  if((PortExByte & 0b10000000) == 0b10000000){//Reads hazard value 
    hazard_raw = 1;
  } 
  else {
    hazard_raw = 0;
  }
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
    Serial.print("Digipot: ");
    Serial.print(digi_pot_val);
    Serial.print(". ");
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
  // SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(POT_PIN,LOW);
  SPI.transfer(0);
  SPI.transfer(value);
  digitalWrite(POT_PIN,HIGH);
  // SPI.endTransaction();
}

byte SPIRead(byte address){
  // SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(EX_PIN,LOW);
  SPI.transfer(CHIP_READ);
  SPI.transfer(address);
  byte retrieved_Val = SPI.transfer(0x00);
  digitalWrite(EX_PIN,HIGH);
  // SPI.endTransaction();
  Serial.print("Expander: ");
  Serial.print(retrieved_Val);
  Serial.print(". ");
  return retrieved_Val;
}

void SPIWrite(byte spiRegister, byte value){
  // SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(EX_PIN,LOW);
  SPI.transfer(CHIP_WRITE);
  SPI.transfer(spiRegister);
  SPI.transfer(value);
  digitalWrite(EX_PIN,HIGH);
  // SPI.endTransaction();
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
  //SPIWrite(IO_CON_REG, 0b00111110); //This is an untested fix for the port expander always returning 0 (11am feb 15th 2024)
  SPIWrite(IO_DIR_REG, 0b11111110); // Set pins 1-7 as INPUT, pin 0 as OUTPUT
  SPIWrite(GPIO_REG, 0x00);         // Sets pin 0 to low
  SPIWrite(GPPU_REG, 0b11111110);   //turns on internal pullups for all pins but 0
}