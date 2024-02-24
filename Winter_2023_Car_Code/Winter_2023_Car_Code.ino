#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "Constants.h"
#include "SPI_Functions.h"
#include "Drive_Functions.h"
#include "LCD_Functions.h"
#include "Lights.h"

bool ENABLE_DEBUGGING = 0;

byte PortExByte;

float accel_pot_raw = 0;

int digi_pot_val = 0;
int throttle_percent = 0; 
float soc = 1;

bool brake_pressed = 0;
bool main_power = 0;
bool cruise_control = 0;
bool hazard_pressed = 0;
bool display_toggle = 0;

bool right_turn = 0;
bool left_turn = 0;
bool display_tog_record = 0;
bool light_state = 1;

unsigned long flash_timing = 0;

//Using buffers to print to lcd for faster response times
String line0;
String line1;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);

void setup() {
  //Begin debugging serial port only if debugging(disables pin 0 and 1)
  if(ENABLE_DEBUGGING) {
    Serial.begin(9600);
  }
  //Begin the SPI for port expander and digital potentiometer
  spi_setup();
  //Safeguard the contactor states on first power-up
  pinMode(CONTACTOR_OUT, OUTPUT);
  digitalWrite(CONTACTOR_OUT, LOW);
  pinMode(LEFT_TURN_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(RIGHT_TURN_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(CRUISE_PIN, INPUT_PULLUP);
  //pinMode(DISPLAY_TOG_SIGNAL_PIN, INPUT); Break in the wire somewhere, not sure if need pullup
  pinMode(BATT_POWER_LIGHT, OUTPUT);
  digitalWrite(BATT_POWER_LIGHT, LOW);
  pinMode(ESTOP_PIN, INPUT);
  pinMode(SOG_SIGNAL_PIN, INPUT);

  lcd_setup();
  
  //Motor Speed Read
  pinMode(WHEEL_SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_SPEED_PIN),SpeedRead,FALLING);

}

void lcd_setup() {
  //lcd setup
  lcd.init();
  lcd.backlight();
  Wire.setClock(60000);//lowers baud rate to reduce interference over long wires, should not go below 50000
  lcd.clear();
}

void loop() {
  read_inputs(); //TODO(debug): hazzard & display_toggle
  move_car();
  run_lights();
  update_display();
  // send_telemetry();
  if(ENABLE_DEBUGGING) {
    debug(); //Comment this out to disable printing debug values
  }
}

void update_display() {
  line0 = "";
  line1 = "";
  //Turn signals, right, left, hazard. hazard overrides right and left
  line0 += "T";
  if(hazard_pressed) {
    line0 += "H";
  }
  else if(right_turn) {
    line0 += "R";
  } else if(left_turn) {
    line0 += "L";
  } else {
    line0 += "0";
  }

  //Digital potentiometer output, aka motor power. 0-255 value
  line0 += " M";
  line0 += (digi_pot_val/100)%10; //Hundreds place
  line0 += (digi_pot_val/10)%10; //Tens place
  line0 += digi_pot_val%10; //Singles place
  
  //Cruise control switch. 0 when out, 1 when in
  line0 += " C";
  if(cruise_control) {
    line0 += "1";
  }
  else {
    line0 += "0";
  }

  //Main power switch. 0 when off, 1 when on
  line0 += " P";
  if(main_power) {
    line0 += "1";
  }
  else {
    line0 += "0";
  }

  //Brake pedal. 0 when not braking, 1 when braking
  line0 += " B";
  if(brake_pressed) {
    line0 += "1";
  }
  else {
    line0 += "0";
  }
  //End of the top row

  //State of charge. 0-99%
  line1 += "V";
  line1 += (round(soc*99)/10)%10;
  line1 += round(soc*99)%10;
  line1 += "%";

  //Speed, 0-99 miles per hour
  line1 += " MPH";
  line1 += (CurSpeedVal/10)%10;
  line1 += CurSpeedVal%10;

  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

void debug() {
  Serial.print(F("Acceleration pot: "));
  Serial.print(accel_pot_raw);
  Serial.print(" ");

  if(main_power) {
    Serial.print(F("Main_Power_On "));
  }
  else {
    Serial.print(F("Main_Power_Off "));
  }

  if(brake_pressed) {
    Serial.print(F("Brake_Engaged "));
  }
  
  if(right_turn) {
    Serial.print(F("Turning_Right! "));
  }

  if(left_turn) {
    Serial.print(F("Turning_Left! "));
  }
  
  if(cruise_control) {
    Serial.print(F("Cruizin! "));
  }
  
  if(main_power) {
    Serial.print(F("Contactors Powered! "));
  } else {
    Serial.print(F("Contactors Unpowered "));
  }

  if(display_toggle) {
    Serial.print(F("get_TOGGLED! "));
  }
  
  if(hazard_pressed) {
    Serial.print(F("Hazards! "));
  } 
  
  Serial.print(F("Digipot: "));
  Serial.print(digi_pot_val);
  Serial.print(F(". "));

  Serial.print(" Lighting: ");
  Serial.print(light_state);

  Serial.print("SOC: ");
  Serial.print(soc);

  Serial.print(" line0:");
  Serial.print(line0);
  Serial.print(" line1:");
  Serial.print(line1);
  Serial.println();
}

void run_lights() {
  flash();
  if(!main_power && light_state) {
    batt_light_on();
    hazard_light_on();
    return;
  }
  else if(!main_power && !light_state) {
    batt_light_off();
    turn_signal_lights_off();
    return;
  } else {
    batt_light_off();
  }
  
  if(hazard_pressed) {
    if(light_state) {
      hazard_light_on();
    }
    else {
      turn_signal_lights_off();
    }
  } 
  else if(left_turn){;    
    if(light_state) {
      left_turn_lights_on();
    } 
    else {
      turn_signal_lights_off();
    }
  }
  else if(right_turn) {    
    if(light_state) {
      right_turn_lights_on();
    } 
    else {
      turn_signal_lights_off();
    }
  }
  else {
    turn_signal_lights_off();
  }

}

void flash() {
  if(millis()-flash_timing > TURN_SIGNAL_MILLIS) {
    light_state = !light_state;
    flash_timing = millis();
  }
}

void read_inputs() {
  //Read port expander (Fixed)
  PortExByte = SPIRead(GPIO_REG);

  //takes in signal from potentionmeter on steering wheel
  accel_pot_raw = analogRead(ACC_POT_PIN); 

  // Reads Main Power switch, WHICH DOES NOT HAVE A CURRENT PORT AVAILABLE SINCE D1 BROKE
  if((PortExByte & 0b00000010)==0b00000010){
    main_power = 0;
  }
  else if(digitalRead(ESTOP_PIN)) {
    main_power = 1;
  }
  else {
    main_power = 0;
  } 

  //Reads the brake signal, high if brake is pressed
  brake_pressed = digitalRead(BRAKE_PIN);

  //Left and right turn signals
  if(!digitalRead(RIGHT_TURN_SIGNAL_PIN)) {
    right_turn = 1;
    left_turn = 0;
  }
  else if(!digitalRead(LEFT_TURN_SIGNAL_PIN)) {
    left_turn = 1;
    right_turn = 0;
  }
  else {
    right_turn = 0;
    left_turn = 0;
  }

  //Cruise control button (it's inverted, so out is true and in is false)
  cruise_control = !digitalRead(CRUISE_PIN);

  // TODO (Does not work, there is a break in the wire)
  display_toggle = digitalRead(DISPLAY_TOG_SIGNAL_PIN);

  if((PortExByte & 0b10000000) == 0b10000000){//Reads hazard value 
    hazard_pressed = 1;
  } 
  else {
    hazard_pressed = 0;
  }

  soc = analogRead(SOG_SIGNAL_PIN)/1023.0;
  
  calculate_speed();
}

void move_car() {
  //Turn on contactors if main power switch is on and BMS is not faulted
  if(main_power) {
    digitalWrite(CONTACTOR_OUT, HIGH);
  } else {
    digitalWrite(CONTACTOR_OUT, LOW);
  }

  //only move if we arent pressing the break
  if(!brake_pressed){
    digi_pot_val = calculate_digi_pot(accel_pot_raw);
    DigiPotWrite(digi_pot_val); //writes acceleration value to digital potentiometer
  }
}