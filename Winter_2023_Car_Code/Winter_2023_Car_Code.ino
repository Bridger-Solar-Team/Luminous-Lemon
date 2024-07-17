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
int cruise_speed = 0;
int ccl_raw = 0;
int dcl_raw = 0;
int ccl_fault = 0;
int dcl_fault = 0;
int dcl_fault_max;
int ccl_fault_max;

bool estop_raw = 0;
bool ccl_last_loop = 0;
bool dcl_last_loop = 0;
bool true_ccl_fault;
bool true_dcl_fault;
bool true_fault;
bool brake_pressed = 0;
bool main_power = 0;
bool cruise_control = 0;
bool hazard_pressed = 0;
bool display_toggle = 0;
bool cruise_toggle = 1;
bool fault = 0;
bool power_switch = 0;

bool right_turn = 0;
bool left_turn = 0;
bool display_tog_record = 0;
bool light_state = 1;

unsigned long flash_timing = 0;
unsigned long curr_loop_time = 0;
unsigned long prev_loop_time = 0;
int loop_time = 10;

//Using buffers to print to lcd for faster response times
String line0;
String line1;
String old_line0;
String old_line1;

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
  Serial.println("before lcd init, ");
  lcd.init();
  Serial.println("after lcd init");
  lcd.backlight();
  Wire.setClock(60000);//lowers baud rate to reduce interference over long wires, should not go below 50000
  lcd.clear();
}

void loop() {
  prev_loop_time = curr_loop_time;
  curr_loop_time = millis();
  loop_time = curr_loop_time-prev_loop_time;
  read_inputs(); //TODO(debug): display_toggle(this is a wiring issue, needs to be fixed on car)
  move_car();
  run_lights();
  update_display();
  // send_telemetry();
  // log_data();
  if(ENABLE_DEBUGGING) {
    debug();
  }
}

void update_display() {
  //00MPH MTR56% CRS
  //V+87% PWR ON  T0
  
  old_line0 = line0;
  old_line1 = line1;
  line0 = "";
  line1 = "";

  //Speed, 0-99 miles per hour
  line0 += (CurSpeedVal/10)%10; //1
  line0 += CurSpeedVal%10; //2
  line0 += "MPH "; //3,4,5,6

  //Brake pedal. BRAKE when braking, motor percent power otherwise
  if(brake_pressed) {
    line0 += "BRAKE!"; //7,8,9,10,11,12
  }
  else {
    if(dcl_raw < 255) {
      line0 += "DCL";
    } else if(ccl_raw < 255) {
      line0 += "CCL";
    } else {
      line0 += "MTR"; //7,8,9
    }
    //0-99 motor power as an integer for no decimals
    int motor_power;
    if(cruise_control){
      motor_power = min(round((cruise_speed/255.0)*100),99);
    } else {
      motor_power = min(round((digi_pot_val/255.0)*100),99);
    }
    line0 += (motor_power/10)%10; //10
    line0 += motor_power%10; //11
    line0 += "%"; //12
  }

  //Cruise control switch. 0 when out, 1 when in
  if(cruise_control) {
    line0 += " CRS"; //13,14,15,16
  }
  else if(true_ccl_fault) {
    line0 += " CCL"; //13, 14, 15, 16
  }
  else if(true_dcl_fault) {
    line0 += " DCL"; //13,14,15,16
  }
  else if (true_dcl_fault || true_ccl_fault) {
    line0 += " BTH"; //13,14,15,16
  }
  else if(true_fault) {
    line0 += " BPS";
  }
  else {
    line0 += " N/A"; //13,14,15,16
  }
  //End of the top row

  //State of charge. 0-99%
  line1 += "V+"; //1,2
  line1 += (round(soc*99)/10)%10; //3
  line1 += round(soc*99)%10; //4
  line1 += "% "; //5, 6

  //Main power switch. PWR ON when on, PWROFF when off
  if(fault) {
    line1 += "FAULT "; //7, 8, 9, 10, 11, 12
  }
  else if(main_power) {
    line1 += "PWR ON"; //7,8,9,10,11,12
  }
  else {
    line1 += "PWROFF"; //7,8,9,10,11,12
  }
  
  //Turn signals (T), right(R), left(L), hazard(H), and nothing(0). hazard overrides right and left
  line1 += "  T"; //13,14,15
  if(hazard_pressed) {
    line1 += "H"; //16
  }
  else if(right_turn) {
    line1 += "R"; //16
  }
  else if(left_turn) {
    line1 += "L"; //16
  }
  else {
    line1 += "0"; //16
  }

  //Only update the display if the data has changed
  if(old_line0 != line0){
    lcd.setCursor(0, 0);
    lcd.print(line0);
    old_line0 = line0;
  }

  //Only update the display if the data has changed
  if(old_line1 != line1) {
    lcd.setCursor(0, 1);
    lcd.print(line1);
    old_line1 = line1;
  }
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
    if(fault || estop_raw) {
      batt_light_on();
    }
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

bool flash() {
  if(millis()-flash_timing > TURN_SIGNAL_MILLIS) {
    light_state = !light_state;
    flash_timing = millis();
  }
}

void read_inputs() {
  //Read port expander
  PortExByte = SPIRead(GPIO_REG);

  //takes in signal from potentionmeter on steering wheel
  accel_pot_raw = analogRead(ACC_POT_PIN); 

  //CCL Pin
  ccl_raw = analogRead(A0);
  if(ccl_raw < CCL_LIMIT) {
    ccl_fault += 1;
    ccl_last_loop = 1;
  }
  
  if(!ccl_last_loop && !fault) {
    if(ccl_fault > ccl_fault_max) {
      ccl_fault_max = ccl_fault;
    }
    ccl_fault = 0;
  }

  //DCL Pin
  dcl_raw = analogRead(A1);
  if(dcl_raw < DCL_LIMIT) {
    dcl_fault += 1;
    dcl_last_loop = 1;
  }

  if(!dcl_last_loop && !fault){
    if(dcl_fault > dcl_fault_max) {
      dcl_fault_max = dcl_fault;
    }
    dcl_fault = 0;
  }

  if((PortExByte & 0b00000010)==0b00000010){
    power_switch = 0;
  } else {
    power_switch = 1;
  }

  if((PortExByte & 0b00001000)!=0b00001000){
    fault = 1;
    true_fault = 1;
  } else if(dcl_fault > FAULT_LOOP_TOLERANCE){
    fault = 1;
    true_dcl_fault = 1;
  } else if(ccl_fault > FAULT_LOOP_TOLERANCE) {
    fault = 1;
    true_ccl_fault = 1;
  } else {
    //fault = 0; removed for legality
  }

  estop_raw = !digitalRead(ESTOP_PIN);

  // Reads Main Power switch, and confirms ESTOP value
  if(!power_switch){
    //main power switch
    main_power = 0;
  }else if(fault){
    //bms fault
    main_power = 0;
  }
  else if(!estop_raw) {
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

  digi_pot_val = calculate_digi_pot(accel_pot_raw, dcl_raw);
  if(cruise_control) {
    if(brake_pressed){
      cruise_control = false;
      goto CRUISE_BREAK;
    }
    if(cruise_toggle) {
      cruise_speed = digi_pot_val;
      cruise_toggle = 0;
    }
    digi_pot_val = cruise_speed;
  }
  CRUISE_BREAK:
  if(!cruise_control) {
    if(!cruise_toggle){
      cruise_toggle = 1;
    }
  }

  //only move if we arent pressing the break
  if(!brake_pressed){
    DigiPotWrite(digi_pot_val); //writes acceleration value to digital potentiometer
  }
}