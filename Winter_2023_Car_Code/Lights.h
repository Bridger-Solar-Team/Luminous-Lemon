void batt_light_on() {
  digitalWrite(BATT_POWER_LIGHT, HIGH);
}

void batt_light_off() {
  digitalWrite(BATT_POWER_LIGHT, LOW);
}

void hazard_light_on() {
  SPIWrite(GPIO_REG, 0b00000001);
}

void turn_signal_lights_off() {
  SPIWrite(GPIO_REG,0b01000000);
}

void left_turn_lights_on() {
  SPIWrite(GPIO_REG, 0b01000001);
}

void right_turn_lights_on() {
  SPIWrite(GPIO_REG, 0b00000000);
}