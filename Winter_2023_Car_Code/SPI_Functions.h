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
  delay(100);                      //This may cause issues, but it currently doesnt
  SPI.begin();
  //SPIWrite(IO_CON_REG, 0b00111110); //This is an untested fix for the port expander always returning 0 (11am feb 15th 2024)
  SPIWrite(IO_DIR_REG, 0b11111110); // Set pins 1-7 as INPUT, pin 0 as OUTPUT
  SPIWrite(GPIO_REG, 0x00);         // Sets pin 0 to low
  SPIWrite(GPPU_REG, 0b11111110);   //turns on internal pullups for all pins but 0
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

void DigiPotWrite(int value){
  //writes to the  Digipot
  // SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(POT_PIN,LOW);
  SPI.transfer(0);
  SPI.transfer(value);
  digitalWrite(POT_PIN,HIGH);
  // SPI.endTransaction();
}

