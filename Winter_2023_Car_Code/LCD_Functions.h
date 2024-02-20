void lcd_setup(LiquidCrystal_I2C lcd) {
  //lcd setup
  lcd.init();
  lcd.backlight();
  Wire.setClock(60000);//lowers baud rate to reduce interference over long wires, should not go below 50000
}