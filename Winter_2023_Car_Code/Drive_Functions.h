int calculate_digi_pot(int accel) {
  //converts the acceleration potentionmeter value (between 475 and 875) to a 0-255 range for SPI, and flips it because of the potentiometer mounting
  if(accel < 50) {
    accel = ACCEL_ZERO_POSITION;
  }
  int calculated_digipot = 0;
  calculated_digipot = int(map(accel, ACCEL_MAX_POSITION, ACCEL_ZERO_POSITION, 255, 0));

  //truncate the value to between 0 and 255
  calculated_digipot = max(calculated_digipot, 0);
  calculated_digipot = min(calculated_digipot, 255);

  return calculated_digipot;
}
