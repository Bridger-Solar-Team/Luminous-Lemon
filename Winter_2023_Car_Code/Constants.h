//Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP23008-MCP23S08-Data-Sheet-20001919F.pdf
#define CHIP_WRITE 0x40       // The chip's address (set by pins 4 & 5)
#define CHIP_READ 0x41 
#define IO_DIR_REG 0x00 // The Input/Output Register (datasheet page 9)
#define GPIO_REG 0x09   // The GPIO Register (datasheet page 18)
#define GPPU_REG  0x06 //The pull-ups register (datasheet page 15)
#define IO_CON_REG 0x05 //The configuration register (datasheet page 14)

#define EX_PIN 9 
#define POT_PIN 10
#define ACC_POT_PIN A6
#define BRAKE_PIN 6
#define CONTACTOR_OUT 17
#define CRUISE_PIN 7
#define LEFT_TURN_SIGNAL_PIN 4
#define RIGHT_TURN_SIGNAL_PIN 5
#define DISPLAY_TOG_SIGNAL_PIN 8

#define ACCEL_ZERO_POSITION 350
#define ACCEL_MAX_POSITION 170