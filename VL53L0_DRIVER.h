/*
Drvier for the VL53L0 Time of Flight Sensor
Assumes default address - and that it is the only VL53L0 connected


Author: Joe Ingham

*/

#ifndef VL53L0_DRIVER
#define VL53L0_DRIVER

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif


//vl53l0 struct
typedef struct{

    //I2C hardware
    i2c_inst_t *I2C_HW;

    //I2C comms pins
    int SDA_PIN;
    int SCL_PIN;

    //Hardware enable pin
    int EN_PIN;

}vl53l0;

#define DEFAULT_BAUD 100000


//WRITE AND READ ADDRESS
#define VL53L0_ADDR 0x29

//Reference registers and there corresponding values 
//Identification
#define MODEL_ID 0xC0
#define EXPECTED_ID 0xEE


//I2C Mode
#define I2C_MODE 0x88
#define I2C_MODE_STANDARD 0x00


//2v8 mode
#define EXTSUP_2V8 0x89


//Power MGMT
#define POW_MGMT 0x80

//Internal tuning
#define INTERNAL_TUNING 0xFF


//Range settings (i.e. tell sensor to start the range then the mode)
#define VL53L0_SYSRANGE_START 0x00
#define VL53L0_SYS_RANGE_MODE_SINGLE 0x00
#define VL53L0_SYS_RANGE_MODE_CONTINUOUS 0x01

//Range interrupt status 
#define VL53L0_RANGE_RESULT_INTERRUPT_STATUS 0x13
#define SYSTEM_INTERRUPT_CLEAR 0x08

//Range status register (i.e. the reading)
#define VL53L0_RANGE_RESULT_STATUS 0x14







//Initialise the pins to connect with the vl53l0
vl53l0 init_vl53l0(int I2C_HW, int SDA_pin, int SCL_pin, int EN_pin);

//Setup the device to a default running state
int setup_default_config();


/*
Write a single byte to the VL53L0

Returns a success/fail int (1 - success, -1 - fail)

*/
int write_byte(vl53l0 *dev, uint8_t *byte);

/*
Read a single byte from the VL53L0
Return a success/fail if the byte is written
*/
int read_byte(vl53l0 *dev, uint8_t addr, uint8_t *buf);


/*
Read a single range measurement from the device
*/
int read_single_range_blocking(vl53l0 *dev, uint16_t *buf);



//Write a single byte to a specified register
int write_register(vl53l0 *dev, uint8_t reg, uint8_t data);


//Read a 16 bit value from a (pair of) register(s)
int read_16_bit_register(vl53l0 *dev, uint8_t reg, uint8_t *buf);


//Read a single range measurement from the ToF device
int get_range(vl53l0 *dev, uint8_t *buf);

/*
TO ADD:
-Read of a two byte value (i.e. range)
*/




#endif