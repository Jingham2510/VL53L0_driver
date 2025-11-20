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


    uint8_t stop_variable;

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

//Sequence config
#define SYSTEM_SEQUENCE_CONFIG 0x01

//Range settings (i.e. tell sensor to start the range then the mode)
#define SYSRANGE_START 0x00
#define SYS_RANGE_MODE_SINGLE 0x00
#define SYS_RANGE_MODE_CONTINUOUS 0x01

//Range interrupt status 
#define SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define RESULT_INTERRUPT_STATUS 0x13
#define SYSTEM_INTERRUPT_CLEAR 0x0B

//Range status register (i.e. the reading)
#define VL53L0_RANGE_RESULT_STATUS 0x14

//GPIO High Voltage
#define GPIO_HV_MUX_ACTIVE_HIGH 0x84

#define REG_MSRC_CONFIG_CONTROL 0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44


//SPAD STUFF
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6


#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71







//Initialise the pins to connect with the vl53l0
vl53l0 init_vl53l0(int I2C_HW, int SDA_pin, int SCL_pin, int EN_pin);



//Setup the device to a default running state
int setup_default_config();


/*
Read a single byte from the VL53L0
Return a success/fail if the byte is written
*/
int read_register(vl53l0 *dev, uint8_t reg, uint8_t *buf);


/*
Read a single range measurement from the device
*/
int read_single_range_blocking(vl53l0 *dev, uint16_t *buf);



//Write a single byte to a specified register
int write_register(vl53l0 *dev, uint8_t reg, uint8_t data);


//Read a 16 bit value from a (pair of) register(s)
uint16_t read_16_bit_register(vl53l0 *dev, uint8_t reg);

int read_multi(vl53l0 *dev, uint8_t reg, uint8_t *buf, uint8_t cnt);


//Read a single range measurement from the ToF device
int get_range(vl53l0 *dev, uint8_t *buf);

//Write a value to a pair of byte registers
int write_16bit_register(vl53l0 *dev, uint8_t reg, uint16_t data);


int write_multi(vl53l0 *dev, uint8_t reg, uint8_t const *data, uint8_t count);

bool performSingleRefCalibration(vl53l0 *dev, uint8_t vhv_init_byte);




#endif