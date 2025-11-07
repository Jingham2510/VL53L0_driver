#include "VL53L0_DRIVER.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"





// I2C defines
#define VL53L0_I2C_0 i2c0
#define VL53L0_I2C_1 i2c1
#define I2C_SDA 8
#define I2C_SCL 9
#define EN_P 10



/*
PROCESS WRITE:
1) Start condition (i.e. read/write address)
2)VL53L0 acknowledges by driving SDA low 
3) Pi sends index
4) VL53L0 acks
5) Data written
6) VL53L0 Ack
7) Stop
*/

/*
PROCESS READ:
1) Start condition (i.e. read/write address)
2)VL53L0 acknowledges by driving SDA low 
3) Pi sends index
4) VL53L0 acks
5) stop/start
6) VL53L0 sends address
7) Pi Ack
8) VL53L0 sends data
9) Pi acks
*/


void test_LED_on();
void wait_for_go();


int main()
{
    stdio_init_all();


    wait_for_go();

    test_LED_on();


   //FOR TESTING PURPOSES - NEED BREADBOARD FOR THIS
    gpio_set_function(10, GPIO_FUNC_SIO);
    gpio_set_dir(10, GPIO_OUT);
    gpio_put(10, true);       

    vl53l0 ToF = init_vl53l0(0, I2C_SDA, I2C_SCL, EN_P);

    uint8_t reg = VL53L0_REF_REG_1;



    //CURRENTLY CRASHING WHEN TRYING TO WRITE A BYTE

    //Attempt to write a single random byte
    int succ = write_byte(&ToF, &reg);  


    //Turn the LED on if byte written successfully
    if (succ == 1){        
        //test_LED_on();        
    }

    return 1;

}

/*
Helper function to turn board LED on to verify parts of code are reached
*/
void test_LED_on(){

    //Using th wireless chip to activate the LED
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
}


/*
Helper function to apply a wait until the USB bus recieves something (anything)
*/
void wait_for_go(){
    
    //Wait for a serial bit/byte
    get_char();

    //Tell the device its going
    printf("Going!");

}


//Initialise the I2C connection with the vl53l0 (and verify the connection)
vl53l0 init_vl53l0(int I2C_HW, int SDA_pin, int SCL_pin, int EN_pin){
    
    //Assign a hardware block

    i2c_inst_t *HW_block;

    //Default to the first I2C block
    if (I2C_HW != 1){
        HW_block = VL53L0_I2C_0;
    }else{
        HW_block = VL53L0_I2C_1;
    }


    //Turn on the I2C hw block
    i2c_init(HW_block, DEFAULT_BAUD);

    //Setup the I2C pins 
    gpio_set_function(SDA_pin, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_pin);
    gpio_set_function(SCL_pin, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_pin);

    //Setup the enable pin - and turn it off
    gpio_set_function(EN_pin, GPIO_FUNC_SIO);
    gpio_set_dir(EN_pin, true);
    gpio_pull_down(EN_pin);
    gpio_put(EN_pin, false);



    //Create the vl53l0 struct
    vl53l0 ToF_dev;
    ToF_dev.I2C_HW = HW_block;
    ToF_dev.SDA_PIN = SDA_pin;
    ToF_dev.SCL_PIN = SCL_pin;
    ToF_dev.EN_PIN = EN_pin;

    return ToF_dev;
}


//Write a single byte to the provided VL53l0 device
int write_byte(vl53l0 *dev, uint8_t *byte){

    //Write one byte to the i2c register - then issue a stop
    int succ = i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR_WRITE, byte, 1, false);

    

    //Check whether the byte was written
    if (succ == 1){        
        test_LED_on();
        return 1;
    }else{
        return -1;
    }
}

//Read a single byte from the VL53l) device
int read_byte(vl53l0 *dev, uint8_t *addr, uint8_t *buf){

    //Indicates a succesful read/write
    int succ;

    //Write the address of interest
    succ = write_byte(dev, addr);

    //Check that the write was succesful
    if (succ == 1){
        //Read from the i2c device
        succ = i2c_read_blocking(dev->I2C_HW, VL53L0_ADDR_READ, buf, 1, false);

    }else{
        return -1;
    }    

     //Check whether the byte was read
    if (succ == 1){
        return 1;
    }else{
        return -1;
    }

}


//Updates a value in a register
int update_register(vl53l0 *dev, uint8_t *reg, uint8_t *data){


    uint8_t data_to_write[2];
    data_to_write[0] = *reg;
    data_to_write[1] = *data;

    //Write addr then index then data to the VL53l0 device
    int succ = i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR_WRITE, data_to_write, 2, false);

    //Check whether the byte was written
    if (succ == 1){
        return 1;
    }else{
        return -1;
    }
}



//Read the range from the device
int read_single_range_blocking(vl53l0 *dev, uint16_t *buf){

    
    //Indicates a succesful read/write
    int succ;


    //Set the mode
    succ = update_register(dev, VL53L0_SYSRANGE_START, VL53L0_SYS_RANGE_MODE_SINGLE);

    //Ensure the mode was set correctly
    if (succ != 1){
        return -1;
    }


    //Wait until the device has a measurement ready

    //Read the range value (2 bytes)


}









