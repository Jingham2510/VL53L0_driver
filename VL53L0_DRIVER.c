#include "VL53L0_DRIVER.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tusb.h"

// I2C defines
#define VL53L0_I2C_0 i2c0
#define VL53L0_I2C_1 i2c1
#define I2C_SDA 16
#define I2C_SCL 17
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

    //Initialise the ToF
    vl53l0 ToF = init_vl53l0(0, I2C_SDA, I2C_SCL, EN_P);


    //16 bits is 2 lots of 8 bit registers
    int8_t result[2];

    if(get_range(&ToF, result) == 1){
        printf("RANGE VAL: %d", result);
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
    

    while(!tud_cdc_connected()){
        sleep_ms(100);
        printf("Waiting...!\n");

    }

    //Wait for a serial bit/byte
    getchar();

    //Tell the device its going
    printf("Going!\n");

}


//Initialise the I2C connection with the vl53l0 (and verify the connection)
vl53l0 init_vl53l0(int I2C_HW, int SDA_pin, int SCL_pin, int EN_pin){
    
    //Assign a hardware block

    i2c_inst_t *HW_block;

    //Default to the first I2C block (incase of erroneous input)
    if (I2C_HW != 1){
        HW_block = VL53L0_I2C_0;
    }else if(I2C_HW == 1){
        HW_block = VL53L0_I2C_1;
    }


    //Turn on the I2C hw block
    if(i2c_init(HW_block, DEFAULT_BAUD) > DEFAULT_BAUD){
        printf("DEVICE MAY BE OVERDRIVEN");
    }

    //printf("I2C HW BLOCK INITIALISED\n");

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



    //Verify that we are connected to correct device
    uint8_t buf;
    int succ = read_byte(&ToF_dev, MODEL_ID, &buf);

    if (succ == 1 && buf == EXPECTED_ID){      
       

        //Setup the default config of the device
        setup_default_config(&ToF_dev);


        printf("ToF initialised\n");
        return ToF_dev;

    }else{
        printf("Read byte - %d\n", buf);
        printf("ToF UNINITIALISED\n");
    }


}

//Setup the default config of the device (particularly to 2v8 mode)
int setup_default_config(vl53l0 *dev){

    //Set the voltage mode to 2v8
    write_register(dev, EXTSUP_2V8, 0x01);

    //Set the I2C mode to standard
    write_register(dev, I2C_MODE, I2C_MODE_STANDARD);

    //Setup the power management
    write_register(dev, POW_MGMT, 0x01);

    //Internal tuning
    write_register(dev, INTERNAL_TUNING, 0x01);

    //Set the default range mode
    write_register(dev, VL53L0_SYSRANGE_START, VL53L0_SYS_RANGE_MODE_SINGLE);

    

}


//Write a single byte to the provided VL53l0 device
int write_byte(vl53l0 *dev, uint8_t *byte){

    //printf("Attempting to write byte - %d \n", *byte);

    //Write one byte to the i2c register - then issue a stop
    int succ = i2c_write_blocking_until(dev->I2C_HW, VL53L0_ADDR, byte, 1, false, make_timeout_time_ms(1000));

    //Check whether the byte was written
    if (succ == 1){    
        return 1;
    }else if (succ == -1){
        printf("Failed to find device/No Acknowledgement!\n");
        return -1;
    }else{
        printf("Failed to write byte! - %d\n", succ);
        return succ;
    }
}

//Read a single byte from the VL53l0 device
int read_byte(vl53l0 *dev, uint8_t addr, uint8_t *buf){

    //Indicates a succesful read/write
    int succ;

    //Write the address of interest
    succ = write_byte(dev, &addr);

    //Check that the write was succesful
    if (succ == 1){
        //Read from the i2c device
        succ = i2c_read_blocking_until(dev->I2C_HW, VL53L0_ADDR, buf, 1, false, make_timeout_time_ms(10));

    }else{
        return succ;
    }    

     //Check whether the byte was read
    if (succ == 1){
        return 1;
    }else{
        printf("READ ERR - %d\n", succ);
        return succ;
    }
}


//Updates a value in a register
int write_register(vl53l0 *dev, uint8_t reg, uint8_t data){


    uint8_t data_to_write[2];
    data_to_write[0] = reg;
    data_to_write[1] = data;

    //Write addr then index then data to the VL53l0 device
    int succ = i2c_write_burst_blocking(dev->I2C_HW, VL53L0_ADDR, data_to_write, 2);

    //Check whether the byte was written
    if (succ == 2){
        return 1;
    }else{
        printf("FAILED TO WRITE TO REG - %d", succ);
        return succ;
    }
}


//Read a 16 bit value from a register
int read_16_bit_register(vl53l0 *dev, uint8_t reg, uint8_t *buf){


    //Request the device primes the required register
    int succ = write_byte(dev, &reg);

    if(succ = 1){

        succ = i2c_read_burst_blocking(dev->I2C_HW, VL53L0_ADDR, buf, 2);

        if (succ == 2){
            return 1;
        }else{
            //Dont return succ here as there is a risk that it equates to 1 (i.e. 1 byte read)
            return -1;
        }


    }else{
        printf("FAILED TO READ REGISTER");
        return -1;
    }


}


//Read the range from the device
int get_range(vl53l0 *dev, uint8_t *buf){

    //Tell the device to take a measurement
    write_register(dev, VL53L0_SYSRANGE_START, VL53L0_SYS_RANGE_MODE_SINGLE);

    int timeout = 0;
    uint8_t timeout_buf;
    
    read_byte(dev, VL53L0_RANGE_RESULT_STATUS, &timeout_buf);

   //Wait until the device has a measurement ready
    while ((timeout_buf & 0x07) == 0){        
        timeout++;
        sleep_us(5000);
        

        if (timeout> 50){
                printf("READING TIMEOUT");
                return -1;
        }

        read_byte(dev, VL53L0_RANGE_RESULT_STATUS, &timeout_buf);

    }

    //Read the range value (2 bytes)
    read_16_bit_register(dev, VL53L0_RANGE_RESULT_STATUS, buf);

    //Clear the interupts
    return write_register(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

}









