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
int init_interrupt(vl53l0 *dev);
int load_def_config(vl53l0 *dev);
int get_spad_info(vl53l0 *dev, uint8_t *count, bool *type_is_aperture);
int init_spad(vl53l0 *dev);
void start_continuous_mode(vl53l0 *dev);
int read_continuous_mode(vl53l0 *dev, uint8_t *buf);
void stop_continuous_mode(vl53l0 *dev);


int main()
{

    stdio_init_all();


    wait_for_go();

    //Initialise the ToF
    vl53l0 ToF = init_vl53l0(0, I2C_SDA, I2C_SCL, EN_P);

    //Start the reading
    start_continuous_mode(&ToF);


    //16 bits is 2 lots of 8 bit registers
    int8_t result[2];



    //Attempt to get a range until the device provides a valid one
    while(read_continuous_mode(&ToF, result) != 1){
        //printf("RANGE VAL: %d\n", result);
    }
    
    printf("RANGE VAL: %d", result);

    stop_continuous_mode(&ToF);

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
    int succ = read_register(&ToF_dev, MODEL_ID, &buf);

    if (succ == 1 && buf == EXPECTED_ID){      
       

        //Setup the default config of the device
        setup_default_config(&ToF_dev);
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
    write_register(dev, 0x80, 0x01);
    write_register(dev, 0xFF, 0x01);
    write_register(dev, 0x00, 0x00);
    //Assign the stop variable
    read_register(dev, 0x91, &dev->stop_variable);
    write_register(dev, 0x00, 0x01);
    write_register(dev, 0xff, 0x00);
    write_register(dev, 0x80, 0x00);


    uint8_t MSRC_read;
    read_register(dev, REG_MSRC_CONFIG_CONTROL, &MSRC_read);
    write_register(dev, REG_MSRC_CONFIG_CONTROL, MSRC_read | 0x12);

    write_16bit_register(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);

    write_register(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);



    
    //SPAD calibration
    init_spad(dev);

    //Default calibration
    load_def_config(dev);


    //Setup interrupt configuration
    init_interrupt(dev);


    write_register(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(dev, 0x40))
    {
        return -1;
    }

    write_register(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(dev, 0x00))
    {
        return -1;
    }

    // "restore the previous Sequence Config"
    write_register(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    printf("Default config setup");
}


//Initialise the SPAD registers
int init_spad(vl53l0 *dev){

    //Get the spad info
    uint8_t spad_count, spad_type_is_apeture;
    
    bool spad_type_is_aperture;
    if (get_spad_info(dev, &spad_count, &spad_type_is_aperture) == false)
    {
        return false;
    }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    read_multi(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    write_register(dev, 0xFF, 0x01);
    write_register(dev,DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_register(dev,DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_register(dev,0xFF, 0x00);
    write_register(dev,GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    write_multi(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);



}

//Borrowed from yspreen VL53L0x driver on github
int load_def_config(vl53l0 *dev){
    
    write_register(dev, 0xFF, 0x01);
    write_register(dev,0x00, 0x00);

    write_register(dev,0xFF, 0x00);
    write_register(dev,0x09, 0x00);
    write_register(dev,0x10, 0x00);
    write_register(dev,0x11, 0x00);

    write_register(dev,0x24, 0x01);
    write_register(dev,0x25, 0xFF);
    write_register(dev,0x75, 0x00);

    write_register(dev,0xFF, 0x01);
    write_register(dev,0x4E, 0x2C);
    write_register(dev,0x48, 0x00);
    write_register(dev,0x30, 0x20);

    write_register(dev,0xFF, 0x00);
    write_register(dev,0x30, 0x09);
    write_register(dev,0x54, 0x00);
    write_register(dev,0x31, 0x04);
    write_register(dev,0x32, 0x03);
    write_register(dev,0x40, 0x83);
    write_register(dev,0x46, 0x25);
    write_register(dev,0x60, 0x00);
    write_register(dev,0x27, 0x00);
    write_register(dev,0x50, 0x06);
    write_register(dev,0x51, 0x00);
    write_register(dev,0x52, 0x96);
    write_register(dev,0x56, 0x08);
    write_register(dev,0x57, 0x30);
    write_register(dev,0x61, 0x00);
    write_register(dev,0x62, 0x00);
    write_register(dev,0x64, 0x00);
    write_register(dev,0x65, 0x00);
    write_register(dev,0x66, 0xA0);

    write_register(dev,0xFF, 0x01);
    write_register(dev,0x22, 0x32);
    write_register(dev,0x47, 0x14);
    write_register(dev,0x49, 0xFF);
    write_register(dev,0x4A, 0x00);

    write_register(dev,0xFF, 0x00);
    write_register(dev,0x7A, 0x0A);
    write_register(dev,0x7B, 0x00);
    write_register(dev,0x78, 0x21);

    write_register(dev,0xFF, 0x01);
    write_register(dev,0x23, 0x34);
    write_register(dev,0x42, 0x00);
    write_register(dev,0x44, 0xFF);
    write_register(dev,0x45, 0x26);
    write_register(dev,0x46, 0x05);
    write_register(dev,0x40, 0x40);
    write_register(dev,0x0E, 0x06);
    write_register(dev,0x20, 0x1A);
    write_register(dev,0x43, 0x40);

    write_register(dev,0xFF, 0x00);
    write_register(dev,0x34, 0x03);
    write_register(dev,0x35, 0x44);

    write_register(dev,0xFF, 0x01);
    write_register(dev,0x31, 0x04);
    write_register(dev,0x4B, 0x09);
    write_register(dev,0x4C, 0x05);
    write_register(dev,0x4D, 0x04);

    write_register(dev,0xFF, 0x00);
    write_register(dev,0x44, 0x00);
    write_register(dev,0x45, 0x20);
    write_register(dev,0x47, 0x08);
    write_register(dev,0x48, 0x28);
    write_register(dev,0x67, 0x00);
    write_register(dev,0x70, 0x04);
    write_register(dev,0x71, 0x01);
    write_register(dev,0x72, 0xFE);
    write_register(dev,0x76, 0x00);
    write_register(dev,0x77, 0x00);

    write_register(dev,0xFF, 0x01);
    write_register(dev,0x0D, 0x01);

    write_register(dev,0xFF, 0x00);
    write_register(dev,0x80, 0x01);
    write_register(dev,0x01, 0xF8);

    write_register(dev,0xFF, 0x01);
    write_register(dev,0x8E, 0x01);
    write_register(dev,0x00, 0x01);
    write_register(dev,0xFF, 0x00);
    write_register(dev,0x80, 0x00);

    printf("Default config loaded\n");
}

//Setup the interrupt config so we can poll the device for when it is ready
int init_interrupt(vl53l0 *dev){

    write_register(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);


    uint8_t HV_MUX = 0;
    read_register(dev, GPIO_HV_MUX_ACTIVE_HIGH, &HV_MUX);
    write_register(dev, GPIO_HV_MUX_ACTIVE_HIGH, (HV_MUX & ~0x10));
    write_register(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);


}




//Read a single byte from the VL53l0 device
int read_register(vl53l0 *dev, uint8_t reg, uint8_t *buf){

    //Indicates a succesful write
    int succ = i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR, &reg, 1, true);

    //Check that the write was succesful
    if (succ == 1){
        //Read from the i2c device
        succ = i2c_read_blocking(dev->I2C_HW, VL53L0_ADDR, buf, 1, false);

    }else{
        return succ;
    }    

     //Check whether the byte was read
    if (succ == 1){
        return 1;
    }else{
        printf("%04x -READ ERR - %d\n", reg, succ);
        return succ;
    }
}


//Updates a value in a register
int write_register(vl53l0 *dev, uint8_t reg, uint8_t data){


    uint8_t data_to_write[2];
    data_to_write[0] = reg;
    data_to_write[1] = data;

    //Write addr then index then data to the VL53l0 device
    int succ = i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR, data_to_write, 2, false);

    //Check whether the byte was written
    if (succ == 2){
        return 1;
    }else{
        printf("FAILED TO WRITE VAL (%04x) TO REG  (%04x) - %d\n", data, reg, succ);
        return succ;
    }
}

//Write a value to a pair of byte registers
int write_16bit_register(vl53l0 *dev, uint8_t reg, uint16_t data){

    uint8_t data_to_write[3];
    data_to_write[0] = reg;
    //MSB first
    //Rightshift the top byte to sit in the lower byte
    data_to_write[1] = data  >> 8;
    //Mask with a full lower byte
    data_to_write[2] = data & 0x00FF;


    int succ = i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR, data_to_write, 3, false);

    //Check whether the byte was written
    if (succ == 3){
        return 1;
    }else{
        printf("FAILED TO WRITE TO 16bit REG - %d\n", succ);
        return succ;
    }


}


//Read a 16 bit value from a register
int read_16_bit_register(vl53l0 *dev, uint8_t reg, uint8_t *buf){


    //Request the device primes the required register
    int succ = i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR, &reg, 1, true);

    if(succ = 1){

        succ = i2c_read_blocking(dev->I2C_HW, VL53L0_ADDR, buf, 2, false);

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


    write_register(dev, 0x80, 0x01);
    write_register(dev, 0xFF, 0x01);
    write_register(dev, 0x00, 0x00);
    write_register(dev, 0x91, dev->stop_variable);
    write_register(dev, 0x00, 0x01);
    write_register(dev, 0xFF, 0x00);
    write_register(dev, 0x80, 0x00);

    //Tell the device to take a measurement
    write_register(dev, SYSRANGE_START, 0x01);

    printf("DEVICE TOLD TO MEASURE-----------\n");
    int timeout = 0;
    uint8_t timeout_buf;

     // Wait until startbit is cleared
    read_register(dev, SYSRANGE_START, &timeout_buf);
    while(timeout_buf & 0x01){
        timeout++;
        sleep_us(5000);
        if (timeout > 50){
            printf("STARTBIT CLEAR TIMEOUT");
            return -1;
        }
        read_register(dev, SYSRANGE_START, &timeout_buf);
    }

    timeout = 0;
    
    read_register(dev, RESULT_INTERRUPT_STATUS, &timeout_buf);

    
   //Wait until the device has a measurement ready
    while ((timeout_buf & 0x07) == 0){      
        
        //printf("(%04x)", timeout_buf);

        timeout++;
        sleep_us(5000);        

        if (timeout > 50){
                printf("INTERRUPT READING TIMEOUT\n");
                //break;
                return -1;
        }
        read_register(dev, RESULT_INTERRUPT_STATUS, &timeout_buf);

    }
    

    //Read the range value (2 bytes)
    read_16_bit_register(dev, VL53L0_RANGE_RESULT_STATUS + 10, buf);

    //Clear the interupts
    return write_register(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

}




int read_multi(vl53l0 *dev, uint8_t reg, uint8_t *buf, uint8_t cnt)
{
    // First write the register address we want to read from
    int succ = i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR, &reg, 1, true);
    // Now read count bytes into the dst buffer
    if (succ == PICO_ERROR_NONE)
    {
        succ = i2c_read_blocking(dev->I2C_HW, VL53L0_ADDR, buf, cnt, false);
    }

    return succ;
}







// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
int get_spad_info(vl53l0 *dev, uint8_t *count, bool *type_is_aperture)
{


    write_register(dev, 0x80, 0x01);
    write_register(dev,0xFF, 0x01);
    write_register(dev,0x00, 0x00);

    write_register(dev,0xFF, 0x06);


    uint8_t reg_0x83_val;
    printf("SPAD--------\n");
    read_register(dev, 0x83, &reg_0x83_val);

    write_register(dev,0x83, reg_0x83_val | 0x04);
    write_register(dev,0xFF, 0x07);
    write_register(dev,0x81, 0x01);

    write_register(dev,0x80, 0x01);

    write_register(dev,0x94, 0x6b);
    write_register(dev,0x83, 0x00);
    

    int cnt = 0;

    read_register(dev, 0x83, &reg_0x83_val);
    while (reg_0x83_val == 0x00)
    {
        if (cnt > 500)
        {
            return -1;
        }

        cnt = cnt + 1;
        read_register(dev, 0x83, &reg_0x83_val);

    }

    write_register(dev,0x83, 0x01);
    uint8_t tmp;
    read_register(dev, 0x92, &tmp);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    write_register(dev,0x81, 0x00);
    write_register(dev,0xFF, 0x06);
    read_register(dev, 0x83, &reg_0x83_val);
    write_register(dev,0x83, reg_0x83_val & ~0x04);
    write_register(dev,0xFF, 0x01);
    write_register(dev,0x00, 0x01);

    write_register(dev,0xFF, 0x00);
    write_register(dev,0x80, 0x00);

    return true;
}

int write_multi(vl53l0 *dev, uint8_t reg, uint8_t const *data, uint8_t count){

    // Create a buffer that is one byte larger than count to accommodate the register address
    uint8_t buffer[count + 1];
    buffer[0] = reg; // First byte is register address
    // Copy the src data into the buffer starting at the second byte
    memcpy(buffer + 1, data, count);
    // Write the buffer to I2C
    return i2c_write_blocking(dev->I2C_HW, VL53L0_ADDR, buffer, count + 1, false);

}


// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(vl53l0 *dev, uint8_t vhv_init_byte)
{
    write_register(dev, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    int count = 0;

    uint8_t int_stat;
    read_register(dev, RESULT_INTERRUPT_STATUS, &int_stat);

    while ((int_stat & 0x07) == 0)
    {
        if (count > 500)
        {
            return false;
        }
        read_register(dev, RESULT_INTERRUPT_STATUS, &int_stat);
        count = count + 1;
    }

    write_register(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    write_register(dev, SYSRANGE_START, 0x00);

    return true;
}


//Start the device running in continious measurement mode
void start_continuous_mode(vl53l0 *dev){

    write_register(dev, 0x80, 0x01);
    write_register(dev, 0xFF, 0x01);
    write_register(dev, 0x00, 0x00);
    write_register(dev, 0x91, dev->stop_variable);
    write_register(dev, 0x00, 0x01);
    write_register(dev, 0xFF, 0x00);
    write_register(dev, 0x80, 0x00);

    write_register(dev, SYSRANGE_START, 0x02);

}

//Read a value from continious mode
int read_continuous_mode(vl53l0 *dev, uint8_t *buf){
    
    int timeout = 0;
    uint8_t timeout_buf;
    
    read_register(dev, RESULT_INTERRUPT_STATUS, &timeout_buf);

    
   //Wait until the device has a measurement ready
    while ((timeout_buf & 0x07) == 0){       
    

        timeout++;
        sleep_us(5000);        

        if (timeout > 50){
                printf("INTERRUPT READING TIMEOUT\n");
                //break;
                return -1;
        }
        read_register(dev, RESULT_INTERRUPT_STATUS, &timeout_buf);

    }
    

    //Read the range value (2 bytes)
    read_16_bit_register(dev, VL53L0_RANGE_RESULT_STATUS + 10, buf);

    //Clear the interupts
    return write_register(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

}

//Stop continious mode measurement
void stop_continuous_mode(vl53l0 *dev){

    write_register(dev, SYSRANGE_START, 0x02);

    write_register(dev, 0xFF, 0x01);
    write_register(dev, 0x00, 0x00);
    write_register(dev, 0x91, dev->stop_variable);
    write_register(dev, 0x00, 0x01);
    write_register(dev, 0xFF, 0x00);

}


