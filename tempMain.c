/*
 * File:   PulseOX.c
 * Author: Dhayalan balasubramanian
 *
 * Created on April 10, 2024, 9:26 AM
 */


#include <xc.h>
#include <stdint.h>
#include "grove-lcd.h"
#include "button.h"


// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL


// Define I2C communication parameters
#define MAX30102_ADDRESS_WRITE 0xAE //writing
#define MAX30102_ADDRESS_READ  0xAF //reading


// Define register addresses
#define FIFO_WR_PTR    0x04
#define FIFO_RD_PTR    0x06
#define FIFO_OVERFLOW  0x05
#define FIFO_DATA      0x07
#define FIFO_DEPTH     32
#define PART_ID        0xFF
uint32_t partID = 0;
//uint32_t FIFO_WR_PTR_DATA;
#define NUM_SAMPLES_TO_READ 50 //50 / 100 / 200/ 400/ 800
//#define NUM_AVAILABLE_SAMPLES

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static const uint8_t MAX30105_MODE_MASK = 0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 0x03;
static const uint8_t MAX30105_MODE_MULTILED = 0x07;
static const uint8_t MAX30105_MODECONFIG = 0x09;

static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A; 

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03;

static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;

#define MAX30105_PULSEWIDTH_69    0x02
#define MAX30105_PULSEWIDTH_118   0x01
#define MAX30105_PULSEWIDTH_215   0x00
#define MAX30105_PULSEWIDTH_411   0x03

static const uint8_t MAX30105_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30105_SHUTDOWN = 		0x80;
static const uint8_t MAX30105_WAKEUP = 			0x00;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30105_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;

static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

//static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
//static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
//static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;

#define MAX30102_MODECONFIG    0x09  // Mode configuration register address
#define MAX30102_RESET_MASK    0xBF  // Mask to clear the RESET bit
#define MAX30102_RESET         0x40  // Value to initiate a reset





/*
 * initialize the PIC24 microcontroller, 
 * mostly done to setup the LCD. 
 */
void pic24_init() {
    CLKDIVbits.RCDIV = 0;  //clock to 16
    AD1PCFG = 0xffff; //all pins digital
    TRISAbits.TRISA0 = 0; 
}


void I2C_start() {
    I2C2CONbits.SEN = 1;   // Initiate start condition
    while (I2C2CONbits.SEN);
};

void I2C_stop() {
    I2C2CONbits.PEN = 1;   // Initiate stop condition
    while (I2C2CONbits.PEN);
};

// Function to write data over I2C
void I2C_write(uint8_t data) {
    I2C2TRN = data;        //  I2C transimit reg
    while (I2C2STATbits.TRSTAT); // Wait for transmission to complete
    
//    while (I2C2STATbits.ACKSTAT); // Wait for ACK/NACK from slave
    //while(IFS3bits.MI2C2IF==0);

}

// Function to read data over I2C
uint32_t I2C_read() {
    I2C2RCV = 0;
    I2C2CONbits.RCEN = 1; // Enable receive mode
    while (I2C2CONbits.RCEN == 1);
    return I2C2RCV;        // I2C receive re
}

void I2C_repeated_start() {
    I2C2CONbits.RSEN = 1;  // Initiate repeated start condition
    while (I2C2CONbits.RSEN == 1);
    //while (I2C2CONbits.RSEN); // Wait for repeated start condition to complete
}

void I2C_ACK() {
    I2C2CONbits.ACKDT =1;//acknowledge data bit(receive)
    I2C2CONbits.ACKEN =1;
    while (IFS3bits.MI2C2IF==0);
}

void I2C_NACK() {
    I2C2CONbits.ACKDT = 0; //not acknowledge data(receive)
    I2C2CONbits.ACKEN = 0 ;
    while (IFS3bits.MI2C2IF==0);
}



// Initialize MAX30102 sensor
void max30102_init(void) {
    I2C2CONbits.I2CEN = 0;
    
    I2C2BRG = 0x25; //set to a clock freq of 400kHz and 16MHz Fcy/ BAUD RATE
//    TRISBbits.TRISB6 = 1;  // SDA pin as input
//    TRISBbits.TRISB7 = 1;  // SCL pin as input
    I2C2CONbits.I2CEN = 1; //Enable I2C mode
    //I2C2CONbits.SMODE = 0;

    IFS3bits.MI2C2IF = 0; // clr Int flag
}





void max30102_write_config_SP02() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x0A); 
    //LATAbits.LATA0 = 1;
    I2C_write(0x27);   //00100111    00100001

    I2C_stop();
}

void max30102_write_config_FIFO() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x08); 

    I2C_write(0x40);     //0b10111111   

    I2C_stop();
}

 void max30102_write_config_MODE() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x09); 

    I2C_write(0X03);       

    I2C_stop();
}
 
 void max30102_write_config_RESET_MODE() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x09); 

    I2C_write(0x40);       

    I2C_stop();
}
 
 void max30102_write_config_RED_PulseAmplitude() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x0C); 

    I2C_write(0x1F);       

    I2C_stop();
}
 
void max30102_write_config_IR_PulseAmplitude() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x0D); 

    I2C_write(0x1F);       

    I2C_stop();
}

void max30102_write_config_LED_CONTROL() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x11); 

    I2C_write(0x1F);       

    I2C_stop();
}

//void read_data_from_FIFO() {
//    uint8_t FIFO_WR_PTR_DATA = 0;
//    uint8_t NUM_AVAILABLE_SAMPLES;
//    //uint8_t NUM_SAMPLES_TO_READ;
//    uint32_t LED1_sample, LED2_sample;
//
//    // First transaction: Get the FIFO_WR_PTR
//    I2C_start();
//    I2C_write(MAX30102_ADDRESS_WRITE); 
//    I2C_write(FIFO_WR_PTR); 
//    I2C_repeated_start();
//    I2C_write(MAX30102_ADDRESS_READ); 
//    FIFO_WR_PTR_DATA = I2C_read(); 
//    I2C_stop();
//
//    // Evaluate the number of samples to be read from the FIFO
//    //FIFO_RD_PTR_DATA = I2C_read(); // Read FIFO_RD_PTR from the device
//    NUM_AVAILABLE_SAMPLES = (FIFO_WR_PTR_DATA - FIFO_RD_PTR + 1) % 256; // Assuming 8-bit pointers
//    //NUM_SAMPLES_TO_READ = 50; // Define the value based on your requirement
//
//    // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
//    I2C_start();
//    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode
//    I2C_write(FIFO_DATA); // Send address of FIFO_DATA
//    I2C_repeated_start();
//    I2C_write(MAX30102_ADDRESS_READ); // Send device address + read mode
//
//    for (int i = 0; i < NUM_SAMPLES_TO_READ; i++) {
//        // Read LED1 sample
//        LED1_sample  = (uint32_t)I2C_read() << 16;
//        LED1_sample |= (uint32_t)I2C_read() << 8;
//        LED1_sample |= (uint32_t)I2C_read();
//
//        // Read LED2 sample
//        LED2_sample  = (uint32_t)I2C_read() << 16;
//        LED2_sample |= (uint32_t)I2C_read() << 8;
//        LED2_sample |= (uint32_t)I2C_read();
//        
//        // Process LED1(RED) and LED2(IR) samples
//        //process_SPO2_data(LED1_sample, LED2_sample); // Replace with your processing function
//    }
//    
//    I2C_stop();
//
//    // Update FIFO_RD_PTR
//    I2C_start();
//    I2C_write(MAX30102_ADDRESS_WRITE);
//    I2C_write(FIFO_RD_PTR);
//    I2C_write(FIFO_RD_PTR);
//    I2C_stop();
//}

//void process_SP02_data(LED1_sample, LED2_sample){
//    int i;
//}


uint32_t max30102_read_partID() {
        I2C_start();  
        I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode
        

        I2C_write(PART_ID); // Send address of PART_ID
        

        I2C_repeated_start(); // Repeated start condition unnecessary
        
        I2C_write(MAX30102_ADDRESS_READ); // Send device address + read mode
        

        partID = I2C_read(); // Read PART_ID
        
        I2C2CONbits.ACKDT = 1;
        
        I2C2CONbits.ACKEN = 1;
        while (I2C2CONbits.ACKEN == 1);
        //LATAbits.LATA0 = 1;
        I2C_stop();
        
        return partID;

}


void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
uint8_t readRegister8(uint8_t reg);
uint8_t I2C_write_with_params(uint8_t reg, uint8_t val);
void setLEDMode(uint8_t mode);

uint8_t readRegister8(uint8_t reg)
{
    I2C_start();

    // Send device address with write bit
    I2C_write(MAX30102_ADDRESS_WRITE);

    // Send register address
    I2C_write(reg);

    // Restart I2C communication
    I2C_repeated_start();

    // Send device address with read bit
    I2C_write(MAX30102_ADDRESS_READ);

    // Read received data
    uint8_t data = I2C_read();
    
    I2C2CONbits.ACKDT = 1;
        
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN == 1);

    // Stop I2C communication
    I2C_stop();

    return data;
}


uint8_t I2C_write_with_params(uint8_t reg, uint8_t val) {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(reg); 

    I2C_write(val);       

    I2C_stop();
}

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing) {
    // Grab current register context
    uint8_t originalContents = readRegister8(reg);

    // Clear out the portions of the register that are masked
    originalContents &= ~mask;  // Invert mask to clear bits

    // Set the desired bits
    originalContents |= (thing & mask);  // Only change bits that are 1s in mask

    // Write the modified contents back to the register
    I2C_write_with_params(reg, originalContents);
}


void setLEDMode(uint8_t mode) {
    // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
    // See datasheet, page 19
    bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void setPulseWidth(uint8_t pulseWidth) {
    bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

void setPulseAmplitudeRed(uint8_t amplitude) {
    I2C_write_with_params(MAX30105_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeProximity(uint8_t amplitude){
    I2C_write_with_params(MAX30105_LED_PROX_AMP, amplitude);
}

void setPulseAmplitudeIR(uint8_t amplitude) {
  I2C_write_with_params(MAX30105_LED2_PULSEAMP, amplitude);
}

void max30102_wakeUp(void) {
  bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

void max30102_enableSlot(uint8_t device) {
  bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
     
}

void max30102_softReset(void) {
    I2C_start();  
    
    I2C_write(MAX30102_ADDRESS_WRITE);
    I2C_write(MAX30102_MODECONFIG);
    I2C_write(0x40);  // Reset command
    I2C_stop();

    delay_ms(100); // Wait for reset to complete

    // Check if reset is complete
    uint8_t resetStatus;
    do {
        resetStatus = readRegister8(MAX30102_MODECONFIG);
    } while (resetStatus & 0x40);  // Wait for the reset bit to clear
}



//void delay_ms(unsigned int ms) {
//    while(ms-- > 0) {
//        asm("repeat #15998");
//        asm("nop");
//    }
//}




int main(void) {
    pic24_init();
    max30102_init();
    grovelcd_init();
    
    max30102_write_config_RESET_MODE();
//    max30102_wakeUp();
    max30102_write_config_MODE();
    max30102_read_partID();
//    max30102_write_config_FIFO();
//    max30102_write_config_SP02();
//    max30102_write_config_RED_PulseAmplitude();
//    max30102_write_config_IR_PulseAmplitude();
//    max30102_write_config_LED_CONTROL();
//    setLEDMode(MAX30105_MODE_REDIRONLY);
//    setPulseWidth(MAX30105_PULSEWIDTH_215);
//    max30102_enableSlot(SLOT_RED_LED);
//    setPulseAmplitudeRed(0xFF);
//    setPulseAmplitudeIR(0xFF);
    //max30102_softReset();
    setLEDMode(0x02);
    delay_ms(100);
    setPulseWidth(0x00);
    delay_ms(100);
    setPulseAmplitudeRed(0xFF);
    delay_ms(100);
    setPulseAmplitudeProximity(0xFF);
    delay_ms(100);
    max30102_enableSlot(0x01);
    delay_ms(100);

    int displayState = 0;
//    while(1){
//        //Testing for PartID
//// Reading the part ID
//        displayState = getButtonState();
//        if(displayState == 1){
//            printHeart();
//            
//        }
//        else{
//            printOxygen();
//        }
//         
//    }
    
    return 0;
    
}

