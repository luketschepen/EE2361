/*
 * File:   PulseOX.c
 * Author: Dhayalan balasubramanian
 *
 * Created on April 10, 2024, 9:26 AM
 */


#include <xc.h>
#include <stdint.h>
#include "spo2_alg.h"
#include "grove-lcd.h"
//#include "button.h"
#include <string.h>
#include <stdio.h>




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
//#define NUM_SAMPLES_TO_READ 50 //50 / 100 / 200/ 400/ 800
//#define NUM_AVAILABLE_SAMPLES

static const uint8_t MAX30105_INTSTAT1 =		0x00;
static const uint8_t MAX30105_INTSTAT2 =		0x01;
static const uint8_t MAX30105_INTENABLE1 =		0x02;
static const uint8_t MAX30105_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06;
static const uint8_t MAX30105_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 		0x08;
static const uint8_t MAX30102_MODECONFIG = 		0x09;
static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT = 		0x1F;
static const uint8_t MAX30105_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 		0xFE;
static const uint8_t MAX30105_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
//static const uint8_t MAX30105_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE = 	0x00;

//static const uint8_t MAX30105_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

//static const uint8_t MAX30105_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

//tatic const uint8_t MAX30105_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

//static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = (byte)~0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =	~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30105_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30105_SHUTDOWN = 		0x80;
static const uint8_t MAX30105_WAKEUP = 			0x00;

static const uint8_t MAX30105_RESET_MASK = 		0xBF;
static const uint8_t MAX30105_RESET = 			0x40;

static const uint8_t MAX30105_MODE_MASK = 		0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 	0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30105_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30105_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30105_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30105_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30105_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30105_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03;

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

volatile unsigned long int overflow = 0;

#define BUFFER_SIZE 32

volatile int readIdxRED = 0;
volatile int writeIdxRED = 0;
volatile int readIdxIR = 0;
volatile int writeIdxIR = 0;
volatile int numElemsInBuffRED = 0;
volatile int numElemsInBuffIR = 0;
volatile unsigned long int RED_Buffer[BUFFER_SIZE];
volatile unsigned long int IR_Buffer[BUFFER_SIZE];


//int32_t bufferLength; //data length
int32_t spo2 = 0;
int8_t validSpo2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;


/*
 * Function: initBuffer
 * ---------------------
 * Initializes the RED circular buffer to zero and resets control variables.
 */
void init_Red_Buffer() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        RED_Buffer[i] = 0; // Set each element of buffer to zero
    }
    writeIdxRED = 0; // Reset currentIndex
}


//PUT FUNCTION
void max30102_RED_putBuff(long int data){ 
    if (numElemsInBuffRED < BUFFER_SIZE){
               RED_Buffer[writeIdxRED++] = data;
               writeIdxRED %= BUFFER_SIZE;
               ++numElemsInBuffRED;
    } 
}

long int max30102_RED_getBuff() {
        long int x;
        x = RED_Buffer[readIdxRED++];
        readIdxRED %= BUFFER_SIZE;
        --numElemsInBuffRED;
        return x;
}



/*
 * Function: initBuffer
 * ---------------------
 * Initializes the SP02 circular buffer to zero and resets control variables.
 */
void init_IR_Buffer() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        IR_Buffer[i] = 0; // Set each element of buffer to zero
    }
    writeIdxIR = 0; // Reset currentIndex
}
void max30102_IR_putBuff(long int data){
    if (numElemsInBuffRED < BUFFER_SIZE){
               IR_Buffer[writeIdxIR++] = data;
               writeIdxIR %= BUFFER_SIZE;
               ++numElemsInBuffIR;
    } 
}

long int max30102_IR_getBuff() {
        long int x;
        x = IR_Buffer[readIdxIR++];
        readIdxIR %= BUFFER_SIZE;
        --numElemsInBuffIR;
        return x;
}


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
    I2C2CONbits.ACKDT = 0;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN == 1);
}

void I2C_NACK() {
    I2C2CONbits.ACKDT = 1;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN == 1);
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



void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void) {
    _T1IF = 0;
    overflow++;
}

void timer1_setup(void) {
    T1CONbits.TON = 0;
    PR1 = 1999; 
    T1CONbits.TCKPS = 1; 
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
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
uint8_t I2C_read_with_params(uint8_t reg);
uint8_t I2C_write_with_params(uint8_t reg, uint8_t val);
void setLEDMode(uint8_t mode);

uint8_t I2C_read_with_params(uint8_t reg)
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

void bitMask(uint8_t reg, uint8_t mask, uint8_t val) {
    // Grab current register context
    uint8_t originalContents = I2C_read_with_params(reg);

    originalContents &= ~mask;  // Invert mask to clear bits

    // Set the desired bits
    originalContents |= (val & mask);  // Only change bits that are 1s in mask
    I2C_write_with_params(reg, originalContents);
}


void setLEDMode(uint8_t mode) {
    // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
    // See datasheet, page 19
    bitMask(MAX30102_MODECONFIG, MAX30105_MODE_MASK, mode);
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
  bitMask(MAX30102_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

void max30102_enableSlot(uint8_t device) {
  bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
     
}

void max30102_shutdown(void) {
    bitMask(MAX30102_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}


void max30102_setADC(uint8_t adcRange) {
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void max30102_setSampleRate(uint8_t sampleRate) {
  // sampleRate: MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void max30102_setFIFOAvg(uint8_t numSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numSamples);
}

void max30102_clearFIFO(void) {
  I2C_write_with_params(MAX30105_FIFOWRITEPTR, 0);
  I2C_write_with_params(MAX30105_FIFOOVERFLOW, 0);
  I2C_write_with_params(MAX30105_FIFOREADPTR, 0);
}

void max30102_enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

void max30102_disableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

void max30102_setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}

void max30102_setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
    I2C_write_with_params(MAX30105_PROXINTTHRESH, threshMSB);
}

//Read the FIFO Write Pointer
uint8_t max30102_getWritePointer(void) {
  return (I2C_read_with_params(MAX30105_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t max30102_getReadPointer(void) {
  return (I2C_read_with_params(MAX30105_FIFOREADPTR));
}

void max30102_readTemp(){
    I2C_write_with_params(MAX30105_DIETEMPCONFIG, 0X01);
    overflow = 0;
    while(overflow < 100) {
        uint8_t temp_Response = I2C_read_with_params(MAX30105_INTSTAT2);
        if ((temp_Response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
        delay_ms(1);
        
    }
    
    int8_t tempInt = I2C_read_with_params(MAX30105_DIETEMPINT);
    uint8_t tempFrac = I2C_read_with_params(MAX30105_DIETEMPFRAC);
    return (float)tempInt + ((float)tempFrac * 0.0625);
}

//float max30102_readTemp_inF() {
//    float temp = max30102_readTemp();
//    if (temp != -999.0) temp = temp *1.8 + 32.0;
//    return (temp);
//}

//interrupt config
uint8_t max30102_int1(){
    return (I2C_read_with_params(MAX30105_INTSTAT1));
}

uint8_t max30102_int2(){
    return (I2C_read_with_params(MAX30105_INTSTAT2));
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
        resetStatus = I2C_read_with_params(MAX30102_MODECONFIG);
    } while (resetStatus & 0x40);  // Wait for the reset bit to clear
}

//uint8_t max30102_available(void){
//  int8_t numberOfSamples = sense.head - sense.tail;
//  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;
//
//  return (numberOfSamples);
//}

//ask cole
void I2C_read_multiple_bytes_params(uint8_t reg, int numSamples) {
    I2C_start();
    I2C_write(MAX30102_ADDRESS_WRITE);

    I2C_write(reg);

    I2C_repeated_start();

    I2C_write(MAX30102_ADDRESS_READ);
    long RED_sample = 0;
    long IR_sample = 0;
    
    for (int i = 0; i < numSamples; i++) {
        RED_sample =  I2C_read() << 16; //msb
        I2C_ACK();
        RED_sample |= I2C_read() << 8; //mid byte
        I2C_ACK();
        RED_sample |= I2C_read(); //lsb
        I2C_ACK();
        max30102_RED_putBuff(RED_sample);
        
        IR_sample =  I2C_read() << 16; //msb
        I2C_ACK();
        IR_sample |= I2C_read() << 8; //mid byte
        I2C_ACK();
        IR_sample |= I2C_read(); //lsb
        I2C_ACK();
        max30102_IR_putBuff(IR_sample);
        
        
 
        if (i + 1 < numSamples) {
                I2C_ACK();            // Not the last byte, send ACK
            } else {
                I2C_NACK();           // Last byte of the last sample, send NACK
            }
    }

    I2C_stop(); 
}

void getRead(){
    I2C_read_multiple_bytes_params(MAX30105_FIFODATA, 32);
    I2C_start();
    I2C_write(MAX30102_ADDRESS_WRITE);
    I2C_write_with_params(MAX30105_FIFOREADPTR, MAX30105_FIFOREADPTR);
    I2C_stop();      
}



void max30102_setup_spo2(){
    max30102_write_config_RESET_MODE();
    max30102_write_config_MODE();
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
    max30102_setFIFOAvg(MAX30105_SAMPLEAVG_8);
    delay_ms(100);
    max30102_enableFIFORollover();
    delay_ms(100);
    max30102_setADC(MAX30105_ADCRANGE_4096);
    delay_ms(100);
    max30102_setSampleRate(MAX30105_SAMPLERATE_800);
    delay_ms(100);
    setPulseAmplitudeIR(0xFF);
    delay_ms(100); 
}


extern void delay_ms(unsigned int ms);
int main(void) {
    pic24_init();
    max30102_init();
    max30102_setup_spo2();
    init_I2C();
    grovelcd_init();
    lcd_clr();
    lcd_cursorReturn();
    //lcd_init();
    //I2C_read_FIFO_data();

    //I2C_read_multiple_bytes_params(MAX30105_FIFODATA, 24);

    //getRead();
    //maxim_heart_rate_and_oxygen_saturation(IR_Buffer, 100, RED_Buffer, &spo2, &validSpo2, &heartRate, &validHeartRate);

    while(1){
            //l//cd_clr();
            char adStr[20];
            char adStr2[20];

            init_Red_Buffer();
            init_IR_Buffer();
      
        //Testing for PartID
// Reading the part ID
            //getRead();
            //maxim_heart_rate_and_oxygen_saturation(IR_Buffer, 32, RED_Buffer, &spo2, &validSpo2, &heartRate, &validHeartRate);
            getRead();
            maxim_heart_rate_and_oxygen_saturation(IR_Buffer, 100, RED_Buffer, &spo2, &validSpo2, &heartRate, &validHeartRate);
            delay_ms(2);
            lcd_printStr("SPO2: ");
            sprintf(adStr, "%d%%", (int)spo2);
            
            lcd_printStr(adStr);
            setCursor(0,1);
            lcd_printStr("Heart: ");
            sprintf(adStr2, "%d", (int)heartRate);
            lcd_printStr(adStr2);
            lcd_printStr(" BPM");
            
            delay_ms(200);
            max30102_clearFIFO();
         
    }
    
    return 0;
    
}
