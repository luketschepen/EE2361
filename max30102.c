/*
 * File:   PulseOX.c
 * Author: Dhayalan balasubramanian
 *
 * Created on April 10, 2024, 9:26 AM
 */


#include <xc.h>
#include <stdint.h>


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
uint32_t partID;
uint32_t FIFO_WR_PTR_DATA;
#define NUM_SAMPLES_TO_READ 50 //50 / 100 / 200/ 400/ 800


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
}

// Function to read data over I2C
uint32_t I2C_read() {
    I2C2CONbits.RCEN = 1; // Enable receive mode
    while (I2C2CONbits.RCEN == 1);
    return I2C2RCV;        // I2C receive reg
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

void max30102_read_data () {
    I2C_start(); 
    I2C_write(MAX30102_ADDRESS_WRITE); //slave id byte is sent

    
    I2C_write(FIFO_WR_PTR); //register we wish to read 
    I2C_repeated_start();
    
    I2C_write(MAX30102_ADDRESS_READ); //read the slave ID

    
    FIFO_WR_PTR_DATA = I2C_read(); //not sure if it is right.
    
    I2C2CONbits.ACKDT = 1;
        
    I2C2CONbits.ACKEN = 1;   
    while (I2C2CONbits.ACKEN == 1);
    LATAbits.LATA0 = 1;
    I2C_stop();
    
    //central processor evaluates the number of samples to be read
}

void max30102_write_config_SP02() {
    I2C_start();  
    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode

    I2C_write(0x0A); 
    //LATAbits.LATA0 = 1;
    I2C_write(0b00100111);   //00100111    00100001

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

    I2C_write(0b01000000);       

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

    I2C_write(0b00010010);       

    I2C_stop();
}
void max30102_temp_interruptE2(void){
    I2C_start(); 
    I2C_write(MAX30102_ADDRESS_WRITE);
    I2C_write(INTE2); // setup temperature interrupt enable 2
    I2C_write(0x02); // write the data
    I2C_stop();
}

void max30102_temp_init(void){
    I2C_start(); 
    I2C_write(MAX30102_ADDRESS_WRITE);
    I2C_write(TEMP_CONFIG); // write to the temperature address
    I2C_write(0x01); // write the data to set the temperature enable bit to on
    I2C_stop();
}

// Read data from MAX30102 sensor
//void read_SP02_measurements() {
//    uint8_t FIFO_WR_PTR, FIFO_RD_PTR;
//    uint8_t NUM_AVAILABLE_SAMPLES, NUM_SAMPLES_TO_READ;
//    uint32_t LED1_sample, LED2_sample;
//
//    // First transaction: Get the FIFO_WR_PTR
//    I2C_start(); 
//    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode
//    I2C_write(FIFO_WR_PTR); // Send address of FIFO_WR_PTR
//    I2C_repeated_start(); // Repeated start condition
//    I2C_write(MAX30102_ADDRESS_READ); // Send device address + read mode
//    FIFO_WR_PTR = I2C_read(); // Read FIFO_WR_PTR
//    I2C_stop(); 
//
//    // Evaluate the number of samples to be read from the FIFO
//    NUM_AVAILABLE_SAMPLES = (FIFO_WR_PTR - FIFO_RD_PTR + FIFO_DEPTH) % FIFO_DEPTH;
//    NUM_SAMPLES_TO_READ = MIN(NUM_AVAILABLE_SAMPLES, MAX_SAMPLES_TO_READ);
//
//    // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
//    I2C_start(); 
//    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode
//    I2C_write(FIFO_RD_PTR); // Send address of FIFO_DATA
//    I2C_repeated_start(); 
//    I2C_write(MAX30102_ADDRESS_READ); // Send device address + read mode
//    for (int i = 0; i < NUM_SAMPLES_TO_READ; i++) {
//        // Read LED1 sample
//        LED1_sample = I2C_read() << 16;
//        LED1_sample |= I2C_read() << 8;
//        LED1_sample |= I2C_read();
//        
//        // Read LED2 sample
//        LED2_sample = I2C_read() << 16;
//        LED2_sample |= I2C_read() << 8;
//        LED2_sample |= I2C_read();
//        
//        // Process LED1 and LED2 samples
//        process_SP02_data(LED1_sample, LED2_sample); //need to be done next
//    }
//    I2C_stop(); // Stop condition
//
//    // Update FIFO_RD_PTR
//    FIFO_RD_PTR = (FIFO_RD_PTR + NUM_SAMPLES_TO_READ) % FIFO_DEPTH;
//    I2C_start(); // Start condition
//    I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode
//    I2C_write(FIFO_RD_PTR); // Send address of FIFO_RD_PTR
//    I2C_write(FIFO_RD_PTR); // Write FIFO_RD_PTR
//    I2C_stop(); // Stop condition
//}

//void process_SP02_data(LED1_sample, LED2_sample){
//     
//}

void max30102_read_partID() {
        I2C_start();  
        I2C_write(MAX30102_ADDRESS_WRITE); // Send device address + write mode
        

        I2C_write(PART_ID); // Send address of PART_ID
        

        I2C_repeated_start(); // Repeated start condition unnecessary
        
        I2C_write(MAX30102_ADDRESS_READ); // Send device address + read mode
        

        partID = I2C_read(); // Read PART_ID
        
        I2C2CONbits.ACKDT = 1;
        
        I2C2CONbits.ACKEN = 1;

        
        while (I2C2CONbits.ACKEN == 1);
        LATAbits.LATA0 = 1;
        I2C_stop();

}



int main(void) {
    pic24_init();
    max30102_init();
    max30102_write_config_RESET_MODE();
    for (int i = 0; i< 100; i++){
        i++ ;
    }
    max30102_write_config_MODE();
    max30102_write_config_FIFO();
    max30102_write_config_SP02();
    max30102_write_config_RED_PulseAmplitude();
    max30102_write_config_IR_PulseAmplitude();
    max30102_write_config_LED_CONTROL();
    

    
    while(1){
        //Testing for PartID
// Reading the part ID
         asm("nop");

    }
    
    return 0;
    
}



