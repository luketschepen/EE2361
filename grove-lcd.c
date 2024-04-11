/*
 * File:   grove-lcd.c
 * Author: colej
 *
 * Created on April 9, 2024, 4:37 PM
 */


#include "xc.h"
#include "grove-lcd.h"
#include <string.h>

//uses c-ASM code to delay 1 millisecond
void delay_ms(unsigned int ms){
    while(ms-- > 0){
        asm("repeat #15998");
        asm("nop");
    }
}

void lcd_init(void){
    I2C1BRG = 37; //set to a clock frequency of 400 kHz section 16.3 PIC24 D.S.
    IFS1bits.MI2C1IF = 0; //Clear Interrupt Flag
    I2C1CONbits.I2CEN = 1; //Enable I2C Mode
    
    //initialization from Grove-LCD data-sheet
    delay_ms(100);//wait > 30ms
    
    lcd_cmd(00111000); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    lcd_cmd(00001100); //display ON/OFF Control 00001DCB D = display off/on (0/1) C = cursor off/on (0/1) B = blink off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    lcd_cmd(00000001); //display clear 00000001
    
    delay_ms(2); //wait > 1.53ms
    
    lcd_cmd(00000110); //entry mode set 000001IS I = decrement/increment mode (0/1) S = entire shift off/on (0/1)
 
    delay_ms(1); //doesn't specify but why not //end initialization
}

void lcd_cmd(char Package) {
    //Send START
    
   // IFS3bits.MI2C2IF = 0; //do i need this?? 
    IFS1bits.MI2C1IF = 0; 
    I2C1CONbits.SEN = 1;	//Initiate Start condition
    while(I2C1CONbits.SEN == 1); //Wait for SEN == 0  // SEN will clear when Start Bit is complete
    //set SEN back to 1??? or flag bit (MI2C2IF) to 0???
    
    IFS1bits.MI2C1IF = 0; 
    
    //Send Address and Write Command
    I2C1TRN = 0b00111110; // 8-bits consisting of the slave address and the R/nW bit 0x3E for the grove LCD
    while(IFS1bits.MI2C1IF == 0); //Wait for IFS3bits.MI2C2IF == 1 
    IFS1bits.MI2C1IF = 0;
    
    //send high byte
    I2C1TRN = 0b00000000; // 8-bits consisting of control byte ?? IS THIS RIGHT?? 
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0;
    
    //Send Package
    I2C1TRN = Package; // 8-bits consisting of the data byte
    while(IFS1bits.MI2C1IF == 0);
    
    IFS1bits.MI2C1IF = 0; 
    
    //Send STOP
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN == 1); //Wait for PEN==0 // PEN will clear when Stop bit is complete
}

void lcd_printChar(char myChar){ 
    //consist of a single I2C packet (Start, Address, control, data, STOP) -- will have dif values for RS and CO bit values
    IFS1bits.MI2C1IF = 0; 
    I2C1CONbits.SEN = 1;	//Initiate Start condition
    while(I2C1CONbits.SEN == 1); //Wait for SEN == 0  // SEN will clear when Start Bit is complete
    IFS1bits.MI2C1IF = 0; 
    
    //Send Address and Write Command
    I2C1TRN = 0b00111110; // 8-bits consisting of the slave address and the R/nW bit
    while(IFS1bits.MI2C1IF == 0); //Wait for IFS3bits.MI2C2IF == 1 
    IFS1bits.MI2C1IF = 0;
    
    //send high byte
    I2C1TRN = 0b01000000; // 8-bits consisting of control byte RS = 1
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0;
    
    //Send Package
    I2C1TRN = myChar; // 8-bits consisting of the data byte
    while(IFS1bits.MI2C1IF == 0);
    
    IFS1bits.MI2C1IF = 0; 
    
    //Send STOP
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN == 1); //Wait for PEN==0 // PEN will clear when Stop bit is complete
    
}



