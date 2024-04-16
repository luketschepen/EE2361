/*
 * File:   grove-lcd.c
 * Author: colej
 *
 * Created on April 9, 2024, 4:37 PM
 */


#include "xc.h"
#include "grove-lcd.h"
#include <string.h>
#include <stdio.h>

#define LCD_SLAVE_ADDR 0x3E //0b00111110
#define RGB_SLAVE_ADDR 0x62 //0b01100010

#define FUNCTION_SET 0x38 //2 line mode, display off //0b00111000
#define DISPLAY_CONTROL 0x0C //display on, cursor off, blink off //0b00001100
#define DISPLAY_CLR 0x01 //clear display //0b00000001
#define ENTRY_MODE_SET 0x06 //decrement mode, shift off //0b00000110
#define CURSOR_RETURN 0x02 //return cursor to start //0b00000010

#define CONTROL_BIT 0x40 
#define RS_BIT 0x80

//uses c-ASM code to delay 1 millisecond
void delay_ms(unsigned int ms){
    while(ms-- > 0){
        asm("repeat #15998");
        asm("nop");
    }
}

void setBacklightColor(int r, int g, int b){
    lcd_cmd(0x4D);
    printColor(r);
    printColor(g);
    printColor(b);
}

void printColor(char color){
        //Send START
    IFS1bits.MI2C1IF = 0; 
    I2C1CONbits.SEN = 1;	//Initiate Start condition
    while(I2C1CONbits.SEN == 1); //Wait for SEN == 0  // SEN will clear when Start Bit is complete
    IFS1bits.MI2C1IF = 0; 
    
    //Send Address and Write Command
    I2C1TRN = RGB_SLAVE_ADDR << 1; // 8-bits consisting of the slave address and the R/nW bit 0x3E for the grove LCD
    while(IFS1bits.MI2C1IF == 0); //Wait for IFS3bits.MI2C2IF == 1 
    IFS1bits.MI2C1IF = 0;
    
    I2C1TRN = 0b01000000; // 8-bits consisting of control byte 
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0;
    
    //Send Package
    I2C1TRN = color; // 8-bits consisting of the data byte
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0; 
    
    //Send STOP
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN == 1); //Wait for PEN==0 // PEN will clear when Stop bit is complete
}

void init_I2C(void){
    I2C1BRG = 37; //set to a clock frequency of 400 kHz section 16.3 PIC24 D.S.
    IFS1bits.MI2C1IF = 0; //Clear Interrupt Flag
    I2C1CONbits.I2CEN = 1; //Enable I2C Mode
}

void grovelcd_init(void){
    //initialization from Grove-LCD data-sheet
    delay_ms(50);//wait > 30ms
    
    lcd_cmd(FUNCTION_SET); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    lcd_cmd(FUNCTION_SET); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    lcd_cmd(FUNCTION_SET); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    lcd_cmd(FUNCTION_SET); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    lcd_cmd(DISPLAY_CONTROL); //display ON/OFF Control 00001DCB D = display off/on (0/1) C = cursor off/on (0/1) B = blink off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    lcd_clr(); //display clear 00000001
    
    delay_ms(2); //wait > 1.53ms
    
    lcd_cmd(ENTRY_MODE_SET); //entry mode set 000001IS I = decrement/increment mode (0/1) S = entire shift off/on (0/1)
 
    delay_ms(1); //doesn't specify but why not //end initialization
}

void lcd_cmd(char Package) {
    //Send START
    IFS1bits.MI2C1IF = 0; 
    I2C1CONbits.SEN = 1;	//Initiate Start condition
    while(I2C1CONbits.SEN == 1); //Wait for SEN == 0  // SEN will clear when Start Bit is complete
    IFS1bits.MI2C1IF = 0; 
    
    //Send Address and Write Command
    I2C1TRN = LCD_SLAVE_ADDR << 1; // 8-bits consisting of the slave address and the R/nW bit 0x3E for the grove LCD
    while(IFS1bits.MI2C1IF == 0); //Wait for IFS3bits.MI2C2IF == 1 
    IFS1bits.MI2C1IF = 0;
    
    I2C1TRN = 0b00000000; // 8-bits consisting of control byte 
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

void lcd_clr(){
    lcd_cmd(DISPLAY_CLR);
}

void lcd_printChar(char myChar){ 
    //consist of a single I2C packet (Start, Address, control, data, STOP)
    IFS1bits.MI2C1IF = 0; 
    I2C1CONbits.SEN = 1;	//Initiate Start condition
    while(I2C1CONbits.SEN == 1); //Wait for SEN == 0  // SEN will clear when Start Bit is complete
    IFS1bits.MI2C1IF = 0; 
    
    //Send Address and Write Command
    I2C1TRN = LCD_SLAVE_ADDR << 1; // 8-bits consisting of the slave address and the R/nW bit
    while(IFS1bits.MI2C1IF == 0); //Wait for IFS3bits.MI2C2IF == 1 
    IFS1bits.MI2C1IF = 0;
    
    //send high byte - is RS first or second bit
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

void lcd_cursorReturn(){
    lcd_cmd(CURSOR_RETURN);
}

void lcd_printStr(const char s[]){
    int len = strlen(s);
    I2C1CONbits.SEN = 1; //Initiate Start condition
 
    //Wait for SEN == 0  // SEN will clear when Start Bit is complete
    while(I2C1CONbits.SEN == 1);
    IFS1bits.MI2C1IF = 0;
    
    I2C1TRN = LCD_SLAVE_ADDR << 1; // 8-bits consisting of the slave address and the R/nW bit
    while(IFS1bits.MI2C1IF == 0);

    IFS1bits.MI2C1IF = 0;
    for(int i = 0; i < len; i++ ){
       
        IFS1bits.MI2C1IF = 0;
        I2C1TRN = 0b11000000; // 8-bits consisting of control byte /w RS=1
        while(IFS1bits.MI2C1IF == 0);

        IFS1bits.MI2C1IF = 0;

        I2C1TRN = s[i]; // 8-bits consisting of the data byte
        while(IFS1bits.MI2C1IF == 0);
    }
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = 0b01000000; // 8-bits consisting of control byte /w RS=0
    while(IFS1bits.MI2C1IF == 0);

    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN==1);// PEN will clear when Stop bit is complete
    IFS1bits.MI2C1IF = 0;
    
}

void printHeartRate(){
    //get heart rate from sensor file
    //convert heart rate to a string
     char heartRateStr[20];
     sprintf(heartRateStr, "%6.4fV", getHeartRate());
     lcd_clr();
     lcd_cursorReturn(0,0);
     lcd_printStr(heartRateStr);
    //lcd_printStr(heart rate);
    //printHeart();
}

void printOxygen(){
    //get oxygen from sensor file
    //convert oxygen to a string
     char oxygenStr[20];
     sprintf(oxygenStr, "%6.4fV", getOxygen());
     lcd_clr();
     lcd_cursorReturn(0,0);
     lcd_printStr(oxygenStr);
    //lcd_printStr(oxygen);
    //print O2 after??
}

void printHeart(){
    //lcd_setCursor()
    //lcd_printStr("<3");
}

void printO2() {
    //lcd_setCursor()
    //lcd_printStr("O2");
}

void setCursor(int x, int y){
    //set Cursor to desired x(column) y(row)
    lcd_cmd(0x0 | RS_BIT | (CONTROL_BIT * y + x));
}

void switchScreen(){
    //clear display
    //heart rate = getCurrent;
    //if heart rate = 1 
        //printOxygen())
        //heart rate = 0
    //else 
        //printHeartRate())
}

void redAlert(){
    //constantly read heart rate or blood oxygen
    //if either > said amount
        //RGB Backlight -> red
    //else 
        //clear color
}



