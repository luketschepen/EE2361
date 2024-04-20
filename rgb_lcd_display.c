/*
 * File:   grove-rgb.c
 * Author(s): 
 *
 * Created on April 9, 2024, 4:37 PM
 */

#include "xc.h"
#include <string.h>
#include "rgb_lcd_display.h"
#include "button_switch.h"
//#include "I2C1.h"

//uses c-ASM code to delay 1 millisecond

//ARE WE USING THESE I2C functions IN HERE OR seperate file???
void delay_ms(unsigned int ms){
    while(ms-- > 0){
        asm("repeat #15998");
        asm("nop");
    }
}

void init_I2C1(void){
    I2C1BRG = 37; //set to a clock frequency of 400 kHz section 16.3 PIC24 D.S.
    IFS1bits.MI2C1IF = 0; //Clear Interrupt Flag
    I2C1CONbits.I2CEN = 1; //Enable I2C Mode
}

void I2C_start() {
    I2C1CONbits.SEN = 1;   // Initiate start condition
    while (I2C1CONbits.SEN);
};

void I2C_stop() {
    I2C1CONbits.PEN = 1;   // Initiate stop condition
    while (I2C1CONbits.PEN);
}
// Function to write data over I2C
void I2C_write(uint8_t data) {
    I2C1TRN = data;        //  I2C transimit reg
    while (I2C1STATbits.TRSTAT); // Wait for transmission to complete
}

void rgb_cmd(char Package) {
    //Send START
    I2C_start();
    
    I2C_write(LCD_SLAVE_ADDR);
    
    I2C_write(COMMANDBIT);
    
    I2C_write(Package);
    
    I2C_stop();
    
}

void rgb_write(char Package){
    
    I2C_write(WRITEBIT);
    
    I2C_write(Package);
    
}

void setReg(unsigned char reg, unsigned char dat) {
    I2C_start();
    I2C_write(RGB_SLAVE_ADDR);
    I2C_write(reg);
    I2C_write(dat);
    I2C_stop();
}

void setRGB(unsigned char r, unsigned char g, unsigned char b){
    setReg(0x04, r);
    setReg(0x03, g);
    setReg(0x02, b);
}

void rgb_clr(){
    rgb_cmd(DISPLAY_CLR);
    delay_ms(2000);
}

void rgb_home() {
    rgb_cmd(LCD_RETURNHOME);
    delay_ms(2000);
}

void noDisplay() {
    rgb_cmd(DISPLAY_CONTROL);
}

void display() {
    rgb_cmd(DISPLAY_CONTROL | LCD_DISPLAYON);
}

void blinkLED(void){
    setReg(0x07, 0x17);
    setReg(0x06, 0x7F);
}

void noBlinkLED(){
    setReg(0x07, 0x00);
    setReg(0x06, 0xFF);
}

void setColorAll() {
    setRGB(0,0,0);
}

void setColorWhite() {
    setRGB(255, 255, 255);
}

void setColorRed() {
    setRGB(255, 0, 0);
}

void setCursor(uint8_t x, uint8_t y){
    rgb_cmd(0x0 | 0x80 | (0x40 * y + x));
}

void grovergb_init(void){
    //initialization from Grove-LCD data-sheet
    delay_ms(50);//wait > 30ms
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(FUNCTION_SET | LCD_8BITMODE | LCD_2LINE); // function set 0011NFXX N = line mode 1/2 (0/1) F = display off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_cmd(DISPLAY_CONTROL | LCD_DISPLAYON); //display ON/OFF Control 00001DCB D = display off/on (0/1) C = cursor off/on (0/1) B = blink off/on (0/1)
    
    delay_ms(1); //wait > 39 usec
    
    rgb_clr(); //display clear 00000001
    
    delay_ms(2); //wait > 1.53ms
    
    rgb_cmd(ENTRY_MODE_SET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT); //entry mode set 000001IS I = decrement/increment mode (0/1) S = entire shift off/on (0/1)
 
    delay_ms(1); //doesn't specify but why not //end initialization
    
    setReg(REG_MODE1, 0);
    
    setReg(REG_OUTPUT, 0xFF);
    
    setReg(REG_MODE2, 0x20);
    
    setColorWhite();
}

void printStr(const char s[]){
    int len = strlen(s);
    
    I2C_start();
    I2C_write(LCD_SLAVE_ADDR);
    
    for(int i = 0; i < len; i++ ){
       
        I2C_write(COMMANDBIT | WRITEBIT);
        I2C_write(s[i]); // 8-bits consisting of the data byte
    }
    I2C_write(WRITEBIT);

    I2C_stop();
    
}

void printChar(char myChar){ 
    I2C_start();
    I2C_write(LCD_SLAVE_ADDR);
    I2C_write(WRITEBIT);
    I2C_write(myChar);
    I2C_stop();
}

void printHeartRate(){
    //get heart rate from sensor file
    //convert heart rate to a string
//     char heartRateStr[20];
//     sprintf(heartRateStr, "%2.2fV", getHeartRate());
//     lcd_clr();
//     lcd_cursorReturn(0,0);
//     lcd_printStr(heartRateStr);
    //lcd_printStr(heart rate);
    //printHeart();
}

void printOxygen(){
    //get oxygen from sensor file
    //convert oxygen to a string
//     char oxygenStr[20];
//     sprintf(oxygenStr, "%2.2fV", getOxygen());
//     lcd_clr();
//     lcd_cursorReturn(0,0);
//     lcd_printStr(oxygenStr);
    //lcd_printStr(oxygen);
    //print O2 after??
}

void printHeart(){
    //lcd_setCursor()
    //lcd_cursorReturn();
//    lcd_printStr("<3 Heart Rate <3");
//    setCursor(6,1);
//    printHeartRate(); //print heart rate from sensor
}

void printO2() {
    //    lcd_clr();
//    lcd_cursorReturn();
//    lcd_printStr("Oxygen Percent");
//    setCursor(0,1);
//    lcd_printStr("SPO2:");
//    printOxygen(); //print oxygen from sensor
//    lcd_printChar('%');
}

