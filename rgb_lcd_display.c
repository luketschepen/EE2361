/*
 * Date: 4/9/2024
 * Name(s): Cole Jaeger, Dhayalan Balasubarmanian, Luke Tschepen, Nickk
 * Project Name: Blood Oxygen Sensor
 * Program Description: This program initializes the Grove RGB/LCD display
 * to be compatible with the PIC24 using I2C communication. It contains various
 * functions to manipulate and display different text characters and colors on
 * the display.
 * References/Code Citations: Arduino Grove RGB file, JHD data sheet
 * 
 */

#include "xc.h"
#include <string.h>
#include "rgb_lcd_display.h"
#include "button_switch.h"
//#include "I2C1.h"

//ARE WE USING THESE I2C functions IN HERE OR seperate file???
void delay_ms(unsigned int ms){
    //uses c-ASM code to delay 1 millisecond
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
    // I2C communication protocol betweem the PIC24 and Grove LCD
    // Used to send a command with data to the LCD 
    // Package: an 8-bit command
    
    I2C_start();
    
    I2C_write(LCD_SLAVE_ADDR);
    
    I2C_write(COMMANDBIT);
    
    I2C_write(Package);
    
    I2C_stop();
    
}

void setReg(unsigned char reg, unsigned char dat) {
    //I2C protocol used to set the LED registers for the RGB display
    //reg: desired LED register
    //data: binary number representing a color from 0-255
    I2C_start();
    I2C_write(RGB_SLAVE_ADDR);
    I2C_write(reg);
    I2C_write(dat);
    I2C_stop();
}

void setRGB(unsigned char r, unsigned char g, unsigned char b){
    //calls the setReg function for each RGB color
    //r: red value from 0-255
    //g: green value from 0-255
    //b: blue value from 0-255
    setReg(0x04, r);
    setReg(0x03, g);
    setReg(0x02, b);
}

void rgb_clr(){
    //clears the text on the display
    rgb_cmd(DISPLAY_CLR);
    delay_ms(2000);
}

void rgb_home() {
    //returns the cursor to the initial position (0,0)
    rgb_cmd(LCD_RETURNHOME);
    delay_ms(2000);
}

void noDisplay() {
    //turns the display off
    rgb_cmd(DISPLAY_CONTROL);
}

void display() {
    //turns the display on
    rgb_cmd(DISPLAY_CONTROL | LCD_DISPLAYON);
}

void blinkLED(void){
    //flashes the specified LED registers
    setReg(0x07, 0x17);
    setReg(0x06, 0x7F);
}

void noBlinkLED(){
    //stops the flashing of the LED registers
    setReg(0x07, 0x00);
    setReg(0x06, 0xFF);
}

void setColorAll() {
    //turns all colors off
    setRGB(0,0,0);
}

void setColorWhite() {
    //turns the screen white
    setRGB(255, 255, 255);
}

void setColorRed() {
    //turns the screen red
    setRGB(255, 0, 0);
}

void setCursor(uint8_t x, uint8_t y){
    //sets the cursor to the desired position on the screen
    //x: column (0-15)
    //y: row (0-1)
    rgb_cmd(0x0 | 0x80 | (0x40 * y + x));
}

void grovergb_init(void){
    //initialization from Grove-LCD/RGB data-sheet and Arduino reference code
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
    //prints a string to the display
    //s: an array of characters representing a string
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
    //prints a character to the display
    //myChar: a character
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

