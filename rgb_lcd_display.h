/* 
 * File:   rgb_lcd_display.h
 * Author: colej
 *
 * Created on April 20, 2024, 7:25 AM
 */

#ifndef RGB_LCD_DISPLAY_H
#define	RGB_LCD_DISPLAY_H

// Device I2C Address
#define LCD_SLAVE_ADDR (0x3E << 1)
#define RGB_SLAVE_ADDR (0x62 << 1)
#define RGB_ADDRESS_V5 0x30

//color define 
#define WHITE 0
#define RED 1
#define GREEN 2
#define BLUE 3

#define REG_MODE1 0x00
#define REG_MODE2 0x01
#define REG_OUTPUT 0x08

//commands
#define FUNCTION_SET 0x20 //2 line mode, display off 
#define DISPLAY_CONTROL 0x08 //display on, cursor off, blink off 
#define DISPLAY_CLR 0x01 //clear display
#define ENTRY_MODE_SET 0x04 //decrement mode, shift off
#define LCD_RETURNHOME 0x02
#define LCD_CURSORSHIFT 0x10
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

//flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

//flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

//flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define WRITEBIT 0x40
#define COMMANDBIT 0x80


#ifdef	__cplusplus
extern "C" {
#endif

//Basic helper functions
void rgb_clr();
void rgb_home();
void noDisplay();
void display();
void blinkLED(void);
void noBlinkLED();

//Initialization functions
void grovergb_init(void);
void rgb_cmd(char Package);
void rgb_write(char Package);
void setReg(unsigned char reg, unsigned char dat);

//Functions to change the color of the display
void setRGB(unsigned char r, unsigned char g, unsigned char b);
void setColorAll();
void setColorWhite();
void setColorRed();

//Functions used to print a character or string
void printStr(const char s[]);
void printChar(char myChar);
void setCursor(uint8_t x, uint8_t y);

//Functions used to print the heart rate and oxygen reads
void printHeartRate();
void printOxygen();
void printHeart();
void printO2();

//I2C1 functions
void delay_ms(unsigned int ms);
void init_I2C1(void);
void I2C_start();
void I2C_stop();
void I2C_write(uint8_t data);

#ifdef	__cplusplus
}
#endif

#endif	/* RGB_LCD_DISPLAY_H */

