/* 
 * File:   grove-lcd.h
 * Author: colej
 *
 * Created on April 9, 2024, 4:39 PM
 */


#ifndef GROVE_LCD_H
#define	GROVE_LCD_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

void delay_ms(unsigned int ms);
void grovelcd_init(void);
void lcd_cmd(char Package);
void lcd_printChar(char myChar);
void lcd_clr();
void init_I2C(void);
void lcd_printStr(const char s[]);
void lcd_cursorReturn();
void setBacklightColor(int r, int g, int b);
void printColor(char color);

#ifdef	__cplusplus
}
#endif

#endif	/* GROVE_LCD_H */

