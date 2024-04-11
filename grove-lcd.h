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
void lcd_init(void);
void lcd_cmd(char Package);
void lcd_printChar(char myChar);

#ifdef	__cplusplus
}
#endif

#endif	/* GROVE_LCD_H */

