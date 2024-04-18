/* 
 * File:   button.h
 * Author: colej
 *
 * Created on April 17, 2024, 4:31 PM
 */

#ifndef BUTTON_H
#define	BUTTON_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

void initButton();
int getButtonState();  
void __attribute__((interrupt, auto_pav)) _T2Interrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _IC1Interrupt(void);

#ifdef	__cplusplus
}
#endif

#endif	/* BUTTON_H */


