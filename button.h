/* 
 * File:   button.h
 * Author: Luke
 *
 *
 */

#ifndef BUTTON_H
#define	BUTTON_H

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

#endif	

