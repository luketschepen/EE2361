/* 
 * File:   button_switch.h
 * Author: colej
 *
 * Created on April 20, 2024, 7:57 AM
 */

#ifndef BUTTON_SWITCH_H
#define	BUTTON_SWITCH_H

#ifdef	__cplusplus
extern "C" {
#endif

    //initialization functions
    void initButton();
    
    //helper functions
    int getButtonState(); 
    
    //interrupt functions
    void __attribute__((interrupt, auto_pav)) _T2Interrupt(void);
    void __attribute__((__interrupt__, __auto_psv__)) _IC1Interrupt(void);


#ifdef	__cplusplus
}
#endif

#endif	/* BUTTON_SWITCH_H */

