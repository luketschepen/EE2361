/*
 * File:   button_switch.c
 * Author(s): Luke, Cole, Nick
 *
 * Created on April 20, 2024, 7:56 AM
 */

#include "xc.h"
#include "button_switch.h"

// Global variables
volatile unsigned long int prev_t;
volatile unsigned long int cur_t;
volatile int displayHeartRate = 0;
volatile unsigned long int overflow = 0;
volatile int prevState = 0;

/**
 * Initializes the button for usage.
 */
void initButton() {
    // Set RB7 (Pin 6) as input for the button
    TRISBbits.TRISB7 = 1;
    // Enable internal pull-up resistor for RB7
    CNPU2bits.CN23PUE = 1;
    
    // Map Input Capture 1 to RB8 (Pin 7)
    __builtin_write_OSCCONL(OSCCON & 0xbf); // Unlock PPS
    RPINR7bits.IC1R = 7;
    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS
    
    // Configure Timer2 for overflow calculation
    T2CON = 0;
    T2CONbits.TCKPS = 0b11; // 1:256 prescaler
    PR2 = 62499; // 1 second
    TMR2 = 0;
    
    // Enable Timer2 interrupt
    _T2IE = 1;
    _T2IF = 0;
    T2CONbits.TON = 1;
    
    // Configure Input Capture 1
    IC1CON = 0;
    IC1CONbits.ICTMR = 1; // Use Timer2
    IC1CONbits.ICM = 1; // Capture mode every edge
    _IC1IF = 0;
    _IC1IE = 1;
}

/**
 * Timer2 interrupt service routine.
 */
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void){
    _T2IF = 0;
    overflow++;
}

/**
 * Input Capture 1 interrupt service routine.
 */
void __attribute__((__interrupt__, __auto_psv__)) _IC1Interrupt(void){
   _IC1IF = 0; 
   
   // Calculate current time
   unsigned long int cur_t = overflow * (uint32_t)(PR2 + 1) + TMR2;
   
   // Check button press duration
   if ((cur_t - prev_t) > 130){
       prev_t = cur_t;
       
       // Toggle button state on press
       if(prevState){ // Previous state: Pressed, current state: Released
           prevState = 0;
       } else { // Previous state: Released, current state: Pressed
           prevState = 1;
           // Toggle displayHeartRate flag
           displayHeartRate = !displayHeartRate;
       }
   }
}

/**
 * Get the state of the button.
 * @return 1 if heart rate should be displayed, 0 otherwise.
 */
int getButtonState() {
    return displayHeartRate;
}
