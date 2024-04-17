/*
 * File:   button.c
 * Author: luke
 *
 * Created on April 16, 2024, 12:08 PM
 */


#include "xc.h"
#include "button.h"

volatile unsigned long int prev_t;
volatile unsigned long int cur_t;
volatile int prevState = 0;
volatile int overflow = 0;

void initButton(){ //button used in lab 4, might need/be easier to have all of this in main
    TRISBbits.TRISB8 = 1;
    CNPU2bits.CN22PUE = 1;
    
    __builtin_write_OSCCONL(OSCCON & 0xbf); //unlock PPS
    RPINR7bits.IC1R = 8; // configure RB8 with INput Capture 1
    __builtin_write_OSCCONL(OSCCON | 0x40); //lock PPS
    
    T2CON = 0;
    T2CONbits.TCKPS = 0b11; // 1:256 pre-scaler
    PR2 = 62499; // 1 second
    TMR2 = 0;
    
    _T2IE = 1;
    _T2IF = 0; // this flag goes off when the timer resets, regardless of
               // whether we use an interrupt or not
    
    T2CONbits.TON = 1;
    
    
    IC1CON = 0; //clear register
    IC1CONbits.ICTMR = 1; //use timer 2
    IC1CONbits.ICM = 0b001; // capture mode every edge
    _IC1IF = 0; // clear IC1 interrupt flag
    _IC1IE = 1; // enable IC1 interrupts
       
}

void __attribute__((interrupt, auto_pav)) _T2Interrupt(void){
    _T2IF = 0;
    overflow++;
}

void __attribute__((__interrupt__, __auto_psv__)) _IC1Interrupt(void){
   _IC1IF = 0; 
   
   
   unsigned long int cur_t = overflow * (uint32_t)(PR2 + 1) + TMR2;
   
   if ((cur_t - prev_t) > 125){
       prev_t = cur_t;
       if (prevState){
           prevState = 0;
       }
       else{
           prevState = 1;
       }
   }
}