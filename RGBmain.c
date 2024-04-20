/*
 * File:   main.c
 * Author: colej
 *
 * Created on April 20, 2024, 8:01 AM
 */

#include "xc.h"
#include "button_switch.h"
#include <string.h>
#include "rgb_lcd_display.h"

// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)

// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))

void pic24_init() {
    //initializes the system clock to 16MHz and configures all the pins to Digital 
    _RCDIV = 0;
    AD1PCFG = 0xffff;
}

int main(void) {
    pic24_init(); //set clock to 16MHz and all pins digital
    init_I2C1();
    grovergb_init(); //initialize the LCD screen for future use
    initButton();
    
    rgb_clr();
    rgb_home();
    
    //setBacklightColor(255, 0, 0);
    
    while(1){
        //lcd_printChar('H'); 
//        lcd_printStr("Hello World");
//        lcd_cursorReturn();
//        delay_ms(1);
//          
        if(getButtonState()){
            delay_ms(2);
            setCursor(14,0);
            printStr("<B");
            //lcd_cursorReturn();
        } else {
            delay_ms(2);
            setCursor(0,0);
            printStr("O2");
            //lcd_cursorReturn();
        }
    }
    
    return 0;
}
