
/****** ASEN 4/5067 Lab 6 ******************************************************
 * Author: YOUR NAME HERE
 * Date  : DATE HERE
 *
 * Updated for XC8
 * 
 * Description
 * On power up execute the following sequence:
 *      RD5 ON for 0.5s +/- 10ms then off
 *      RD6 ON for 0.5s +/- 10ms then off
 *      RD7 ON for 0.5s +/- 10ms then off
 * The following then occurs forever:
 *      RD4 blinks: 100ms +/- 10ms ON, then 900ms +/- 10ms OFF
 *      LCD Displays the following lines:
 *          'T=xx.x C'
 *          'PT=x.xxV'
 *      Where the 'x' is replaced by a digit in the measurement.
 *          Temperature data must be calculated / displayed with one digit to
 *          the right of the decimal as shown.  The sensor itself can have
 *          errors up to +/- 5 degrees Celsius.
 *          Potentiometer data must be calculated / displayed with two digits
 *          to the right of the decimal as shown.
 *          These measurements must be refreshed at LEAST at a frequency of 5Hz.
 *      USART Commands are read / executed properly. '\n' is a Line Feed char (0x0A)
 *          ASEN 4067:
 *              'TEMP\n'     - Transmits temperature data in format: 'XX.XC'
 *              'POT\n'      - Transmits potentiometer data in format: X.XXV'
 *          ASEN 5067: Same as ASEN 4067, plus two additional commands
 *              'CONT_ON\n'  - Begins continuous transmission of data over USART
 *              'CONT_OFF\n' - Ends continuous transmission of data over USART
 *
 *              Continuous transmission should output in the following format:
 *                  'T=XX.XC; PT = X.XXV\n'
 *      DAC is used to output analog signal onto RA5 with jumper cables. 
 *          ASEN 4067:
 *              Potentiometer voltage is converted from a digital value to analog 
 *              and output on the DAC. 
 *          ASEN 5067: 
 *              A 0.5 Hz 0-3.3V triangle wave is output on the DAC. 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
//const char LCDRow1[] = {0x80,'T','=',0x00};
char LCDRow1[] = {0x80,'T','=','X','X','.','X','C',0x00};
char LCDRow2[] = {0xC0,'P','T','=',0x00};
unsigned int Alive_count = 0;
//unsigned int led_max = 2;
//unsigned int toggle = 0xFF;
//unsigned char alive_times[] = {0x24, 0x46,0xE7, 0x8C};

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
unsigned short get_temperature(void);
unsigned short get_pot(void);
void update_display(unsigned short temperature, unsigned short pot);

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
     unsigned short temp;
     unsigned short pot;
     
      while(1) {
        // Sit here for ever
          temp = get_temperature();
          pot = get_pot();
          update_display(temp,pot);
          
        
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    TRISD  = 0b00001111;
    LATD = 0;
    TRISC  = 0b10010011;
    LATC = 0;
    
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;              // 
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero


    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);
    DisplayC(LCDRow2);
    
    LATDbits.LATD5 = 1;
    __delay_ms(500);
    LATDbits.LATD5 = 0;
    
    LATDbits.LATD6 = 1;
    __delay_ms(500);
    LATDbits.LATD6 = 0;
    
    LATDbits.LATD7 = 1;
    __delay_ms(500);
    LATDbits.LATD7 = 0;

    // Initializing TMR0
    T0CON = 0b00000101;             // 16-bit, 64x prescaler
    TMR0L = 0x8C;
    TMR0H = 0xE7;

    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    ADCON0 = 0x02;
    
    ADCON1 = 0x00;
    ADCON2 = 0b10010101;
    
    ANCON0 = 0x05;
    TRISA = 0x05;
    ADCON0 = 0x03;
    __delay_ms(1);
    unsigned char t = get_temperature();
    
    
    
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR0IF and CCP1IF are clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if( INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }
        // Save temp copies of WREG, STATUS and BSR if needed.
        break;      // Supports RETFIE automatically
    }
}


/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
void TMR0handler() {
        LATD = LATD^0x10;

        if(PORTDbits.RD4){
            TMR0L = 0x46; //900 ms
            TMR0H =  0x24;
            INTCONbits.TMR0IF = 0;
            return;
        }
        TMR0L = 0x8C; //100 ms
        TMR0H = 0xE7;

           
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}

unsigned short get_temperature(void){
    unsigned short temperature;
    unsigned short temph;
    ADCON0bits.CHS0 =1;
    ADCON0bits.CHS1 =1;
    //throw away
    ADCON0bits.GO  = 1;
    __delay_ms(1);
    while(1){
        if(ADCON0bits.DONE ==0){
            
            break;
        }
    }
    __delay_ms(1);
    ADCON0bits.GO  = 1;
    while(1){
        if(ADCON0bits.DONE ==0){
                temperature = ADRESL;
                temph = ADRESH & 0x0F; //0b00001111
                temperature =  temperature | (temph<<8)  ;
                
            break;
        }
    }    
    
    return temperature;
    
}
    
unsigned short get_pot(void){
    unsigned short pot;
    
    
    return pot;
}
    
void update_display(unsigned short temperature, unsigned short pot){
    //806 uv /bin
     float voltage = temperature*806; //uv
     //voltage = 0;
     float temp_c = -55+voltage/10000; 
    temp_c = temp_c*10;
    
    char temp_str[10];
    sprintf(temp_str, "%f", voltage);
    
    LCDRow1[3]= temp_str[0];
    LCDRow1[4]= temp_str[1];
    LCDRow1[6]= temp_str[2];
    DisplayC(LCDRow1);
    
    
    
    
}