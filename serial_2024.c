#define _XTAL_FREQ 4000000UL  

// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include "my_ser.h"

#define STARTVALUE  3036


int heaterState = 0;
int coolerState=0;
int relay1=0;
int relay2=0;

char binaryString[9]; // 8 bits + null terminator
void initialize(void) {
    
    ADCON0 = 0;
    ADCON1 = 0b00001100; 
    TRISB = 0xFF; 
    TRISC = 0x80; 
    PORTC =0;
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
    TRISB = 0b00111001;
    PORTD = 0x00;   
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    INTCONbits.GIE = 1;  
    INTCONbits.PEIE = 1;    
 
}
void turnEverythingOff() {
    PORTCbits.RC5 = 0;  // Assuming RC5 is connected to Heater
    PORTCbits.RC2 = 0;  // Assuming RC2 is connected to Fan
    PORTCbits.RC0 = 0;  // Turn off Relay1
    PORTEbits.RE0 = 0;  // Turn off Relay2
    relay1=0;
    relay2=0;
    coolerState = PORTCbits.RC2;
    heaterState =PORTCbits.RC5;
    LATD = 0x0000 ; // for leds 
}
void toggleLEDs2345() {
    LATD ^= 0x3C;
}

void toggleLEDs0167() {
    LATD ^= 0xC3;  
}
void turnOFFHeaterTurnOnCooler(){
     
      PORTCbits.RC5 =0;
      PORTCbits.RC2 =1;
      coolerState = 1;
      heaterState = 0;
}
void turnONHeaterTurnOffCooler(){
     
      PORTCbits.RC5 =1;
      PORTCbits.RC2 =0;
      coolerState =0;
      heaterState =1;
}
void toggleRelay1(){
     relay1=!relay1;
     PORTCbits.RC0 = relay1;
      
}
void toggleRelay2(){
    relay2=!relay2 ;
        PORTEbits.RE0 = relay2; 
        
}
void updateRelayStatesFromHardware() {
    relay1 = PORTCbits.RC0;
    relay2 = PORTEbits.RE0;
}

void delay_ms(unsigned int n)
{
    int i;
    for (i=0; i < n; i++){
         __delaywdt_ms(1) ; 
    }
}
void intToBinaryString(unsigned int value, char* buffer, int bufferSize) {
    buffer[bufferSize - 1] = '\0'; // Null-terminate the string
    for (int i = bufferSize - 2; i >= 0; i--) {
        buffer[i] = (value & 1) ? '1' : '0';
        value >>= 1;
    }
}
void sendBinaryStatus() {
    intToBinaryString(PORTD, binaryString, sizeof(binaryString));
    char message[40];
    sprintf(message, "LEDs Bin: %s\n", binaryString);
    send_string_no_lib((unsigned char *)message);
}
void main(void) {
    char Buffer[32]; // for sprintf
    char x;
    initialize();
    setupSerial();
    send_string_no_lib((unsigned char *) "Welcome Sama & Sadeen\n");
  
    while (1) {
    CLRWDT();

        if (!PORTBbits.RB0) {
            turnEverythingOff();
            delay_ms(250);// Button debounce
            }
        if (!PORTBbits.RB1) {
            turnOFFHeaterTurnOnCooler();
            delay_ms(250);// Button debounce
            }
        if (!PORTBbits.RB2) {
            turnONHeaterTurnOffCooler();
            delay_ms(250);// Button debounce
            }
        if (!PORTBbits.RB3) {
            toggleLEDs0167();
            delay_ms(250); // Button debounce
        }
        if (!PORTBbits.RB4) {
            toggleLEDs2345();
            delay_ms(250); // Button debounce
        }
        if (!PORTBbits.RB5) {
            toggleRelay1();
            toggleRelay2();
            delay_ms(250); // Button debounce
        }
    
    if(is_byte_available()){
    char ch = read_byte_no_lib();
    if (ch == 'G') {
                        updateRelayStatesFromHardware();
    sprintf(Buffer, 
    "\nHeater:%s,Cooler:%s,Relay1:%s,Relay2:%s\n",
    heaterState ? "ON" : "OFF",coolerState ? "ON" : "OFF",
    relay1 ? "ON" : "OFF", relay2 ? "ON" : "OFF");
	send_string_no_lib(Buffer);
    ch='\0';
            }
    
    else if (ch == 'g') {
        sprintf(Buffer, "LEDs: 0x%02X\n", PORTD); // D7-D0
        send_string_no_lib(Buffer);
        sendBinaryStatus();
        ch='\0';
            }
    else if (ch == 'H') {
        turnONHeaterTurnOffCooler();
        ch='\0';
            }
    else if (ch == 'h') {
       turnOFFHeaterTurnOnCooler();
              ch='\0';
            }
      
      
    else if (ch == 'D') {
        toggleLEDs2345();
                ch='\0';
            } 
    else if (ch == 'd') {
            toggleLEDs0167();
                ch='\0';
            }
    else if (ch == 'c') {
          relay1=0;
                    PORTCbits.RC0 = relay1;  // Turn off Relay1
          ch='\0';
            }
    else if (ch == 'C') { 
        relay1=1;
          PORTCbits.RC0 = relay1;  // Turn on Relay1   
           ch='\0';
            }
         
    else if (ch == 'r') {
    
    relay2=0;
    PORTEbits.RE0 = relay2;  // Turn off 
                ch='\0';
            }
    else if (ch == 'R') {
        relay2=1;
    PORTEbits.RE0 =relay2;  // Turn on Relay2
    
          ch='\0';
            }
    else if (ch == 'z') {
              turnEverythingOff();
                ch='\0';
            }
             
             
    }
        
    }
    return;
}
