// Serial Code
// Karthik Gangadhar

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// variables for getCommand
uint32_t MAX_CHARS = 0X50;
//#define BACK_SPACE          (*((volatile uint32_t *)0x08))
//#define SPACE               (*((volatile uint32_t *)0x20))
//#define CARRIAGE_RETURN     (*((volatile  uint32_t *)0x0D))

char strn;
char * strp = &strn;

//parseStr valiables
uint8_t argc = 0;
uint8_t * pos[20];
char * types[20];


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
//void waitPbPress()
//{
//    while(PUSH_BUTTON);
//}

// Initialize Hardware
void initSerialHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// Blocking function that returns with serial data entered by user
void getCommand()
{
    uint8_t count= 0;
    while(count < 0x50){
        char c = getcUart0();
        if(c == 0x08){ // if character is BACK_SPACE
            if(count > 0){
                GREEN_LED = 0;
                RED_LED ^= 1;
                count = count - 1;
            }
        }else if(c == 0x0D){ //CARRIAGE_RETURN
            strp[count] = 0x0;
            break;
        }else if( c >= 0x20 ){ // character value greater than SPACE.
            strp[count++] = c;
            GREEN_LED ^= 1;
            RED_LED = 0;
        }
    }
}

// Blocking function that returns with serial data entered by user
void parseStr()
{
    uint8_t count= 0;
    uint8_t cmdLength = strlen(strp);
    uint8_t cnt = 0;

    while(count < cmdLength){
        if((strp[count] >= 0x30 && strp[count] <= 0x39) || (strp[count] >= 0x41 && strp[count] <= 0x59) || (strp[count] >= 0x61 && strp[count] <= 0x7A)){
            count += 1;
        }
        else{
            strp[count++] = 0x20;
        }
    }

//    uint8_t argCount = 0;
    while(cnt < cmdLength){
       if(cnt == 0 && strp[cnt] > 0x20 ){
           pos[argc] = cnt;
           // if chara is between 0 to 9
           if( strp[cnt] >= 0x30 && strp[cnt] <= 0x39 ){
               types[argc] = "number";
           }
           // if its between Ascii characters
           else if(strp[cnt] >= 0x41){
               types[argc] = "string";
           }
           argc += 1;
       }else if(strp[cnt] == 0x20 && strp[cnt+1] > 0x20){
           pos[argc] = cnt + 1;
           // if character is between 0 to 9
           if( strp[cnt+1] >= 0x30 || strp[cnt+1] <= 0x39 ){
               types[argc] = "number";
           }
           // if its between Ascii characters
           else if(strp[cnt+1] >= 0x41){
               types[argc] = "string";
           }
           argc += 1;
       }
       cnt += 1;
    }

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void serialCheck(void)
{
    // Initialize hardware
    initSerialHw();

    // Display greeting
    putsUart0("Enter Commands\r\n");

    while(1)
    {
        getCommand();
        putsUart0(strp);
        putsUart0("\r\n");
        parseStr();
        putsUart0("\r\n");
        putsUart0(strp);
        putsUart0("\r\n");

        // reset global fields
        argc = 0;
        uint8_t i = 0;
        for(i=0; i<5; i++){
            pos[i] = 0x00;
            types[i] = 0x00;
        }
    }
}
