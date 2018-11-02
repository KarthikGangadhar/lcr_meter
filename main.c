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
#include <ctype.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// variables for getCommand
char strn;
char * strp = &strn;

//parseStr variables
uint8_t argc = 0;
char * commandArgs[80];

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
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOA;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure circuit output pins
    GPIO_PORTA_DIR_R = 0x20; //bit 5 for output MEAS_C
    GPIO_PORTA_DR2R_R = 0x20; // set drive strength to 2mA
    GPIO_PORTA_DEN_R = 0x20;  // enable PIN

    GPIO_PORTD_DIR_R = 0x04; //bit 2 for output HiGHSIDE_R
    GPIO_PORTD_DR2R_R = 0x04; // set drive strength to 2mA
    GPIO_PORTD_DEN_R = 0x04;  // enable PIN

    GPIO_PORTE_DIR_R = 0x32; //bits 1,4 and 5 for output INTEGRATE, MEAS_LR and LOWSIDE_R respectively
    GPIO_PORTE_DR2R_R = 0x32; // set drive strength to 2mA
    GPIO_PORTE_DEN_R = 0x32;  // enable PIN

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

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
            strp[count++] = tolower(c);
            GREEN_LED ^= 1;
            RED_LED = 0;
        }
    }
}

// re-initializes the global values to NULL values
void resetCommandArguments(){
    uint8_t i = 0;
    for(i=0; i< 10; i++){
        commandArgs[i] = 0x00;
    }
    argc = 0;
}

// Blocking function that returns with serial data entered by user
void parseStr()
{
    uint8_t count= 0;
    uint8_t cmdLength = strlen(strp);
//    uint8_t cnt = 0;

    while(count < cmdLength){
        if((strp[count] >= 0x30 && strp[count] <= 0x39) || (strp[count] >= 0x41 && strp[count] <= 0x59) || (strp[count] >= 0x61 && strp[count] <= 0x7A)){
            count += 1;
        }
        // ignore if character is "_"
        else if(strp[count] != 95){
            strp[count++] = 0x20;
        }else{
            count += 1;
        }
    }

    // Returns first token
    if(strlen(strp) > 0){
        char *token = strtok(strp, " ");
        commandArgs[argc] = token;
        while (token != 0x00)
        {
                argc += 1;
                token = strtok(0x00, " ");
                commandArgs[argc] = token;
        }
    }
}

//check if number
bool isNumber(char * value){
    uint8_t i = 0;

    for(i = 0; i < strlen(value); i++ ){
     // Iterate through each character and check its number
        if(value[i] >= 0x30 && value[i] <= 0x39){
            continue;
        }else{
            return false;
        }
    }
    return true;
}

//Checks for valid command and return boolean value
bool isCommand(uint8_t argCount){
    uint8_t i = 0;
    char * commands[7] = { "set","reset","voltage","resistor","capacitance","inductance","esr","auto" };
    char * outputs[5] = { "meas_lr","meas_c","highside_r","lowside_r","integrate"};

    for(i=0; i < 7; i++ ){

        if(!(strcmp(commandArgs[0],commands[i]))){

            //1. Check for set command
            if(!(strcmp(commandArgs[0],"set"))){
                if(argCount == 3){
                    //2. second argument lies within the expected output terminals
                    uint8_t j = 0;
                    for(j =0; j < 5; j++){
                        if(!(strcmp(commandArgs[1],outputs[j]))){
                            //3. if the third argument is a valid number
                            if(isNumber(commandArgs[2])){
                                return true;
                            }else{
                                return false;
                            }
                        }
                    }
                }else{
                   return false;
                }
            }
            //check for remaining commands
            else{
                if(argCount > 0){
                   return true;
                }else{
                   return false;
                }
            }
        }
    }
    return false;
}

//returns arguments value based on the position
char * getValue(argCount){
    return commandArgs[argCount];
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void ledCheck(void)
{
    initSerialHw();
    // Toggle red LED every 500m second
    while(1)
    {
      RED_LED ^= 1;
      waitMicrosecond(500000);
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
        GREEN_LED = 0;

        //validate the entered command
        if(isCommand(argc)){
            GREEN_LED = 1;
            RED_LED = 0;
            putsUart0("Entered Command is VALID\r\n");
        }else{
            GREEN_LED = 0;
            RED_LED = 1;
            putsUart0("The Entered Command is NOT VALID\r\n");
        }

        // re-initialize the global values
        resetCommandArguments();
    }
}

/**
 * main.c
 */

int main(void)
{
//    code to blink red led for 500ms
//    ledCheck();
//    code to receive commands
       serialCheck();
}
