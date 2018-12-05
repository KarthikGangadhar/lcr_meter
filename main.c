// LCR meter Code
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
//   Configured to 115,20    0 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <hw_nvic.h>
#include <hw_types.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// variables for getCommand
char  strp[80];

//parseStr variables
uint8_t argc = 0;
char * commandArgs[80];

//timer and frequency related variables
uint32_t time = 0;
uint32_t resistor_time_value = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initSerialHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC ;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0B;  // bits 0,1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0B; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1B;  // enable LEDs and pushbuttons
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

    //drive output pins to zero
    GPIO_PORTA_DATA_R &= ~(0x20);
    GPIO_PORTD_DATA_R &= ~(0x04);
    GPIO_PORTE_DATA_R &= ~(0x32);

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

    // Configure AN10(PB4),AN11(PB5) as an analog input to DUT2,DUT1 respectively
    SYSCTL_RCGCADC_R |= 0x03;                        // turn on ADC module 0 clocking
    GPIO_PORTB_AFSEL_R |= 0x30;                      // select alternative functions for AN11 nad AN10 (PB4,PB5)
    GPIO_PORTB_DEN_R &= ~0x30;                       // turn off digital operation on pin PB4,PB5
    GPIO_PORTB_AMSEL_R |= 0x30;                      // turn on analog operation on pin PB4,PB5

    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 10;                               // set first sample to AN0
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 11;                               // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // setTimerMode
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;  // turn-on Timer

    //configure analog comparator
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;

    // Configure Analog comparator input pins
    GPIO_PORTC_DIR_R &=  ~(0x80);//~(0xC0); //bit 7 and 6 input for analog comparator, PC7,PC6
    GPIO_PORTC_DEN_R &=  ~(0x80);//~(0x80);  // disable PIN
    GPIO_PORTC_AMSEL_R |= 0x80;  // turn on analog operation on pin PC7
    GPIO_PORTC_AFSEL_R |= 0x80;  // select alternative functions

    COMP_ACREFCTL_R |= (COMP_ACREFCTL_EN | COMP_ACREFCTL_VREF_M); // EN = 1, VDDA = 3.3V // 0x40c


    COMP_ACREFCTL_R &= ~(COMP_ACREFCTL_RNG); //RNG = 0x20f
    COMP_ACCTL0_R |= (COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_ISEN_M); // COMP_ACCTL0_ISEN_RISE | COMP_ACCTL0_TSEN_RISE); //0x40c COMP_ACCTL0_CINV

    // interrupt configuration
    COMP_ACRIS_R |= COMP_ACRIS_IN0;
    NVIC_EN0_R |= ~(1 << (INT_COMP0-16));  // turn-on interrupt 41 (COMP0)
    COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
}

// timer function
void startTimer(){
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;  // turn-on Timer
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
{    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// To read Analog Input
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}

// Period timer service publishing latest time measurements every positive edge
void WideTimer5Isr()
{
    char time_count[20];
    float time_value = 0.0;
    time = WTIMER5_TAV_R;                        // read counter input

    time_value = (time / 40.0);
    sprintf(time_count, ": %f", time_value);
    putsUart0("\r\n Time in us ");
    putsUart0(time_count);
    putsUart0("\r\n");

    WTIMER5_TAV_R = 0;                           // zero counter for next edge
    GREEN_LED ^= 1;                              // status
    WTIMER5_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void stopTimer(){
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);  // turn-on interrupt 120 (WTIMER5A)
    WideTimer5Isr();
}

void analogComparator05Isr(){
    //  resistor time constant
    resistor_time_value = WTIMER5_TAV_R;
    // reset the interrupt
    COMP_ACMIS_R |= COMP_ACMIS_IN0;
}

// Blocking function that returns with serial data entered by user
void getCommand()
{
    uint8_t count= 0;
    while(count < 0x50){
        char c = getcUart0();
        if(c == 0x08){ // if character is BACK_SPACE
            if(count > 0){
                count = count - 1;
            }
        }else if(c == 0x0D){ //CARRIAGE_RETURN
            strp[count] = 0x0;
            break;
        }else if( c >= 0x20 ){ // character value greater than SPACE.
            strp[count++] = tolower(c);
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
    char * outputs[5] = { "meas_lr","meas_c","highside_r","lowside_r","integrate"};
    char * commands[18] = { "t", "test","e","a","i","v","c","r","set","reset","voltage","resistor","capacitance","inductance","esr","auto", "timer" };

    for(i=0; i < 18; i++ ){

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
            //2. Check for voltage command
            else if(!(strcmp(commandArgs[0],"voltage")) || !(strcmp(commandArgs[0],"v"))){
                if(argCount == 1){
                    return true;
                }
            }
            //3. Check for resistor command
            else if(!(strcmp(commandArgs[0],"resistor")) || !(strcmp(commandArgs[0],"r"))){
                if(argCount == 1){
                    return true;
                }
            }
            //4. Check for resistor command
            else if(!(strcmp(commandArgs[0],"reset"))){
                if(argCount == 1){
                    return true;
                }
            }
            //5. Check for timer command
            else if(!(strcmp(commandArgs[0],"timer"))){
                if(argCount == 2){
                    return true;
                }
            }
            //6. Check for timer command
            else if(!(strcmp(commandArgs[0],"capacitance")) || !(strcmp(commandArgs[0],"c"))){
                if(argCount == 1){
                    return true;
                }
            }
            //6. Check for timer command
            else if(!(strcmp(commandArgs[0],"inductance")) || !(strcmp(commandArgs[0],"i"))){
                if(argCount == 1){
                    return true;
                }
            }
            else if(!(strcmp(commandArgs[0],"auto")) || !(strcmp(commandArgs[0],"a"))){
                if(argCount == 1){
                    return true;
                }
            }
            else if(!(strcmp(commandArgs[0],"esr")) || !(strcmp(commandArgs[0],"e"))){
                if(argCount == 1){
                    return true;
                }
            }
            else if(!(strcmp(commandArgs[0],"test")) || !(strcmp(commandArgs[0],"t"))){
                if(argCount == 1){
                    return true;
                }
            }
        }
    }
    return false;
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
//    while(1)
//    {
      GREEN_LED ^= 1;
      waitMicrosecond(500000);
      GREEN_LED ^= 1;
//    }
}

void resetLcrMeter(){
    NVIC_APINT_R = (NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ);
    putsUart0("Reset is Done\r\n");
}

void checkTimer(){
    while(1){
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;  // turn-on Timer
        waitMicrosecond(2000000);
        NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);  // turn-on interrupt 120 (WTIMER5A)
        WideTimer5Isr();
    }
}

void resetOutputTerminals(){

    GPIO_PORTE_DATA_R &= ~(0x10);
    GPIO_PORTA_DATA_R &= ~(0x20);
    GPIO_PORTD_DATA_R &= ~(0x04);
    GPIO_PORTE_DATA_R &= ~(0x20);
    GPIO_PORTE_DATA_R &= ~(0x02);
}

// measure voltage
void measureVoltage(){
    uint16_t Dut1;
    uint16_t Dut2;
    char V1[20];
    char V2[20];
    char Vtg[20];

    Dut1 = readAdc0Ss3(); // Dut1
    Dut2 = readAdc1Ss3(); // Dut2

    float v1 = ((Dut1 * 3.3)/4096.0);
    float v2 = ((Dut2 * 3.3)/4096.0);

    float voltage = v2 - v1 ;

    putsUart0("V1 : ");
    sprintf(V1, "%f", v1);
    putsUart0(V1);
    putsUart0(", V2 : ");
    sprintf(V2, "%f", v2);
    putsUart0(V2);
    putsUart0(", Voltage : ");

    sprintf(Vtg, "%f", voltage);
    putsUart0(Vtg);
    putsUart0("\r\n");
}

// Method to measure resistance
void measureResistance(){

        // Reset timer Count
        WTIMER5_TAV_R = 0;

        char resistor_time_count[20];  // character to store time value
        char resistor_characters[20];
        float time_value = 0.0;
        float constant = 1.5308702267422474;
        float resistance;

        // Reset output terminals to 0v
        resetOutputTerminals();

        // wait for sometime
        waitMicrosecond(60000);

        // discharge capacitor
        GPIO_PORTE_DATA_R &= ~(0x10);//  MEAS_LR = 0;
        GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;
        GPIO_PORTE_DATA_R |= 0x02;   // INTEGRATE = 1;

        // wait for 2 sec
        waitMicrosecond(400000);

        // charge capacitor
        GPIO_PORTE_DATA_R &= ~(0x20);   // LOWSIDE_R = 0;
        GPIO_PORTE_DATA_R |= 0x10;      // MEAS_LR = 1;

        WTIMER5_TAV_R = 0; // reset the timer
//        WTIMER5_CTL_R |= TIMER_CTL_TAEN;  // turn-on Timer
        NVIC_EN0_R |= (1 << (INT_COMP0-16)); // Set the comparator Interrupt

        waitMicrosecond(1500000);         // wait for 2 sec
        NVIC_EN0_R |= ~(1 << (INT_COMP0-16)); // Reset the Comparator Interrupt

        // time in micro seconds
        time_value = (resistor_time_value / 40.0);
        sprintf(resistor_time_count, ": %f", time_value);
        putsUart0("\r\n Time in us ");
        putsUart0(resistor_time_count);

        resistance = (time_value / (constant * 1000));

        sprintf(resistor_characters, ": %f", resistance);
        putsUart0(", Resistance in (kilo-ohm) ");
        putsUart0(resistor_characters);
        putsUart0("\r\n");


        // resetting the counter to zero
        WTIMER5_TAV_R = 0;

        // reset the output terminal potentials
        resetOutputTerminals();
}

void displayOutputVoltage(){
    // Blocking function that returns only when SW1 is pressed
    while(GPIO_PORTF_DATA_R & 0x10){
        measureVoltage();
        waitMicrosecond(500000);
    };
}

// Method to measure capacitance
void measureCapacitance(){

        // Reset timer Count
        WTIMER5_TAV_R = 0;

        char capacitor_time_count[20];  // character to store time value
        char capacitor_characters[20];
        float time_value = 0.0;
        float constant = 60.0;
        float capacitance;

        // Reset output terminals to 0v
        resetOutputTerminals();

        // discharge capacitor
        GPIO_PORTA_DATA_R |= 0x20;//  MEAS_C = 1;
        GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

        // wait for 2 sec
        waitMicrosecond(15000000);

        // charge capacitor
        GPIO_PORTE_DATA_R &= ~(0x20);   // LOWSIDE_R = 0;
        GPIO_PORTD_DATA_R |= 0x04;      // HIGHSIDE_R = 1;

        WTIMER5_TAV_R = 0; // reset the timer
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;  // turn-on Timer
        NVIC_EN0_R |= (1 << (INT_COMP0-16)); // Set the comparator Interrupt

        waitMicrosecond(15000000);         // wait for 2 sec
        NVIC_EN0_R |= ~(1 << (INT_COMP0-16)); // Reset the Comparator Interrupt

        // time in micro seconds
        time_value = resistor_time_value;
        sprintf(capacitor_time_count, ": %f", time_value);
        putsUart0("\r\n Time in us ");
        putsUart0(capacitor_time_count);
        putsUart0("\r\n");
        if(time_value < 10000)
            constant = 23.0;

        capacitance = (time_value / (constant * 100000.0));

        sprintf(capacitor_characters, ": %f", capacitance);
        putsUart0("\r\n Capacitance in (u-farad) ");
        putsUart0(capacitor_characters);
        putsUart0("\r\n");

        // resetting the counter to zero
        WTIMER5_TAV_R = 0;

        // reset the output terminal potentials
        resetOutputTerminals();
}

// Method to measure inductance
void measureInductance(){
        char inductance_time_count[20];  // character to store time value
        char inductance_characters[20];
        float time_value = 0.0;
        float constant = 52.14;
        float inductance = 0.0;

        // Reset output terminals to 0v
        resetOutputTerminals();

        // discharge capacitor
        GPIO_PORTA_DATA_R |= 0x20;//  MEAS_C = 1;
        GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

        waitMicrosecond(4000000);

        GPIO_PORTA_DATA_R &= ~(0x20);//  MEAS_C = 0;

        // discharge capacitor
        GPIO_PORTE_DATA_R |= 0x10;//  MEAS_LR = 1;
        GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

        WTIMER5_TAV_R = 0; // reset the timer
        NVIC_EN0_R |= (1 << (INT_COMP0-16)); // Set the comparator Interrupt

        waitMicrosecond(2000000);         // wait for 2 sec
        NVIC_EN0_R |= ~(1 << (INT_COMP0-16)); // Reset the Comparator Interrupt

        // time in micro seconds
        time_value = resistor_time_value;

        //constant different for mill henry inductors
        if(time_value > 1000)
            constant = 23.0;

        sprintf(inductance_time_count, ": %f", time_value);
        putsUart0("\r\n Time in us ");
        putsUart0(inductance_time_count);
        putsUart0("\r\n");

        inductance = ((time_value * 33) / (constant));

        sprintf(inductance_characters, ": %f", inductance);
        putsUart0("\r\n Inductance in (u-henry) ");
        putsUart0(inductance_characters);
        putsUart0("\r\n");

        // resetting the counter to zero
        WTIMER5_TAV_R = 0;

        // reset the output terminal potentials
        resetOutputTerminals();
}

void measureEsr(){

    float Vo = 0; // voltage across the Highside_R
    float Vin = 3.288721; // input voltage
    char esr_value[20];  // character to store time value
    float esr = 0.0;

    // Reset output terminals to 0v
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    // discharge capacitor
    GPIO_PORTA_DATA_R |= 0x20;//  MEAS_C = 1;
    GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

    waitMicrosecond(2000000);

    GPIO_PORTA_DATA_R &= ~(0x20);//  MEAS_C = 0;
    GPIO_PORTE_DATA_R |= 0x10;//  MEAS_LR = 1;
    GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

    waitMicrosecond(4000000);

    uint16_t Dut2 = readAdc1Ss3(); // Dut2

    Vo = ((Dut2 * 3.3)/4096.0);

    char Dut2Vtg[20];

    sprintf(Dut2Vtg, ": %f", Vo);
    putsUart0("\r\n in volts ");
    putsUart0(Dut2Vtg);
    putsUart0("\r\n");
    putsUart0("\r\n");

    esr = (33 * ((Vin - Vo) / Vo));

    sprintf(esr_value, ": %f", esr);
    putsUart0("\r\n in Ohm ");
    putsUart0(esr_value);
    putsUart0("\r\n");
    putsUart0("\r\n");

    // reset the output terminal potentials
    resetOutputTerminals();
}

void checkAuto(){

    putsUart0("\r\n Auto started... \r\n");

    //variable for inductance
    float inductive_time_value = 0.0;
    float resistance_time_value = 0.0;

    char  inductance_time_count[20];  // character to store time value
    char  inductance_characters[20];
    float Lc = 52.14;
    float inductance = 0.0;

    //variable for resistance
    char resistor_time_count[20];  // character to store time value
    char resistor_characters[20];
    float Rc = 1.5308702267422474;
    float resistance;

    //variable for capacitance
    char capacitor_time_count[20];  // character to store time value
    char capacitor_characters[20];
    float Cc = 60.0;
    float capacitance;
    float time_value = 0.0;


    // test for inductance
    putsUart0("\r\n Test for Inductance... \r\n \r\n");
    uint8_t i = 0;
    for(i =0; i <3; i++){

    // Reset output terminals to 0v
    resetOutputTerminals();

    // discharge capacitor
    GPIO_PORTA_DATA_R |= 0x20;//  MEAS_C = 1;
    GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

    waitMicrosecond(4000000);

    GPIO_PORTA_DATA_R &= ~(0x20);//  MEAS_C = 0;

    // discharge capacitor
    GPIO_PORTE_DATA_R |= 0x10;//  MEAS_LR = 1;
    GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

    WTIMER5_TAV_R = 0; // reset the timer
    NVIC_EN0_R |= (1 << (INT_COMP0-16)); // Set the comparator Interrupt

    waitMicrosecond(2000000);         // wait for 2 sec
    NVIC_EN0_R |= ~(1 << (INT_COMP0-16)); // Reset the Comparator Interrupt

    // time in micro seconds
    inductive_time_value = resistor_time_value;

    //constant different for mill henry inductors
    if(inductive_time_value > 1000)
        Lc = 23.0;

    inductance = ((inductive_time_value * 33) / (Lc));

    if(i > 0){
        sprintf(inductance_time_count, ": %f", inductive_time_value);
//        putsUart0("\r\n Time in us ");
//        putsUart0(inductance_time_count);

        sprintf(inductance_characters, ": %f", inductance);
//        putsUart0(", Inductance in (uH) ");
//        putsUart0(inductance_characters);
//        putsUart0("\r\n \r\n");
    }

    // resetting the counter to zero
    WTIMER5_TAV_R = 0;

    // reset the output terminal potentials
    resetOutputTerminals();

    waitMicrosecond(4000000);
}

    // test for resistor
    putsUart0("\r\n Test for Resistance... \r\n \r\n");
     // Reset output terminals to 0v
     resetOutputTerminals();

     // wait for sometime
     waitMicrosecond(60000);

     // discharge capacitor
     GPIO_PORTE_DATA_R &= ~(0x10);//  MEAS_LR = 0;
     GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;
     GPIO_PORTE_DATA_R |= 0x02;   // INTEGRATE = 1;

     // wait for 2 sec
     waitMicrosecond(400000);

     // charge capacitor
     GPIO_PORTE_DATA_R &= ~(0x20);   // LOWSIDE_R = 0;
     GPIO_PORTE_DATA_R |= 0x10;      // MEAS_LR = 1;

     WTIMER5_TAV_R = 0; // reset the timer
//        WTIMER5_CTL_R |= TIMER_CTL_TAEN;  // turn-on Timer
     NVIC_EN0_R |= (1 << (INT_COMP0-16)); // Set the comparator Interrupt

     waitMicrosecond(500000);         // wait for 2 sec
     NVIC_EN0_R |= ~(1 << (INT_COMP0-16)); // Reset the Comparator Interrupt

     // time in micro seconds
     resistance_time_value = (resistor_time_value / 40.0);
     sprintf(resistor_time_count, ": %f", resistance_time_value);
//     putsUart0("\r\n Time in us ");
//     putsUart0(resistor_time_count);

     resistance = (resistance_time_value / (Rc * 1000));

     sprintf(resistor_characters, ": %f", resistance);
//     putsUart0(", Resistance in (kilo-ohm) ");
//     putsUart0(resistor_characters);
//     putsUart0("\r\n \r\n");

     // reset the output terminal potentials
     resetOutputTerminals();

    // test for capacitance
     putsUart0("\r\n Test for Capacitance... \r\n \r\n");

    // Reset output terminals to 0v
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(100000);

    // discharge capacitor
    GPIO_PORTA_DATA_R |= 0x20;//  MEAS_C = 1;
    GPIO_PORTE_DATA_R |= 0x20;   //  LOWSIDE_R = 1;

    // wait for 2 sec
    waitMicrosecond(15000000);

    // charge capacitor
    GPIO_PORTE_DATA_R &= ~(0x20);   // LOWSIDE_R = 0;
    GPIO_PORTD_DATA_R |= 0x04;      // HIGHSIDE_R = 1;

    WTIMER5_TAV_R = 0; // reset the timer
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;  // turn-on Timer
    NVIC_EN0_R |= (1 << (INT_COMP0-16)); // Set the comparator Interrupt

    waitMicrosecond(15000000);         // wait for 2 sec
    NVIC_EN0_R |= ~(1 << (INT_COMP0-16)); // Reset the Comparator Interrupt

    // time in micro seconds
    time_value = resistor_time_value;
    sprintf(capacitor_time_count, ": %f", time_value);
//    putsUart0("Time in us ");
//    putsUart0(capacitor_time_count);

    if(time_value < 10000)
        Cc = 23.0;

    capacitance = (time_value / (Cc * 100000.0));

    sprintf(capacitor_characters, ": %f", capacitance);
//    putsUart0(", Capacitance in (u-farad) ");
//    putsUart0(capacitor_characters);
//    putsUart0("\r\n \r\n");

    // resetting the counter to zero
    WTIMER5_TAV_R = 0;

    // reset the output terminal potentials
    resetOutputTerminals();

    if(( capacitance > 4.00 && capacitance < 5.00) && (inductance > 150000.0) && (resistance >= 2)){
        putsUart0("\r\n Circuit is Resistive   -->");
        putsUart0(" Resistance in (kilo-ohm) ");
        putsUart0(resistor_characters);
        putsUart0("\r\n \r\n");

    }
    else if(((capacitance > 5.00 && capacitance < 9.5) && (resistance < 2)) || ((capacitance < 9.5) && (resistance < 2))){
        putsUart0("\r\n Circuit is Inductive   -->");
        putsUart0(", Inductance in (u-Henry) ");
        putsUart0(inductance_characters);
        putsUart0("\r\n \r\n");

    }
    else if(resistance < 2.0 && capacitance > 9.0){
        putsUart0("\r\n Circuit is Capacitive  -->");
        putsUart0(" Capacitance in (u-farad) ");
        putsUart0(capacitor_characters);
        putsUart0("\r\n \r\n");
    }
    else if((inductance > 150000.0) && (inductance > 250000.0)){
        putsUart0("\r\n Circuit is Inductive   -->");
        putsUart0(", Inductance in (u-Henry) ");
        putsUart0(inductance_characters);
        putsUart0("\r\n \r\n");
    }
    else if(resistance > 5){
        putsUart0("\r\n Circuit is Resistive   -->");
        putsUart0(" Resistance in (kilo-ohm) ");
        putsUart0(resistor_characters);
        putsUart0("\r\n \r\n");
    }
    else if((capacitance > 9.0)){
        putsUart0("\r\n Circuit is Capacitive  -->");
        putsUart0(" Capacitance in (u-farad) ");
        putsUart0(capacitor_characters);
        putsUart0("\r\n \r\n");
    }


}

void checkCircuit() {

    // reset the output terminal potentials
    resetOutputTerminals();

    putsUart0("\r\n MEAS_LR = 1     -->");
    // set meas_lr
    GPIO_PORTE_DATA_R |= 0x10;
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n MEAS_LR = 0     -->");
    // reset meas_lr
    GPIO_PORTE_DATA_R &= ~(0x10);
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n MEAS_C = 1      -->");
    // set meas_c
    GPIO_PORTA_DATA_R |= 0x20;
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n MEAS_C = 0      -->");
    // reset meas_c
    GPIO_PORTA_DATA_R &= ~(0x20);
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n HIGHSIDE_R = 1  -->");
    // set highside_r
    GPIO_PORTD_DATA_R |= 0x04;
    waitMicrosecond(200000);
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n HIGHSIDE_R = 0  -->");
    // reset highside_r
    GPIO_PORTD_DATA_R &= ~(0x04);
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n LOWSIDE_R = 1   -->");
    // set lowside_r
    GPIO_PORTE_DATA_R |= 0x20;
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n LOWSIDE_R = 0   -->");
    // reset lowside_r
    GPIO_PORTE_DATA_R &= ~(0x20);
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n INTEGRATE = 1   -->");
    // set integrate
    GPIO_PORTE_DATA_R |= 0x02;
    measureVoltage();
    resetOutputTerminals();

    // wait for sometime
    waitMicrosecond(500000);

    putsUart0("\r\n INTEGRATE = 0   -->");
    // reset integrate
    GPIO_PORTE_DATA_R &= ~(0x02);
    measureVoltage();

    // reset the output terminal potentials
    resetOutputTerminals();
}

bool ExecuteCommand(){
    // if command is set and argument count is 3
    if(!(strcmp(commandArgs[0],"set")) && argc == 3){
             //2. second argument lies within the expected output terminals
                 if(!(strcmp(commandArgs[1],"meas_lr"))){
                     //3. if the third argument is a valid number
                     if(!strcmp(commandArgs[2],"0")){
                         GPIO_PORTE_DATA_R &= ~(0x10);
                     }else if(!strcmp(commandArgs[2],"1")){
                         GPIO_PORTE_DATA_R |= 0x10;
                     }
                 }else if(!(strcmp(commandArgs[1],"meas_c"))){
                     if(!strcmp(commandArgs[2],"0")){
                         GPIO_PORTA_DATA_R &= ~(0x20);
                     }else if(!strcmp(commandArgs[2],"1")){
                         GPIO_PORTA_DATA_R |= 0x20;
                     }
                 }else if(!(strcmp(commandArgs[1],"highside_r"))){
                     if(!strcmp(commandArgs[2],"0")){
                         GPIO_PORTD_DATA_R &= ~(0x04);
                     }else if(!strcmp(commandArgs[2],"1")){
                         GPIO_PORTD_DATA_R |= 0x04;
                     }
                 }else if(!(strcmp(commandArgs[1],"lowside_r"))){
                     if(!strcmp(commandArgs[2],"0")){
                         GPIO_PORTE_DATA_R &= ~(0x20);
                     }else if(!strcmp(commandArgs[2],"1")){
                         GPIO_PORTE_DATA_R |= 0x20;
                     }
                 }else if(!(strcmp(commandArgs[1],"integrate"))){
                     if(!strcmp(commandArgs[2],"0")){
                         GPIO_PORTE_DATA_R &= ~(0x02);
                     }else if(!strcmp(commandArgs[2],"1")){
                         GPIO_PORTE_DATA_R |= 0x02;
                     }
                }
         }
    else if((!(strcmp(commandArgs[0],"voltage")) || !(strcmp(commandArgs[0],"v"))) && argc == 1){
        measureVoltage();
        return true;
    }
    else if(!(strcmp(commandArgs[0],"reset")) && argc == 1){
        resetLcrMeter();
        return true;
    }
    else if((!(strcmp(commandArgs[0],"resistor")) || !(strcmp(commandArgs[0],"r"))) && argc == 1){
        measureResistance();
        return true;
    }
    else if((!(strcmp(commandArgs[0],"capacitance")) || !(strcmp(commandArgs[0],"c"))) && argc == 1){
        measureCapacitance();
        return true;
    }
    else if((!(strcmp(commandArgs[0],"inductance")) || !(strcmp(commandArgs[0],"i"))) && argc == 1){
        measureInductance();
        return true;
    }
    else if((!(strcmp(commandArgs[0],"esr")) || !(strcmp(commandArgs[0],"e"))) && argc == 1){
            measureEsr();
            return true;
     }
    else if((!(strcmp(commandArgs[0],"auto")) || !(strcmp(commandArgs[0],"a"))) && argc == 1){
            checkAuto();
            return true;
     }
    else if(!(strcmp(commandArgs[0],"timer")) && argc == 2 && !(strcmp(commandArgs[1],"start"))){
        checkTimer();
        return true;
    }
    else if(!(strcmp(commandArgs[0],"timer")) && argc == 2 && !(strcmp(commandArgs[1],"stop"))){
        return true;
    }
    else if((!(strcmp(commandArgs[0],"test")) || !(strcmp(commandArgs[0],"t"))) && argc == 1){
            checkCircuit();
            return true;
    }
    else{
            return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
void serialCheck(void)
{
    // Initialize hardware
    initSerialHw();

    putsUart0("\r\nEnter Commands\r\n \r\n");

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
            if(ExecuteCommand()){}
                putsUart0("\r\n \r\n");
        }else{
            GREEN_LED = 0;
            putsUart0("False \r\n \r\n");
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
    ledCheck();
//    code to receive commands
    serialCheck();
}
