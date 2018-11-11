//// Serial Code
//// Karthik Gangadhar
//
////-----------------------------------------------------------------------------
//// Hardware Target
////-----------------------------------------------------------------------------
//
//// Target Platform: EK-TM4C123GXL Evaluation Board
//// Target uC:       TM4C123GH6PM
//// System Clock:    40 MHz
//
//// Hardware configuration:
//// Red Backlight LED:
////   PB5 drives an NPN transistor that powers the red LED
//// Green Backlight LED:
////   PE4 drives an NPN transistor that powers the green LED
//// Blue Backlight LED:
////   PE5 drives an NPN transistor that powers the blue LED
//// Red LED:
////   PF1 drives an NPN transistor that powers the red LED
//// Green LED:
////   PF3 drives an NPN transistor that powers the green LED
//// Pushbutton:
////   SW1 pulls pin PF4 low (internal pull-up is used)
//// UART Interface:
////   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
////   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
////   Configured to 115,200 baud, 8N1
//
////-----------------------------------------------------------------------------
//// Device includes, defines, and assembler directives
////-----------------------------------------------------------------------------
