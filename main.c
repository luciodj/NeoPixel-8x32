/**
 NeoPixel 8x32 display demo

 Author: Lucio Di Jasio

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v3.00 Beta
        Device            :  PIC16F18855
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.35
        MPLAB             :  MPLAB X v3.10
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#include "NeoPixel.h"
#include <ctype.h>
#include <string.h>

#define STR_SIZE    14
char s[STR_SIZE];

uint32_t bg = NEO_BLACK;        // default background color
uint32_t fg = NEO_GREEN;        // default foreground color
bool scrolling = true;

uint32_t xtoc(char *s)
{ // convert a hex string to a 24-bit RGB value
    char c;
    uint32_t color = 0;
    while( c = *s++) {
        if (isxdigit(c)) {
            c -= '0'; c = (c > 9) ? (c-7) : c;
            color = (color << 4) + c; 
        }
    }
    return color & 0xffffff;    
}

void ProcessInput( char c)
{
    static bool cmd = false;
    uint8_t len = strlen(s);
    if (cmd) {
        switch(toupper(c)) {
        case '\r': 
                return;
        case 'D': // display string
            NeoPixel_Clear(bg);
            NeoPixel_Puts(0, s, fg);
            NeoPixel_Show();
            scrolling = false;
            puts("->Display");
            break;
        case 'S': // scroll string
            NeoPixel_Scroll(s, fg, true);
            scrolling = true;
            puts("->Scroll");
            cmd = false;
            putch('"');
            return;
        case 'B': // background color
            if (len >= 6)
                bg = xtoc(s);
            puts("->Background" );
            break;
        case 'F': // foreground color
            if (len >= 6)
                fg = xtoc(s);
            puts("->Foreground");
            break;
        case 'C': // clear screen
            NeoPixel_Clear(bg);
            NeoPixel_Show();
            scrolling = false;
            puts("->Clear");
            break;
        case '\n': // start a new string
            break;
        default:   // command not recognized
            puts("->valid commands are: B, C, D, F, S");
            putch(':');
            return; 
        }
        s[0] = '\0';
        cmd = false;
        putch('"');
    }
    else switch(c) {
        case '\r': 
            break;
        case '\n':
            cmd = true; putch(':');
            break;
        default:    // buffering
            if ((c < ' ') || (c > '~'))
                return;
            if (len < STR_SIZE-1)          
                s[len++] = c; s[len] = '\0';
            break;
    }
}

/*
                         Main application
 */
void main(void)
{
    uint8_t delay = 0;
    s[0] = '\0';
    scrolling = true;
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    puts("\nXPRESS NeoPixel Demo");
    NeoPixel_Clear(NEO_BLACK);  
    strcpy(s, "** Xpress ");
    NeoPixel_Scroll(s, NEO_GREEN, true);
    
    // start in string mode
    putch('"');
    while (1)
    {
        if (EUSART_DataReady)
            ProcessInput(getche());
        
        if ( TMR2_HasOverflowOccured())     // 10ms
        {   
            if ( delay > 0) delay--;
            else {
                if (scrolling)  NeoPixel_Scroll(s, fg, false);
                delay = 2;
            }
        }
    }
}
/**
 End of File
*/