/*
 * File:   Digital_Thermometer.c
 * Author: A. Inácio Morais
 * 
 * PIC16F886 - MPLAB X IDE v5.40 - XC8-CC v2.30
 * MCLRE OFF - INTRC_NOCLKOUT 8 MHz
 * 
 * 16x2 LCD Display (CGRAM Custom Character Handling), Pushbutton (Input with the Internal Pull-Up Resistor Enabled), LM50 Temperature Sensor,
 * and Oversampled ADC Readings
 * 
 * Celsius / Fahrenheit Measures (Mode Selected by a Pushbutton)
 *
 * LCD Display interface based on the PICDEM 2 Plus Demo Board LCD library provided by Microchip:
 * https://www.microchip.com/en-us/development-tool/DM163022-1
 * https://ww1.microchip.com/downloads/en/DeviceDoc/PICDEM2pluscode.zip
 * 
 * In a real-world application consider NOT applying a bypass capacitor (filter) with the LM50 component in order to ensure the noisy state required by the oversampling technique
 * And, implement a calibration variable if necessary
 * 
 * anderson.morais@protonmail.com - (35)991619878
 * 
 * Created on 28 de Setembro de 2020, 15:00
 */

// PIC16F886 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 8000000

// set up the timing for the LCD delays
#define LCD_delay           5     // ~5mS
#define LCD_Startup         15    // ~15mS

// Command set for the LCD display controller (HITACHI HD44780U or WINSTAR WH1602C)
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_CURSOR_BACK     0x10
#define LCD_CURSOR_FWD      0x14
#define LCD_PAN_LEFT        0x18
#define LCD_PAN_RIGHT       0x1C
#define LCD_CURSOR_OFF      0x0C
#define LCD_CURSOR_ON       0x0E
#define LCD_CURSOR_BLINK    0x0F
#define LCD_CURSOR_LINE2    0xC0
#define LCD_CGRAM           0x40 // Initial CGRAM address

// display controller setup commands from page 46 of Hitachi datasheet or page 17 of Winstar comprehensive datasheet (Ex.: WH1602B-TMI-ET#)
#define FUNCTION_SET        0x28                         // 4 bit interface, 2 lines, 5x8 font
#define ENTRY_MODE          0x06                         // increment mode
#define DISPLAY_SETUP       0x0C                         // display on, cursor off, blink off

// single bit for selecting command register or data register
#define instr        0
#define data         1

// These #defines create the pin connections to the LCD Display
#define LCD_PORT     PORTC
#define LCD_EN       PORTCbits.RC5                      // LCD enable
#define LCD_RS       PORTCbits.RC4                      // LCD register select line

#define NB_LINES    2                                   // Number of display lines
#define NB_COL      16                                  // Number of characters per line

// Function to write a specified nibble to the LCD
void LCDWriteNibble(unsigned char ch,unsigned char rs)
{
    // always send the upper nibble
    ch = (ch >> 4);

    // mask off the nibble to be transmitted
    ch = (ch & 0x0F);

    // clear the lower half of LCD_PORT
    LCD_PORT = (LCD_PORT & 0xF0);

    // move the nibble onto LCD_PORT
    LCD_PORT = (LCD_PORT | ch);

    //set data/instr bit to 0 = instructions; 1 = data
    LCD_RS = rs;

    // RW - set write mode
    //LCD_RW = 0;

    // set up enable before writing nibble
    LCD_EN = 1;

    // turn off enable after write of nibble
    LCD_EN = 0;
}

// Send an ASCII command to the LCD in instruction mode
void LCDPutCmd(unsigned char ch) {
    __delay_ms(LCD_delay);

    //Send the higher nibble
    LCDWriteNibble(ch,instr);

    //get the lower nibble
    ch = (ch << 4);

    __delay_ms(1);

    //Now send the lower nibble
    LCDWriteNibble(ch,instr);
}

// This routine initializes the LCD driver
// This routine must be called before any other LCD routine is called 
void LCD_Initialize() {
    // clear latches before enabling TRIS bits
    LCD_PORT = 0;

    TRISC = 0x00;

    // power up the LCD
    //LCD_PWR = 1;

    // required by display controller to allow power to stabilize
    __delay_ms(LCD_Startup);

    // required by display initialization
    LCDPutCmd(0x32);

    // set interface size, # of lines and font
    LCDPutCmd(FUNCTION_SET);

    // turn on display and sets up cursor
    LCDPutCmd(DISPLAY_SETUP);
    
    LCDPutCmd(LCD_CLEAR);

    // set cursor movement direction
    LCDPutCmd(ENTRY_MODE);
}

// Writes character to LCD at current cursor position
void LCDPutChar(unsigned char ch) {
    __delay_ms(LCD_delay);

    //Send higher nibble first
    LCDWriteNibble(ch,data);

    //get the lower nibble
    ch = (ch << 4);

    // Now send the low nibble
    LCDWriteNibble(ch,data);
}

// This routine writes string to LCD at current cursor position
void LCDPutStr(const char *str) {
    unsigned char i = 0;
    
    // While string has not been fully traversed
    while (str[i])
    {
        // Go display current char
        LCDPutChar(str[i++]);
    }    
}                      

// This function positions the cursor at the specified Line and column
void LCDGoto(unsigned char pos,unsigned char ln) {
    // if incorrect line or column
    if ((ln > (NB_LINES-1)) || (pos > (NB_COL-1)))
    {
        // Just do nothing
        return;
    }

    // LCD_Goto command
    LCDPutCmd((ln == 1) ? (0xC0 | pos) : (0x80 | pos));

    // Wait for the LCD to finish
    __delay_ms(LCD_delay);
}

void config() { // General and ADC Config
    OSCCONbits.IRCF = 0b111;    // IRCF<2:0> = 111 = 8 MHz
    
    TRISBbits.TRISB7 = 1;  // 1 = PORTB pin configured as an input (tri-stated)   
    OPTION_REGbits.nRBPU = 0;   // Global RBPU bit of the OPTION register must be cleared for individual pull-ups to be enabled.
    WPUBbits.WPUB7 = 1; //1 = Pull-up enabled
    
    //See A/D CONVERSION PROCEDURE Page 105 (PIC16F886 datasheet - Order according to that page)   
    TRISAbits.TRISA0 = 1; // 1 = PORTA pin configured as an input (tri-stated)
    ANSEL = 0x01;  // 0 = Digital I/O. Pin is assigned to port or special function 1 = Analog input. Pin is assigned as analog input [AN0]
    ANSELH = 0x00; // 0 = Digital I/O. Pin is assigned to port or special function
    ADCON0bits.ADCS = 0b10; // ADCS<1:0>: A/D Conversion Clock Select bits 10 = FOSC/32
    ADCON1bits.VCFG0 = 0b00; // VCFG1: Voltage Reference bit 0 = VSS VCFG0: Voltage Reference bit 0 = V DD
    ADCON0bits.CHS = 0b0000; // CHS<3:0>: Analog Channel Select bits 0000 = AN0
    ADCON1bits.ADFM = 1; // ADFM: A/D Conversion Result Format Select bit 1 = Right justified
    ADCON0bits.ADON = 1; // ADON: ADC Enable bit 1 = ADC is enabled                    
}

unsigned int ADC_read() {
    __delay_us(7); // See 9.2.6 and 9.3 - Delay in case of analog input switching or multiple readings in a short period of time
    ADCON0bits.GO_nDONE = 1; // GO/nDONE: A/D Conversion Status bit
    /* 1 = A/D conversion cycle in progress. Setting this bit starts an A/D conversion cycle.
     * This bit is automatically cleared by hardware when the A/D conversion has completed. */
    
    while (ADCON0bits.GO_nDONE == 1); // Wait 0 (A/D conversion completed/not in progress)
    return ((ADRESH << 8) + ADRESL);    
}

void main(void) {
    char mode = 0;
    
    config();
    LCD_Initialize();
    
    LCDPutCmd(LCD_CGRAM); // Entering initial CGRAM address
    LCDPutChar(0x0C);   // It includes the first CGRAM(1) line      XXX001100 
    LCDPutChar(0x12);   // It includes the second CGRAM(1) line     XXX010010 
    LCDPutChar(0x12);   // It includes the third CGRAM(1) line      XXX010010
    LCDPutChar(0x0C);   // It includes the fourth CGRAM(1) line     XXX001100
    LCDPutChar(0x00);   // It includes the fifth CGRAM(1) line      XXX000000
    LCDPutChar(0x00);   // It includes the sixth CGRAM(1) line      XXX000000
    LCDPutChar(0x00);   // It includes the seventh CGRAM(1) line    XXX000000
    LCDPutChar(0x00);   // It includes the seventh CGRAM(1) line    XXX000000 - Always blank (cursor location)
    // Continue the sequence of 8 lines per custom character above to include characters in other CGRAM positions
    
    // How to call CGRAM characters: 
    // LCDPutChar(0x00); // Calling the first CGRAM character [CGRAM(1)]
    // LCDPutChar(0x01); // Calling the second CGRAM character [CGRAM(2)]
    // [...]
    // LCDPutChar(0x07); // Calling the eighth CGRAM character [CGRAM(8)]
    
    // How to call some special built-in characters:
    
    //HITACHI HD44780U ROM A02 Pattern (page 18)
        //LCDPutChar(0x9A); // Uppercase Omega
        //LCDPutChar(0xAB); // <<
        //LCDPutChar(0xBB); // >>
        //LCDPutChar(0xB0); // Degree Symbol
        //LCDPutChar(0xC0); // À
        //LCDPutChar(0xE0); // à
        //LCDPutChar(0xC1); // Á
        //LCDPutChar(0xE1); // á
        //LCDPutChar(0xC2); // Â
        //LCDPutChar(0xE2); // â
        //LCDPutChar(0xC3); // Ã
        //LCDPutChar(0xE3); // ã
        //LCDPutChar(0xC9); // É
        //LCDPutChar(0xE9); // é
        //LCDPutChar(0xCA); // Ê
        //LCDPutChar(0xEA); // ê
        //LCDPutChar(0xCD); // Í
        //LCDPutChar(0xED); // í
        //LCDPutChar(0xD3); // Ó
        //LCDPutChar(0xF3); // ó
        //LCDPutChar(0xD4); // Ô
        //LCDPutChar(0xF4); // ô
        //LCDPutChar(0xD5); // Õ
        //LCDPutChar(0xF5); // õ
        //LCDPutChar(0xF6); // ö
        //LCDPutChar(0xC7); // Ç
        //LCDPutChar(0xE7); // ç
        //LCDPutChar(0xDA); // Ú
        //LCDPutChar(0xFA); // ú
        //LCDPutChar(0xFC); // ü
        
    //WINSTAR WH1602C English and European standard font (ET) - See Extended ASCII table
        
    while(1) {        
        unsigned int vin0;
        unsigned long vin1 = 0;
        float vin2;
        unsigned int temp0;
        unsigned char ch;
        
        
        for (unsigned char i = 0; i < 255; i++) { // Oversampling (10-bit to 14-bit) - See 'AVR121: Enhancing ADC resolution by oversampling'
            vin0 = ADC_read();                       
            vin1 += vin0;
        }
        
        vin1 /= 16; // Decimation - See 'AVR121: Enhancing ADC resolution by oversampling'
        
        vin2 = (float)vin1 * 5.0 / 16383.0; // Normalization (14-bit --- 5V)
        
        if (PORTBbits.RB7 == 0) {
            __delay_ms(10);
            if (PORTBbits.RB7 == 0) mode ^= (1 << 0);
            while (PORTBbits.RB7 == 0);
        }
                       
        LCDGoto(0,0);
        LCDPutStr("Temperature:    ");
        
        if ((vin2 < 0.5 && mode == 0) || (vin2 < 0.322222 && mode == 1)) { // 0.5 V = 0 °C - According to LM50 datasheet: VO = (+10 mV/°C × T °C) + 500 mV   
            if (mode == 0 ) temp0 = (unsigned int)((((vin2 - 0.500) / 0.01) - 0.5 ) * -100.0); // -0.5 - Float rounding before integer conversion
            else temp0 = (unsigned int)(((vin2 * 180.0 - 58.0) - 0.5 ) * -100.0);
            
            if (temp0 < 1000) {
                LCDGoto(0,1);
                LCDPutStr("     -");
                ch = temp0 / 100; // Extract first digit (integer part)
                LCDPutChar(48+ch); // 48+ - Corresponding the integer value to the ASCII table
                
                LCDPutStr(".");
                
                ch = temp0 % 10; // Extract second digit (tenths)
                LCDPutChar(48+ch);
                
                LCDPutChar(0x00);
                // Proteus seems to simulate an LCD Display HITACHI HD44780U with ROM A00 (Japanese Characters Pattern) hence you can use 'LCDPutChar(223);'
                // to display a degree symbol while simulating it. 
                                
                if (mode == 0 ) LCDPutStr("C     ");
                else LCDPutStr("F     ");
            }
            else {
                LCDGoto(0,1);
                LCDPutStr("    -");
                ch = temp0 / 1000; // Extract first digit (integer part - tens)
                LCDPutChar(48+ch);
                                         
                ch = (temp0 / 100) % 10; // Extract second digit (integer part - ones)
                LCDPutChar(48+ch);
                
                LCDPutStr(".");
                
                ch = (temp0 / 10) % 10; // Extract third digit (tenths)
                LCDPutChar(48+ch);
                
                LCDPutChar(0x00);
                                
                if (mode == 0 ) LCDPutStr("C     ");
                else LCDPutStr("F     ");
            }
        }
        else {
            if (mode == 0 ) temp0 = (unsigned int)((((vin2 - 0.500) / 0.01) + 0.5 ) * 100.0); // +0.5 - Float rounding before integer conversion
            else temp0 = (unsigned int)(((vin2 * 180.0 - 58.0) + 0.5 ) * 100.0);
            
            if (temp0 < 1000) {
                LCDGoto(0,1);
                LCDPutStr("      ");
                ch = temp0 / 100; 
                LCDPutChar(48+ch);
                
                LCDPutStr(".");
                
                ch = temp0 % 10;
                LCDPutChar(48+ch);
                
                LCDPutChar(0x00);
                                
                if (mode == 0 ) LCDPutStr("C     ");
                else LCDPutStr("F     ");                
            }
            else {
                LCDGoto(0,1);
                LCDPutStr("     ");
                ch = temp0 / 1000; 
                LCDPutChar(48+ch); 
                
                ch = (temp0 / 100) % 10; 
                LCDPutChar(48+ch);
                
                LCDPutStr(".");
                
                ch = (temp0 / 10) % 10; 
                LCDPutChar(48+ch);
                
                LCDPutChar(0x00);
                                
                if (mode == 0 ) LCDPutStr("C     ");
                else LCDPutStr("F     ");
            }
        }       
        __delay_ms(200);
    }
}
