#include <stdbool.h>
#include "expander.h"
#include "lcd.h"
#include "digipot.h"
#include "mcc_generated_files/timer/delay.h"

#define LCDMini_nCS_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define LCDMini_nCS2_SetHigh()           do { LATDbits.LATD0 = 1; } while(0)
#define LCDMini_nReset_SetHigh()         do { LATDbits.LATD7 = 1; } while(0)

void lcd_sendNibble(uint8_t nibble, bool RSbit){
    uint8_t packet = (nibble << 4) | (RSbit << 2);
    expander_setOutput(packet);
    expander_setOutput(packet | (1<<3));
    DELAY_milliseconds(1);
    expander_setOutput(packet);
    DELAY_milliseconds(40);
}

void lcd_sendByte(uint8_t byte, bool RSbit){
    uint8_t nibbleHigh = byte >> 4;
    uint8_t nibbleLow = byte & 0xF;
    uint8_t packetHigh = (nibbleHigh << 4) | (RSbit << 2);
    uint8_t packetLow = (nibbleLow << 4) | (RSbit << 2);
    
    expander_setOutput(packetHigh);
    DELAY_milliseconds(2);
    expander_setOutput(packetHigh | (1<<3));
    DELAY_milliseconds(2);
    expander_setOutput(packetLow);
    DELAY_milliseconds(2);
    expander_setOutput(packetLow | (1<<3));
    DELAY_milliseconds(40);
}

void lcd_returnHome(void){
    lcd_sendByte(0b10, false);
    DELAY_milliseconds(2);
}

void lcd_clearDisplay(void){
    lcd_sendByte(0b1, false);
    DELAY_milliseconds(2);
}

void lcd_setAddr(uint8_t row, uint8_t character){
    lcd_sendByte(0x80 | (character + (row*40)), false);
}

void lcd_writeChar(uint8_t character){
    lcd_sendByte(character, true);
}

void lcd_writeString(uint8_t* string, uint8_t row) {
    lcd_setAddr(row, 0);
    uint8_t i = 0;
    for (i = 0; i < 16; i++) {
        if (string[i]) {
            lcd_writeChar(string[i]);
        }
    }
    lcd_returnHome();
}

void lcd_setContrast(uint8_t contrast){
    digipot_setWiper(contrast);
}

void lcd_setup(){
    LCDMini_nCS_SetHigh(); /* set LCDMini_nCS output high */
    LCDMini_nCS2_SetHigh(); /* set LCDMini_nCS2 output high */
    LCDMini_nReset_SetHigh(); /* set LCDMini_nReset output high */
    expander_setup();
    expander_setOutput(0);
    DELAY_milliseconds(40);
    lcd_sendNibble(0b11, false);
    DELAY_milliseconds(10);

    lcd_sendNibble(0b11, false);
    DELAY_milliseconds(10);
    lcd_sendNibble(0b11, false);
    DELAY_milliseconds(10);
    lcd_sendNibble(0x2,false);
    lcd_sendByte(0x2C, false);
    lcd_sendByte(0b1100, false);
    lcd_sendByte(0x06, false);
    lcd_sendByte(0x0C, false);
    DELAY_milliseconds(2);
    lcd_returnHome();
    lcd_clearDisplay();
}