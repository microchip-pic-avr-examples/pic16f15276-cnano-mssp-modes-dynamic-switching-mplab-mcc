#include <stdint.h>
#include <xc.h>
#include "user_mssp1.h"
#include "user_spi_interface.h"

#define IODIRB      0x01
#define OLATB       0x15
#define WRITE_BYTE  0b01000000

#define LCDMini_nCS_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define LCDMini_nCS_SetLow()             do { LATCbits.LATC7 = 0; } while(0)

void expander_sendByte(uint8_t addr, uint8_t byte){
    SPI1_Open(MSSP1_DEFAULT);
    LCDMini_nCS_SetLow(); /* set LCDMini_nCS output low */
    uint8_t cmd[3];
    cmd[0] = WRITE_BYTE;
    cmd[1] = addr;
    cmd[2] = byte;
    SPI1_WriteBlock(cmd, 3);
    LCDMini_nCS_SetHigh(); /* set LCDMini_nCS output high */
    SPI1_Close();
}

void expander_setup(void){
    expander_sendByte(IODIRB, 0);
}

void expander_setOutput(uint8_t output){
    expander_sendByte(OLATB, output);
}