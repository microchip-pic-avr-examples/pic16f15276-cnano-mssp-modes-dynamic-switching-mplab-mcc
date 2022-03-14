#include "digipot.h"
#include <stdint.h>
#include <xc.h>
#include "user_mssp1.h"
#include "user_spi_interface.h"

#define LCDMini_nCS2_SetLow()             do { LATDbits.LATD0 = 0; } while(0)
#define LCDMini_nCS2_SetHigh()            do { LATDbits.LATD0 = 1; } while(0)

void digipot_setWiper(uint8_t val){
    SPI1_Open(MSSP1_DEFAULT);
    LCDMini_nCS2_SetLow(); /* set LCDMini_nCS2 output low */
    uint8_t cmd[2];
    cmd[0] = 0;
    cmd[1] = val;
    SPI1_WriteBlock(cmd, 2);
    LCDMini_nCS2_SetHigh(); /* set LCDMini_nCS2 output high */
    SPI1_Close();
}