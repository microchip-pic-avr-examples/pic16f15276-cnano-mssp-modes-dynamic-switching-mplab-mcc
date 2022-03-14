/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef APPLICATION_H
#define	APPLICATION_H

#include <xc.h>                                                              // Include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mcc_generated_files/system/system.h"
#include "digipot.h"
#include "expander.h"
#include "lcd.h"
#include "user_mssp1.h"
#include "user_spi_interface.h"
#include "mcc_generated_files/timer/delay.h"
#include "mcc_generated_files/i2c_host/mssp1.h"

// Internal Address
#define MAX30100_ADDR                   (0x57)                               // Heartrate Sensor(Client) Address

/* 
 *      MAX30100 IC (Sensor) Registers
 */
#define MAX30100_INTERRUPT_STAT_REG     (0x00)                               // Interrupt Status Register (Read Only)                                                                             
#define MAX30100_INTERRUPT_EN_REG       (0x01)                               // Interrupt Enable Register
#define MAX30100_FIFO_WR_PTR_REG        (0x02)                               // FIFO Write Pointer
#define MAX30100_OVF_CTR_REG            (0x03)                               // FIFO Overflow Counter Pointer
#define MAX30100_FIFO_RD_PTR_REG        (0x04)                               // FIFO Read Pointer
#define MAX30100_FIFO_DATA_REG          (0x05)                               // FIFO Data Register
#define MAX30100_MODE_CONFIG_REG        (0x06)                               // Mode Configuration Register                                                                             
#define MAX30100_SPO2_CONFIG_REG        (0x07)                               // SpO2 Configuration Register                                                                             
#define MAX30100_LED_CONFIG_REG         (0x09)                               // LED Configuration Register                                                                             

/* 
 *      MAX30100 USER DEFINED CONFIGURATION
 */
#define MAX30100_CLR_WR_PTR_REG         (0x00)                               // Reset Write Pointer
#define MAX30100_CLR_RD_PTR_REG         (0x00)                               // Reset Read Pointer
#define MAX30100_CLR_OVF_CTR_REG        (0x00)                               // Reset Overflow control counter
#define MAX30100_RESET                  (0x72)                               // Enable Reset bit
#define MAX30100_SHDN                   (0x82)                               // Enable shdn bit
#define MAX30100_INTR_EN                (0x80)                               // FIFO Full - Enabled HR_RDY, TEP_RDY, SO2_RDY - Disabled
#define MAX30100_MODE_CONFIG            (0x02)                               // Mode : 010 (HR Only), shdn, reset, temp_en - 0
#define MAX30100_SPO2_CONFIG            (0x47)                               // HI_RES_EN - 1, Sample Rate - 001, Pulse Width - 11
#define MAX30100_LED_CONFIG             (0xFF)                               // Set IR and RED LED Current as 50mA (Value - 1111)

/* 
 *      USER DEFINED VALUES
 */
#define ZERO                            (0)                                  // Null Value
#define ONE                             (1)                                  // Unit Value
#define BIT_RESET                       (0)                                  // Reset bit
#define SET                             (1)                                  // Set bit
#define PRESSED                         (1)                                  // Switch press check
#define SAMPLE_LENGTH                   (16)                                 // FIFO Sample Length
#define SAMPLE_SIZE                     (4)                                  // Size of 4 bytes to be read from Sensor
#define SHIFT_BYTE                      (8)                                  // Higher Byte shift
#define RAW_DATA_SIZE                   (32)                                 // Raw Data array size
#define SAMPLE_COUNT1                   (17)                                 // Set-1 of raw data sample count
#define SAMPLE_COUNT2                   (15)                                 // Set-2 of raw data sample count
#define LAST_SAMPLE1                    (17)                                 // Last position of array-1
#define LAST_SAMPLE2                    (15)                                 // Last position of array-2
#define ARRAY_SHIFT                     (17)                                 // Shift added to move to next set of raw dat array
#define SW_DEBOUNCE_DELAY               (100)                                // Switch debouncing delay
#define DISPLAY_DELAY                   (2000)                               // 1sec delay to display the state on LCD
#define PROCESSING_DELAY                (4)                                  // 4ms delay to avoid overwriting or rewriting same data
#define INIT_IDX                        (0)

#define b                               (0x62)                               // LCD Characters
#define p	                            (0x70)
#define m	                            (0x6D)
/* 
 *      USER DEFINED STRUCTURES
 */
typedef enum {
    INIT,
    IDLE,
    MSSP_AS_SPI_1,
    MSSP_AS_I2C,
    CALCULATE_HR,
    MSSP_AS_SPI_2,
    DATA_DISP
} eventstate;

eventstate Current_State = INIT;

typedef struct {
    uint16_t v_ctr[2];
} filter_t;

/* 
 *      USER DEFINED FUNCTIONS
 */

/**
   @Param
    none
   @Returns
    none
   @Description
    Reads FIFO data from the sensor 
   @Example
    none
 */
void Sensor_ReadFifoData(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Clears read, write pointers and overflow counter
   @Example
    none
 */
void Sensor_ClearCounters(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Implements the I2C (Heart rate) Functionality
   @Example
    none
 */
void Application_Dynamic_Switching_I2C_SPI(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Initializes all the MAX30100 sensor registers
   @Example
    none
 */
void Initialize_I2C_Sensor(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Resets all the registers to power-on-state
   @Example
    none
 */
void I2C_Sensor_Reset(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Shutdown the device in power-saver mode
   @Example
    none
 */
void I2C_Sensor_Shutdown(void);

/**
   @Param
    current sample and previous sample
   @Returns
    low pass filtered value
   @Description
    Low Pass Butterworth Filter (first order)
   @Example
    none
 */
uint16_t Filter_BWLowPass(uint16_t curr_value, uint16_t prev_value);

/**
   @Param
    current sample
   @Returns
    shifted sample
   @Description
    Moves the sample to accommodate new sample
   @Example
    none
 */
int16_t Shift_Left(int16_t value);

/**
   @Param
    filtered sample array
   @Returns
    zero crossover count
   @Description
    Counts the zero crossovers
   @Example
    none
 */
int16_t Zero_Crossing(int16_t filter_value[]);

/**
   @Param
    none
   @Returns
    none
   @Description
    Sets the timer flag when interrupt occurs
   @Example
    none
 */
void TMR2_UserInterruptHandler(void);

/**
   @Param
    true or false (as per condition)
   @Returns
    none
   @Description
    Returns FIFO Full interrupt flag status
   @Example
    none
 */
bool I2C_Sensor_isFifoAfull(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Initialize pins for SPI Configuration
   @Example
    none
 */
void SPI_PIN_MANAGER_Initialize(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Initialize PPS for SPI configuration
   @Example
    none
 */
void SPI_PPS_Initialize(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    De-Initialize SPI Configuration PPS
   @Example
    none
 */
void SPI_PPS_DeInitialize(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    De-Initialize I2C Configuration PPS
   @Example
    none
 */
void I2C_PPS_DeInitialize(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Initialize SPI Configuration
   @Example
    none
 */
void Config_MSSP_as_SPI(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Initialize I2C Configuration
   @Example
    none
 */
void Config_MSSP_as_I2C(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Initialize EUSART & Pin Configuration
   @Example
    none
 */
void EUSART_Initialize(void);

/**
   @Param
    none
   @Returns
    none
   @Description
    Initialize LCD Configuration
   @Example
    none
 */
void LCD_Initialize(void);

/**
   @Param
 data string, row position, column position
   @Returns
    none
   @Description
    User defined LCD write string function to print data on LCD display
   @Example
    none
 */
void User_lcd_writeString(char* string, uint8_t row, uint8_t column);

#endif	/* APPLICATION_H */

