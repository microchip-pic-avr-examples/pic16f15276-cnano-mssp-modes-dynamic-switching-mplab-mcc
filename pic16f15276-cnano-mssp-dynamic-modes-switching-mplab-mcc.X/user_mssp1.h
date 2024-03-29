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

// This is a guard condition so that contents of this file are not included more than once.  

#ifndef SPI1_H
#define SPI1_H

/**
 * Section: Included Files
 */ 

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "user_spi_interface.h"

/**
 * @ingroup spi1
 * @struct SPI_INTERFACE SPI1
 * @brief SPI driver interface object.
 */                
extern const struct SPI_INTERFACE SPI1;

/**
 * @ingroup spi1
 * @typedef enum spi1_modes_t
 * @brief Enumeration of the different configurations supported by the driver.A configuration is specified as parameter to SPI1_Open()
 *        and is used by the function to set SPI parameters as specified by the configuration.
 */
typedef enum { 
    MSSP1_DEFAULT
} spi1_modes_t;

/**
 * @ingroup spi1
 * @brief Initialize SPI interface.
 * @param none
 * @return none
 */
void SPI1_Initialize(void);

/**
 * @ingroup spi1
 * @brief Sets the index of Configuration to use in the transfer.
 * @param uint8_t spiConfigIndex - The configuration index
 * @retval true  - SPI status was open.
 * @retval false - SPI status was not open.
 */
bool SPI1_Open(uint8_t spiConfigIndex);

/**
 * @ingroup spi1
 * @brief Close the SPI for communication.
 * @param none
 * @return none
 */
void SPI1_Close(void);

/**
 * @ingroup spi1
 * @brief Exchange one byte over SPI. Blocks until done.
 * @param uint8_t byteData - The byte to transfer
 * @return uint8_t - Received data byte.
 */
uint8_t SPI1_ByteExchange(uint8_t byteData);

/**
 * @ingroup spi1
 * @brief Exchange a buffer over SPI. Blocks if using polled driver.
 * @param[inout] void * bufferData The buffer to transfer. Received data is returned here
 * @param[in] size_t bufferSize The size of buffer to transfer
 * @return none
 */
void SPI1_BufferExchange(void * bufferData, size_t bufferSize);

/**
 * @ingroup spi1
 * @brief Write a buffer over SPI. Blocks if using polled driver.
 * @param[in] void * bufferData The buffer to transfer
 * @param[in] size_t bufferSize The size of buffer to transfer
 * @return none
 */
void SPI1_BufferWrite(void * bufferData, size_t bufferSize);

/**
 * @ingroup spi1
 * @brief Read a buffer over SPI. Blocks if using polled driver.
 * @param[out] void * bufferData Received data is written here.
 * @param[in] size_t bufferSize The size of buffer to transfer
 * @return none
 */
void SPI1_BufferRead(void * bufferData, size_t bufferSize);

/**
 * @ingroup spi1
 * @brief Write data byte to SPI.
 * @param uint8_t byteData The byte to transfer.
 * @return none
 */
void SPI1_ByteWrite(uint8_t byteData);

/**
 * @ingroup spi1
 * @brief Get received data byte from SPI.
 * @param none
 * @return uint8_t - The received data
 */
uint8_t SPI1_ByteRead(void);

uint8_t __attribute__((deprecated)) SPI1_ExchangeByte(uint8_t data);
void __attribute__((deprecated)) SPI1_ExchangeBlock(void *block, size_t blockSize);
void __attribute__((deprecated)) SPI1_WriteBlock(void *block, size_t blockSize);
void __attribute__((deprecated)) SPI1_ReadBlock(void *block, size_t blockSize);
void __attribute__((deprecated)) SPI1_WriteByte(uint8_t byte);
uint8_t __attribute__((deprecated)) SPI1_ReadByte(void);

#endif //SPI1_H