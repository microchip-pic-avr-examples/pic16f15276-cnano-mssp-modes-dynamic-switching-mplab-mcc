#include <xc.h>
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/i2c_host/mssp1.h"
#include "application.h"

#define PRINT_DEBUG

/*
 ***********************************************************
 *                                                         *
 *              User Defined Variables                     *
 *                                                         *
 ***********************************************************
*/
static uint8_t fifo_wr_ptr;
static uint8_t fifo_rd_ptr;
static uint8_t ovf_ctr;

bool timer_flag;
static uint8_t user_flag;

static uint16_t iterator;
static uint16_t zero_cross;
static int16_t prev_value;
static uint16_t raw_idx;
static int16_t prev_avgf_value;

static uint16_t raw_ir_data[RAW_DATA_SIZE];
static int16_t dc_data[17];
static int16_t lpbwf_data[18];
static int16_t summation_filter[18];
static int16_t temp[8];

/*
 ***********************************************************
 *                                                         *
 *              Static Function Declarations               *
 *                                                         *
 ***********************************************************
*/

static void I2C_Sensor_WriteData(uint8_t reg_addr, uint8_t data);
static uint8_t I2C_Sensor_ReadData(uint8_t reg_addr);
static void I2C_Sensor_ReadDataBlock(uint8_t reg_addr, uint8_t *read_buff, uint8_t length);
static void HeartRateClick_Initialize(void);

/*
 ***********************************************************
 *                                                         *
 *          User Defined Function Definitions              *
 *                                                         *
 ***********************************************************
*/

static void HeartRateClick_Initialize(void)
{
    I2C_Sensor_Reset();    
    I2C_Sensor_WriteData(MAX30100_INTERRUPT_EN_REG, MAX30100_INTR_EN);
    I2C_Sensor_WriteData(MAX30100_MODE_CONFIG_REG, MAX30100_MODE_CONFIG);
    I2C_Sensor_WriteData(MAX30100_SPO2_CONFIG_REG, MAX30100_SPO2_CONFIG);    
    I2C_Sensor_WriteData(MAX30100_LED_CONFIG_REG, MAX30100_LED_CONFIG);
}

void I2C_Sensor_Reset(void) 
{
    I2C_Sensor_WriteData(MAX30100_MODE_CONFIG_REG, MAX30100_RESET);
}

void I2C_Sensor_Shutdown(void) 
{
    I2C_Sensor_WriteData(MAX30100_MODE_CONFIG_REG, MAX30100_SHDN);
}

void Initialize_I2C_Sensor(void)
{
    HeartRateClick_Initialize();
}

void Application_Dynamic_Switching_I2C_SPI(void) 
{
    uint8_t dataBuffer[6] = {0};     

    switch(Current_State)
    {
        case INIT:
            Config_MSSP_as_SPI();                                            // Default state to display LCD message
            LCD_Initialize();												 // Initialize LCD 
            lcd_writeString((uint8_t*)" MSSP SWITCHING ",0);
#ifdef PRINT_DEBUG
            printf(" \r\nMSSP as SPI ");
#endif 
            __delay_ms(DISPLAY_DELAY);
            lcd_writeString((uint8_t*)"PRESS SW FOR HR ",1);                 // Press switch to initiate the heart rate counting process
#ifdef PRINT_DEBUG
            printf(" \r\nPRESS SW FOR HR ");
#endif
            __delay_ms(DISPLAY_DELAY);
            Config_MSSP_as_I2C();
            EUSART_Initialize();
            Current_State = IDLE;
            
        case IDLE:                                                           // Do nothing until the state change is initiated by switch press
            if (SW_GetValue() != 1)                                            
            {
                __delay_ms(SW_DEBOUNCE_DELAY);								 // Switch de-bouncing delay
                Current_State = MSSP_AS_SPI_1;								 // Move from Idle state to MSSP_AS_SPI_1
            }
            break;
        
        case MSSP_AS_SPI_1:                                                  // Initialize MSSP for SPI Mode
            Config_MSSP_as_SPI();
            LCD_Initialize();												 // Initialize LCD 
            lcd_writeString((uint8_t*)" SWITCHING MODE ",0);
#ifdef PRINT_DEBUG
            printf(" \r\nMSSP as I2C ");
#endif
            __delay_ms(DISPLAY_DELAY);
            Current_State = MSSP_AS_I2C;									 // Move from MSSP_AS_SPI_1 to MSSP_AS_I2C state for acquiring data via I2C
            break;
        
        case MSSP_AS_I2C:                                                    // Switch to I2C mode
            lcd_clearDisplay();                                              // Initialize MSSP for I2C Mode
            lcd_writeString((uint8_t*)"SWITCHING TO I2C",0);
            __delay_ms(DISPLAY_DELAY);
            lcd_writeString((uint8_t*)"   MEASURING    ",0);
            lcd_writeString((uint8_t*)"   HEART RATE   ",1);
#ifdef PRINT_DEBUG
            printf(" \r\nCalculating Heart rate ");
#endif
            Config_MSSP_as_I2C();
            EUSART_Initialize();                                             // EUSART & Pin Config Initialize
            Current_State = CALCULATE_HR;									 // Move to HR data acquiring and calculating state
            break;
        
        case CALCULATE_HR:                                                   // Start Reading & Calculating data
            Timer2_Start();
            while(!timer_flag)
            {
                if(I2C_Sensor_isFifoAfull())								 // Continuously monitor the overflow flag to avoid data loss
                {
                    Sensor_ReadFifoData();                                   // Acquire Data from the sensor FIFO Buffer
                    LED_Toggle();											 // Visual indication 
                    Sensor_ClearCounters();                                  // Clear the memory pointers on sensor side
                }
            }        
            Timer2_Stop();       
            timer_flag = false;
            Current_State = MSSP_AS_SPI_2;                          		 // Move to re configuring MSSP to SPI for data display
            break;
        
        case MSSP_AS_SPI_2:                                                  // Re-Initialize MSSP for SPI Mode to display the data
            Config_MSSP_as_SPI();
            Current_State = DATA_DISP;									 	 // Move to Data Display phase
            break;
        
        case DATA_DISP:                                                      // Display Data on LCD screen
            LCD_Initialize();
            lcd_writeString((uint8_t*)"SWITCHING TO SPI",0);
#ifdef PRINT_DEBUG
            printf(" \r\nMSSP as SPI ");
#endif
            __delay_ms(DISPLAY_DELAY);
            lcd_clearDisplay();
            User_lcd_writeString((char*)"Hrt Rate:        ",0,0);
            sprintf((char*)dataBuffer,"%3d%c%c%c   ",(zero_cross/10),b,p,m); // Display the result  
            User_lcd_writeString((char*)dataBuffer,0,9);
#ifdef PRINT_DEBUG
            printf(" \r\nDisplaying heart rate on LCD ");
#endif
            zero_cross = BIT_RESET;
            __delay_ms(DISPLAY_DELAY);
            Config_MSSP_as_I2C();
            Current_State = INIT;       									 // Move to Idle phase
            break;       
    }    
}

void Sensor_ClearCounters(void)
{
    I2C_Sensor_WriteData(MAX30100_FIFO_WR_PTR_REG, MAX30100_CLR_WR_PTR_REG);
    I2C_Sensor_WriteData(MAX30100_OVF_CTR_REG, MAX30100_CLR_OVF_CTR_REG);
    I2C_Sensor_WriteData(MAX30100_FIFO_RD_PTR_REG, MAX30100_CLR_RD_PTR_REG);
}

void Sensor_ReadFifoData(void)
{
    uint32_t sum, avg;
    uint16_t ir_data, red_data;
    uint8_t fifo_buffer[16];
    for(int sampl_set_idx = 0; sampl_set_idx < 2; sampl_set_idx++)           // Accept 64B of raw data(16 samples)
    {
        for(int samp_idx=0; samp_idx<SAMPLE_LENGTH; samp_idx++)
        {
            I2C_Sensor_ReadDataBlock(MAX30100_FIFO_DATA_REG, fifo_buffer, SAMPLE_SIZE);    
            ir_data = (fifo_buffer[0] << SHIFT_BYTE) | fifo_buffer[1];       // First 2 bytes are IR LED Data
            red_data = (fifo_buffer[2] << SHIFT_BYTE) | fifo_buffer[3];      // Next 2 bytes are Red LED Data
            raw_ir_data[raw_idx] = ir_data;                                  // Stores current sample in array of size - 32 samples
            raw_idx++;                                                       // Index of the raw data array will increment until 2 sets of 16 are not filled in the local array
            __delay_ms(PROCESSING_DELAY);                                    // Delay is provided for avoiding overlapping and rewriting the previous data
        }

        if(iterator%2 == ONE)                                                // Perform post-processing after even set of data are accumulated
        {
            lpbwf_data[0] = prev_value;
            summation_filter[0] = prev_avgf_value;
            for(int arr0_index=0; arr0_index<SAMPLE_COUNT1;arr0_index++)     // Summation loop for avg calculation for DC filtering 
            {
                sum = BIT_RESET;                                             // Reset the sum of the previous calculation
                for(int sum0_fltr_idx=0;sum0_fltr_idx<16;sum0_fltr_idx++)
                {
                    sum += raw_ir_data[arr0_index+sum0_fltr_idx];            // Calculate the sum of raw IR samples
                }
                avg = sum/SAMPLE_LENGTH;
                dc_data[arr0_index] = raw_ir_data[arr0_index] - avg;         // the DC shifted signal is calculated wrt moving avg
                summation_filter[arr0_index+1] = Shift_Left(dc_data[arr0_index+1]);
                lpbwf_data[arr0_index+1] = Filter_BWLowPass(-summation_filter[arr0_index], -summation_filter[arr0_index-1]);                
            }
            prev_value = lpbwf_data[LAST_SAMPLE1];                           // The last sample from lpbwf array set of 16 samples is required for further calculations
            prev_avgf_value = summation_filter[LAST_SAMPLE1];                // The last sample from summation array set of 16 samples is required for further calculations
            zero_cross += Zero_Crossing(lpbwf_data);                         // Increments as soon as the pulse receives a zero crossing due to heart beat detection
            iterator++;
            raw_idx = BIT_RESET;                                             // Raw data array will get overwritten after 2 cycles
        }
        else if(iterator == ZERO)                                            // Default condition to accumulate first set of 16 samples
        {
            iterator++;                                                      // Variable used to differentiate between even odd sets of samples
            continue;
        }
        else                                                                 // Perform post-processing after odd set of data are accumulated
        {
            summation_filter[16] = ZERO;                                     // Initial samples require zero padding for calculating the moving sum                                        
            summation_filter[LAST_SAMPLE1] = ZERO;
            summation_filter[INIT_IDX] = prev_avgf_value;                    // First location will carry the last value of previous set of data for summation filter

            lpbwf_data[LAST_SAMPLE1] = ZERO;                                 // Initial samples require zero padding for calculation
            lpbwf_data[16] = ZERO;
            lpbwf_data[INIT_IDX] = prev_value;                               // First location will carry the last value of previous set of data for lpbwf
            for(int arr1_index=0; arr1_index<SAMPLE_COUNT2;arr1_index++)     // Summation loop for avg for DC filtering 
            {
                sum = BIT_RESET;                                             // Reset the sum of the previous calculation
                for(int sum1_fltr_idx=0;sum1_fltr_idx<SAMPLE_LENGTH;sum1_fltr_idx++)
                {
                    sum += raw_ir_data[(arr1_index+sum1_fltr_idx+17)%RAW_DATA_SIZE];    // The index of the array needs to be shifted by 17 to acquire data from 32 sized raw data array 
                }
                avg = (sum/SAMPLE_LENGTH);
                dc_data[arr1_index] = raw_ir_data[arr1_index+17] - avg;      // the DC shifted signal is calculated wrt moving avg
                summation_filter[arr1_index+1] = dc_data[arr1_index+1];
                lpbwf_data[arr1_index+1] = Filter_BWLowPass(-summation_filter[arr1_index], -summation_filter[arr1_index-1]);                
            }
            prev_value = lpbwf_data[LAST_SAMPLE2];                           // Last LPBWF value is stored
            prev_avgf_value = summation_filter[LAST_SAMPLE2];                // Last summation filter value is stored
            zero_cross += Zero_Crossing(lpbwf_data);                         // Adds up the zero crossing count
            iterator++;
        }	
    }
    Sensor_ClearCounters();                                                  // Clears the read, write pointers and overflow counter
}

int16_t Zero_Crossing(int16_t filter_value[])
{
    int16_t value, count = ZERO;
	for(int i=0;i<SAMPLE_COUNT1;i++)
	{
		value = filter_value[i] * filter_value[i+1];                         
		if(value < 0)                                                        // Determine if the consecutive value involves zero crossing
		{
			count++;
		}
	}
	return count;                                                            // Return total zero crossings
}

int16_t Shift_Left(int16_t value)
{
	int16_t post_dc_filter_sum = ZERO;                       				 // n-1 = 7: size to fill array
    for(int shft_arr_index=0 ; shft_arr_index < 7; shft_arr_index++)
    {
        temp[shft_arr_index] = temp[shft_arr_index+1];
		post_dc_filter_sum += temp[shft_arr_index];
    }
    temp[7] = value;                                                         // Add the current element getting introduced in the array
	post_dc_filter_sum += temp[7];
	return post_dc_filter_sum;
}

uint16_t Filter_BWLowPass(uint16_t curr_value, uint16_t prev_value)          
{
    filter_t irFilters;
    irFilters.v_ctr[0] = prev_value;                                         // Low Pass Butterworth Filter
    irFilters.v_ctr[1] = curr_value;                                         // Fs = 100Hz, Fc = 10Hz, 1st order
        
    irFilters.v_ctr[1] = (irFilters.v_ctr[1]) + (0.5 * irFilters.v_ctr[0]);  // Scaling values are rounded off for efficient & fast processing
    return (irFilters.v_ctr[0] + irFilters.v_ctr[1]);
}

bool I2C_Sensor_isFifoAfull(void) 
{
    bool value = I2C_Sensor_ReadData(MAX30100_INTERRUPT_STAT_REG);           // Monitor the flag to check if FIFO is full
    return value;                                                            // Reading Interrupt Register clears the bits
}

void TMR2_UserInterruptHandler(void)                                         // User defined interrupt function to trigger the 10secs count
{
    timer_flag = true;
}

/*
 ***********************************************************
 *                                                         *
 *              I2C Function Definitions                   *
 *                                                         *
 ***********************************************************
*/

static void I2C_Sensor_WriteData(uint8_t reg_addr, uint8_t data) 
{
    bool retStatus;
    uint8_t txBuffer[2] = {0};
    txBuffer[0] = reg_addr;
    txBuffer[1] = data;
    retStatus = I2C1_Write(MAX30100_ADDR, txBuffer, 2);
    if (retStatus)
    {
        while (I2C1_IsBusy())
        {
            I2C1_Tasks(); 
        }
    }
    __delay_ms(5);
}

static uint8_t I2C_Sensor_ReadData(uint8_t reg_addr) 
{
    bool retStatus;
    uint8_t readByte;
    retStatus =  I2C1_WriteRead(MAX30100_ADDR, &reg_addr,1,&readByte,1);
    if (retStatus)
    {
        while (I2C1_IsBusy())
        {
            I2C1_Tasks(); 
        }      
    }
    return readByte;
}

static void I2C_Sensor_ReadDataBlock(uint8_t reg_addr, uint8_t *read_buff, uint8_t length) 
{
    bool retStatus;
    retStatus =  I2C1_WriteRead(MAX30100_ADDR, &reg_addr,1,read_buff,length);
    if (retStatus)
    {
        while (I2C1_IsBusy())
        {
            I2C1_Tasks(); 
        }      
    }
}

void SPI_PIN_MANAGER_Initialize(void)
{
    /**
    TRISx registers
    */
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISC = 0x3B;
    TRISD = 0x7E;
    TRISE = 0xF;

    /**
    ANSELx registers
    */
    ANSELA = 0xFF;
    ANSELB = 0xFF;
    ANSELC = 0x1F;
    ANSELD = 0x7E;
    ANSELE = 0x7;

    /**
    WPUx registers
    */
    WPUA = 0x0;
    WPUB = 0x0;
    WPUC = 0x0;
    WPUD = 0x0;
    WPUE = 0x0;
}

void SPI_PPS_Initialize(void)
{
    SSP1CLKPPS = 0x16;                                                       // RC6->MSSP1:SCK1;    
    SSP1DATPPS = 0x15;                                                       // RC5->MSSP1:SDI1;
    RC6PPS     = 0x07;                                                       // RC6->MSSP1:SCK1;
    RC2PPS     = 0x08;                                                       // RC2->MSSP1:SDO1;
}

void SPI_PPS_DeInitialize(void)
{
    SSP1CLKPPS = 0x00;                                                       // RC6->MSSP1:SCK1;    
    RC2PPS     = 0x00;                                                       // RC2->MSSP1:SDO1;    
    RC6PPS     = 0x00;                                                       // RC6->MSSP1:SCK1;    
    SSP1DATPPS = 0x00;                                                       // RC5->MSSP1:SDI1;   
}

void I2C_PPS_DeInitialize(void)
{
    RX1PPS      = 0x00;                                                      // RB0->EUSART1:RX1;
    RB1PPS      = 0x00;                                                      // RB1->EUSART1:TX1;
    SSP1CLKPPS  = 0x00;                                                      // RC3->MSSP1:SCL1;
    RC3PPS      = 0x00;                                                      // RC3->MSSP1:SCL1;
    SSP1DATPPS  = 0x00;                                                      // RC4->MSSP1:SDA1;
    RC4PPS      = 0x00;                                                      // RC4->MSSP1:SDA1;
}

void Config_MSSP_as_SPI(void)
{
    I2C_PPS_DeInitialize();    
    __delay_ms(20);
    SPI1_Initialize();
    SPI_PIN_MANAGER_Initialize();
    SPI_PPS_Initialize();
    __delay_ms(20);
}

void LCD_Initialize(void)
{
    lcd_setup();
    lcd_setContrast(0x20);                                                   // Set the LCD Contrast for Clear visibility 
    lcd_clearDisplay();                                                      // Clear the LCD Memory if Memory filled with any previous value
}

void Config_MSSP_as_I2C(void)
{
    SPI_PPS_DeInitialize();
    __delay_ms(20);
    I2C1_Initialize(); 
    PIN_MANAGER_Initialize();
    __delay_ms(100);
}

void EUSART_Initialize(void)
{
    EUSART1_Initialize();
}

void User_lcd_writeString(char* string, uint8_t row, uint8_t column) 
{
    lcd_setAddr(row, column);
    uint8_t i = 0;
    for (i = 0; i < 16; i++) {
        if (string[i]) {
            lcd_writeChar(string[i]);
        }
    }
    lcd_returnHome();
}
