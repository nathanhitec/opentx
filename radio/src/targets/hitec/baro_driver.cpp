

#include "opentx.h"

#define BARO_CS_HIGH()                 BARO_GPIO->BSRRL = BARO_GPIO_CS_PIN
#define BARO_CS_LOW()                  BARO_GPIO->BSRRH = BARO_GPIO_CS_PIN

#define CMD_RESET 0x1E // ADC reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command
#define CMD_ADC_D1 0x00 // ADC D1 conversion
#define CMD_ADC_D2 0x10 // ADC D2 conversion
#define CMD_ADC_256 0x00 // ADC OSR=256
#define CMD_ADC_512 0x02 // ADC OSR=512
#define CMD_ADC_1024 0x04 // ADC OSR=1024
#define CMD_ADC_2048 0x06 // ADC OSR=2056
#define CMD_ADC_4096 0x08 // ADC OSR=4096
#define CMD_PROM_RD 0xA0 // Prom read command 

uint16_t coeffs[8];
uint8_t crc;

typedef struct {
    uint32_t D1, D2; //uncompensated Temp and Pressure
    int32_t T, P; //compensated Temp and Pressure
} BaroData;

void baroWriteCmd(uint8_t byte)
{
    BARO_CS_LOW();
    //while no errors and transmit buffer is not empty wait
    while ((SPI3->SR & SPI_SR_TXE) == 0) {
        // Wait
    }
    (void)SPI3->DR; // Clear receive
    LCD_SPI->DR = byte;
    //while no errors and recieve buffer is empty, wait
    while ((SPI3->SR & SPI_SR_RXNE) == 0) {
        // Wait
    }
    BARO_CS_HIGH();
}


uint8_t cmdPROM(uint8_t coeff_num)
{
    uint16_t ret;
    uint16_t rC = 0;

    baroWriteCmd(CMD_PROM_RD | (coeff_num << 1)); // send PROM READ command
    baroWriteCmd(0x00); // send 0 to read the MSB
    ret = SPI3->DR;
    rC = 256 * ret;
    baroWriteCmd(0x00); // send 0 to read the LSB
    ret = SPI3->DR;
    rC = rC + ret;
    return rC;
}

uint16_t cmdADC(uint8_t cmd)
{
    uint16_t ret;
    uint32_t result;
    baroWriteCmd(CMD_ADC_CONV + cmd); // send conversion command
    switch (cmd & 0x0f) // wait necessary conversion time
    {
        case CMD_ADC_256: delay_us(900); break;
        case CMD_ADC_512: delay_ms(3); break;
        case CMD_ADC_1024: delay_ms(4); break;
        case CMD_ADC_2048: delay_ms(6); break;
        case CMD_ADC_4096: delay_ms(10); break;
    }
    
    baroWriteCmd(CMD_ADC_READ); // send ADC read command
    baroWriteCmd(0x00); // send 0 to read 1st byte (MSB)
    ret = SPI3->DR;
    result = 65536 * ret;
    baroWriteCmd(0x00); // send 0 to read 2nd byte
    ret = SPI3->DR;
    result = result + 256 * ret;
    baroWriteCmd(0x00); // send 0 to read 3rd byte (LSB)
    ret = SPI3->DR;
    result = result + ret;
    return result;
}

//How frequently will this need to be ran?
void baroGetData(BaroData *bd)
{
    int32_t dT;
    int64_t OFF, SENS;

    bd->D1 = cmdADC(CMD_ADC_D1 + CMD_ADC_256); // read uncompensated pressure
    bd->D2 = cmdADC(CMD_ADC_D2 + CMD_ADC_4096); // read uncompensated temperature

    // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
    dT = bd->D2 - coeffs[5] * pow(2, 8);
    OFF = coeffs[2] * pow(2, 17) + dT * coeffs[4] / pow(2, 6);
    SENS = coeffs[1] * pow(2, 16) + dT * coeffs[3] / pow(2, 7);

    bd->T = (2000 + (dT * coeffs[6]) / pow(2, 23)) / 100;
    bd->P = (((bd->D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;
}


void baroHardwareInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = BARO_GPIO_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BARO_GPIO, &GPIO_InitStructure);

    /*
    BARO_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA to set regs
    BARO_DMA->HIFCR = BARO_DMA_FLAGS; // Write ones to clear bits
    BARO_DMA_Stream->CR = DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_2; //priority low, perf-mem, mem addr ptr incremeneted
    BARO_DMA_Stream->PAR = (uint32_t)&LCD_SPI->DR;
    //Num data items to transfer
    BARO_DMA_Stream->NDTR = 8;

    //direct mode, fifo thresh half
    BARO_DMA_Stream->FCR = 0x05; // DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;
    BARO_DMA_Stream->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; // Enable DMA & TC interrupts
    LCD_SPI->CR2 |= SPI_CR2_RXDMAEN;
    NVIC_EnableIRQ(BARO_DMA_Stream_IRQn);
    */
}


void baroGetCoeffs()
{
    for (int i = 0; i < 8; i++) {
        coeffs[i] = cmdPROM(i);
    }
}

void baroReset(void)
{
    baroWriteCmd(CMD_RESET); // send reset sequence 
    delay_ms(3); // wait for the reset sequence timing
}

uint8_t crc4(uint16_t n_prom[])
{
    int cnt; // simple counter
    uint8_t new_crc;
    uint16_t n_rem; // crc reminder
    uint16_t crc_read; // original value of the crc
    uint8_t n_bit;

    n_rem = 0x00;
    crc_read = n_prom[7]; //save read CRC
    n_prom[7] = (0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt % 2 == 1) n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem = (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
    n_prom[7] = crc_read; // restore the crc_read to its original place
    new_crc = (n_rem ^ 0x00);
    return new_crc;
}



void baroInit(void)
{
    baroHardwareInit();
    baroReset();
    baroGetCoeffs();
    crc = crc4(coeffs);
}

bool checkCRC()
{
    return (crc && 0x0F) == (coeffs[7] && 0x0F);
}


/*
extern "C" void BARO_DMA_Stream_IRQHandler()
{
    BARO_DMA_Stream->CR &= ~DMA_SxCR_TCIE; // Stop interrupt, transfer complete int disable
    BARO_DMA->HIFCR |= BARO_DMA_FLAG_INT; // Clear interrupt flag
}
*/