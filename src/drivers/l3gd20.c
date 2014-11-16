#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_spi.h>
#include <sensors.h>
#include <l3gd20.h>

#define L3GD20_SendByte(byte) spiSendByte(l3gd20ctx.spiCtx, byte)

#define L3GD20_SPI_CS_PIN                GPIO_Pin_3                  /* PE.03 */
#define L3GD20_SPI_CS_GPIO_PORT          GPIOE                       /* GPIOE */
#define L3GD20_SPI_CS_GPIO_CLK           RCC_AHBPeriph_GPIOE
#define L3GD20_CS_LOW()       GPIO_ResetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN)
#define L3GD20_CS_HIGH()      GPIO_SetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN)

typedef struct l3gd20Ctx_st
{
	void *spiCtx;
} l3gd20Ctx_st;

static l3gd20Ctx_st l3gd20ctx;

/**
  * @brief  Reads a block of data from the L3GD20.
  * @param  pBuffer : pointer to the buffer that receives the data read from the L3GD20.
  * @param  ReadAddr : L3GD20's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the L3GD20.
  * @retval None
  */
void L3GD20_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  L3GD20_CS_LOW();

  /* Send the Address of the indexed register */
  L3GD20_SendByte(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to L3GD20 (Slave device) */
    *pBuffer = L3GD20_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  L3GD20_CS_HIGH();
}

/**
  * @brief  Writes one byte to the L3GD20.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the L3GD20.
  * @param  WriteAddr : L3GD20's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void L3GD20_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit:
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  L3GD20_CS_LOW();

  /* Send the Address of the indexed register */
  L3GD20_SendByte(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    L3GD20_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  L3GD20_CS_HIGH();
}

static void L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct)
{
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;

  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
                    L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);

  ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
                    L3GD20_InitStruct->Full_Scale);

  /* Write value to MEMS CTRL_REG1 regsister */
  L3GD20_Write(&ctrl1, L3GD20_CTRL_REG1_ADDR, 1);

  /* Write value to MEMS CTRL_REG4 regsister */
  L3GD20_Write(&ctrl4, L3GD20_CTRL_REG4_ADDR, 1);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  L3GD20_FilterStruct: pointer to a L3GD20_FilterConfigTypeDef structure
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20_FilterConfig(L3GD20_FilterConfigTypeDef *L3GD20_FilterStruct)
{
  uint8_t tmpreg;

  /* Read CTRL_REG2 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);

  tmpreg &= 0xC0;

  /* Configure MEMS: mode and cutoff frquency */
  tmpreg |= (uint8_t) (L3GD20_FilterStruct->HighPassFilter_Mode_Selection |\
                      L3GD20_FilterStruct->HighPassFilter_CutOff_Frequency);

  /* Write value to MEMS CTRL_REG2 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be:
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void L3GD20_FilterCmd(uint8_t HighPassFilterState)
 {
  uint8_t tmpreg;

  /* Read CTRL_REG5 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);

  tmpreg &= 0xEF;

  tmpreg |= HighPassFilterState;

  /* Write value to MEMS CTRL_REG5 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}

/**
  * @brief  Get status for L3GD20 data
  * @param  None
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg;

  /* Read STATUS_REG register */
  L3GD20_Read(&tmpreg, L3GD20_STATUS_REG_ADDR, 1);

  return tmpreg;
}

void initL3GD20( sensorConfig_st *config, sensorCallback_st *callbacks )
{
	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure GPIO PIN for Lis Chip select */
	GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(L3GD20_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Deselect : Chip Select high */
	L3GD20_CS_HIGH();

	l3gd20ctx.spiCtx = config->spiCtx;
	/* Configure Mems L3GD20 */
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
	L3GD20_Init(&L3GD20_InitStructure);

	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

