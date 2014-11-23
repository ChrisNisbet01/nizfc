#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stm32f30x_gpio.h>
#include <spi_stm32f30x.h>
#include <sensors.h>
#include <l3gd20.h>
#include <utils.h>

#define L3GD20_SendByte(byte) spiSendByte(l3gd20ctx.spiCtx, byte)

#define L3GD20_SPI_CS_PIN                GPIO_Pin_3                  /* PE.03 */
#define L3GD20_SPI_CS_GPIO_PORT          GPIOE                       /* GPIOE */
#define L3GD20_SPI_CS_GPIO_CLK           RCC_AHBPeriph_GPIOE
#define L3GD20_CS_LOW()       GPIO_ResetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN)
#define L3GD20_CS_HIGH()      GPIO_SetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN)

#define L3G_Sensitivity_250dps     114.285f       /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     57.1429f       /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

#define NB_GYRO_CAL_SAMPLES			100

typedef struct l3gd20Ctx_st
{
	void *spiCtx;
	float gyroSensitivity;
	int16_t zero_bias[3];
} l3gd20Ctx_st;

static l3gd20Ctx_st l3gd20ctx;

/**
  * @brief  Reads a block of data from the L3GD20.
  * @param  pBuffer : pointer to the buffer that receives the data read from the L3GD20.
  * @param  ReadAddr : L3GD20's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the L3GD20.
  * @retval None
  */
bool L3GD20_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	bool result = false;
	if(NumByteToRead > 1)
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
	if ( L3GD20_SendByte(ReadAddr) < 0 )
		goto done;

	/* Receive the data that will be read from the device (MSB First) */
	while(NumByteToRead > 0)
	{
		/* Send dummy byte (0x00) to generate the SPI clock to L3GD20 (Slave device) */
		int ch = L3GD20_SendByte(DUMMY_BYTE);

		if ( ch < 0 )
			goto done;
		*pBuffer = (uint8_t)ch;
		NumByteToRead--;
		pBuffer++;
	}

	result = true;

done:
	/* Set chip select High at the end of the transmission */
	L3GD20_CS_HIGH();

	return result;
}

/**
  * @brief  Writes one byte to the L3GD20.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the L3GD20.
  * @param  WriteAddr : L3GD20's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
bool L3GD20_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	bool result = false;

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
	if ( L3GD20_SendByte(WriteAddr) < 0 )
		goto done;
	/* Send the data that will be written into the device (MSB First) */
	while(NumByteToWrite >= 0x01)
	{
		if ( L3GD20_SendByte(*pBuffer) < 0 )
			goto done;
		NumByteToWrite--;
		pBuffer++;
	}

	result = true;

done:
  /* Set chip select High at the end of the transmission */
  L3GD20_CS_HIGH();

  return result;
}

static bool L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct)
{
	bool result = false;
	uint8_t ctrl1 = 0x00, ctrl4 = 0x00;

	/* Configure MEMS: data rate, power mode, full scale and axes */
	ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
	                L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);

	ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
	                L3GD20_InitStruct->Full_Scale);

	/* Write value to MEMS CTRL_REG1 regsister */
	if ( L3GD20_Write(&ctrl1, L3GD20_CTRL_REG1_ADDR, 1) == false )
		goto done;

	/* Write value to MEMS CTRL_REG4 regsister */
	if ( L3GD20_Write(&ctrl4, L3GD20_CTRL_REG4_ADDR, 1) == false )
		goto done;

	result = true;
done:

	return result;
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  L3GD20_FilterStruct: pointer to a L3GD20_FilterConfigTypeDef structure
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
bool L3GD20_FilterConfig(L3GD20_FilterConfigTypeDef *L3GD20_FilterStruct)
{
	uint8_t tmpreg = 0;	/* avoid warning */
	bool result = false;

	/* Read CTRL_REG2 register */
	if ( L3GD20_Read(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1) == false )
		goto done;

	tmpreg &= 0xC0;

	/* Configure MEMS: mode and cutoff frquency */
	tmpreg |= (uint8_t) (L3GD20_FilterStruct->HighPassFilter_Mode_Selection |\
	                  L3GD20_FilterStruct->HighPassFilter_CutOff_Frequency);

	/* Write value to MEMS CTRL_REG2 regsister */
	if ( L3GD20_Write(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1) == false )
		goto done;

	result = true;
done:
	return result;
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be:
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE
  * @retval None
  */
bool L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
	uint8_t tmpreg = 0;	/* avoid warning */
	bool result = false;

	/* Read CTRL_REG5 register */
	if ( L3GD20_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1) == false )
		goto done;

	tmpreg &= 0xEF;

	tmpreg |= HighPassFilterState;

	/* Write value to MEMS CTRL_REG5 regsister */
	if ( L3GD20_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1) == false )
		goto done;

	result = true;

done:
	return result;
}

/**
  * @brief  Get status for L3GD20 data
  * @param  None
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg = 0;

  /* Read STATUS_REG register */
  L3GD20_Read(&tmpreg, L3GD20_STATUS_REG_ADDR, 1);

  return tmpreg;
}

static bool readRaw( int16_t rawbuf[3] )
{
	bool readGyro = false;
	uint8_t buf[6];

	if ( L3GD20_Read( buf, L3GD20_OUT_X_L_ADDR, sizeof(buf) ) == false )
		goto done;

    rawbuf[0] = (int16_t)((uint16_t)buf[1] << 8) + buf[0];
    rawbuf[1] = (int16_t)((uint16_t)buf[3] << 8) + buf[2];
    rawbuf[2] = (int16_t)((uint16_t)buf[5] << 8) + buf[4];

	readGyro = true;
done:
	return readGyro;
}

static bool l3gd20ReadGyro( void * pv, gyrometer_sensor_t *gyrometer )
{
	l3gd20Ctx_st * l3gCtx = pv;
	bool readGyro = false;
    int16_t buf[3];

	if ( readRaw( buf ) == false )
		goto done;

    gyrometer[0] = (float)(buf[0] - l3gCtx->zero_bias[0]) / l3gCtx->gyroSensitivity;
    gyrometer[1] = (float)(buf[1] - l3gCtx->zero_bias[1]) / l3gCtx->gyroSensitivity;
    gyrometer[2] = (float)(buf[2] - l3gCtx->zero_bias[2]) / l3gCtx->gyroSensitivity;

	readGyro = true;
done:
	return readGyro;
}

static bool calculateGyroZeroBias( l3gd20Ctx_st * l3gCtx )
{
	bool doneZeroBias = false;
	int i;
	int32_t largebuf[3] = {0,0,0};
	int16_t buf[3];

	for (i = 0; i < NB_GYRO_CAL_SAMPLES; i++ )
	{
		long tmo = 4;
		while ((L3GD20_GetDataStatus() & 0x08) == 0)	/* wait for new xyz axis data */
		{
			if ( tmo-- == 0 )
				goto done;
			delayMilliseconds( 10 );
		}
		if ( readRaw( buf ) == false )
			goto done;
		/* any shaking of the board resets things */
		if ( fabs((float)buf[0] / l3gCtx->gyroSensitivity) > 10.0f )
		{
			i = 0;
			largebuf[0] = 0;
			largebuf[1] = 0;
			largebuf[2] = 0;
		}
		else
		{
			largebuf[0] += buf[0];
			largebuf[1] += buf[1];
			largebuf[2] += buf[2];
		}
	}

	l3gCtx->zero_bias[0] = (largebuf[0]+i/2)/i;
	l3gCtx->zero_bias[1] = (largebuf[1]+i/2)/i;
	l3gCtx->zero_bias[2] = (largebuf[2]+i/2)/i;

	doneZeroBias = true;

done:
	return doneZeroBias;
}

void * initL3GD20( sensorConfig_st *config, sensorCallback_st *callbacks )
{
	l3gd20Ctx_st * l3gCtx = &l3gd20ctx;

	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	l3gCtx->spiCtx = config->spiCtx;

	/* Configure GPIO PIN for Lis Chip select */
	GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(L3GD20_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Deselect : Chip Select high */
	L3GD20_CS_HIGH();

	/* Configure Mems L3GD20 */
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_2000;
	l3gCtx->gyroSensitivity = L3G_Sensitivity_2000dps;	// TODO: configurable
	if ( L3GD20_Init(&L3GD20_InitStructure) == false )
		goto error;

	L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	if ( L3GD20_FilterConfig(&L3GD20_FilterStructure) == false )
		goto error;

	if ( L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE) == false )
		goto error;

	if ( calculateGyroZeroBias( l3gCtx ) == false )
		goto error;

	callbacks->readGyro = l3gd20ReadGyro;

	return l3gCtx;
error:
	return NULL;
}

