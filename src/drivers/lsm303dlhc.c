#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <utils.h>
#include <i2c_stm32f30x.h>
#include <sensors.h>
#include <lsm303dlhc.h>

#define LSM_Acc_Sensitivity_2g     (float)     0.001f            /*!< accelerometer sensitivity with 2 g full scale [g/LSB] */
#define LSM_Acc_Sensitivity_4g     (float)     0.002f            /*!< accelerometer sensitivity with 4 g full scale [g/LSB] */
#define LSM_Acc_Sensitivity_8g     (float)     0.004f            /*!< accelerometer sensitivity with 8 g full scale [g/LSB] */
#define LSM_Acc_Sensitivity_16g    (float)     0.012f            /*!< accelerometer sensitivity with 16 g full scale [g/LSB] */

/* read write funtions */
#define LSM303DLHC_Read(DeviceAddr,RegAddr,pBuffer,NumByteToRead)	i2cRead(lsm303dlhcCtx.i2cCtx, DeviceAddr, RegAddr, pBuffer, NumByteToRead)
#define LSM303DLHC_Write(DeviceAddr, RegAddr, pBuffer)	i2cWrite(lsm303dlhcCtx.i2cCtx, DeviceAddr, RegAddr, *pBuffer)

typedef struct lsm303dlhcContext_st
{
	void *i2cCtx;
	float accelerometerSensitivity;		/* g/LSB */
	float magnetometerXYSensitivity;
	float magnetometerZSensitivity;
} lsm303dlhcContext_st;

static lsm303dlhcContext_st lsm303dlhcCtx;


/**
  * @brief  Set LSM303DLHC Mag Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_MagInitTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
static bool LSM303DLHC_MagInit(LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct)
{
	bool magInitOK = false;
	uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;

	/* Assumes that the I2C driver has been Initialised */

	/* Configure MEMS: temp and Data rate */
	cra_regm |= (uint8_t) (LSM303DLHC_InitStruct->Temperature_Sensor | LSM303DLHC_InitStruct->MagOutput_DataRate);

	/* Configure MEMS: full Scale */
	crb_regm |= (uint8_t) (LSM303DLHC_InitStruct->MagFull_Scale);

	/* Configure MEMS: working mode */
	mr_regm |= (uint8_t) (LSM303DLHC_InitStruct->Working_Mode);

	/* Write value to Mag MEMS CRA_REG regsister */
	if ( LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, &cra_regm) == false )
		goto done;

	/* Write value to Mag MEMS CRB_REG regsister */
	if ( LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &crb_regm) == false )
		goto done;

	/* Write value to Mag MEMS MR_REG regsister */
	if ( LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, &mr_regm) == false )
		goto done;

	magInitOK = true;

done:
	return magInitOK;
}

/**
  * @brief  Set LSM303DLHC Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_InitTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
static bool LSM303DLHC_AccInit(LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct)
{
	bool accInitOK = false;
	uint8_t ctrl1 = 0x00, ctrl4 = 0x00;

	/* Configure MEMS: data rate, power mode, full scale and axes */
	ctrl1 |= (uint8_t) (LSM303DLHC_InitStruct->Power_Mode | LSM303DLHC_InitStruct->AccOutput_DataRate | \
	                LSM303DLHC_InitStruct->Axes_Enable);

	ctrl4 |= (uint8_t) (LSM303DLHC_InitStruct->BlockData_Update | LSM303DLHC_InitStruct->Endianness | \
	                LSM303DLHC_InitStruct->AccFull_Scale|LSM303DLHC_InitStruct->High_Resolution);

	/* Write value to ACC MEMS CTRL_REG1 regsister */
	if ( LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1) == false )
		goto done;

	/* Write value to ACC MEMS CTRL_REG4 regsister */
	if ( LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4) == false )
		goto done;

	accInitOK = true;

done:
	return accInitOK;
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  LSM303DLHC_FilterStruct: pointer to a LSM303DLHC_FilterConfigTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
static bool LSM303DLHC_AccFilterConfig(LSM303DLHCAcc_FilterConfigTypeDef *LSM303DLHC_FilterStruct)
{
	bool lpfInitOk = false;
	uint8_t tmpreg;

	/* Read CTRL_REG2 register */
	if ( LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1) == false )
		goto done;

	tmpreg &= 0x0C;

	/* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
	tmpreg |= (uint8_t) (LSM303DLHC_FilterStruct->HighPassFilter_Mode_Selection |\
	                  LSM303DLHC_FilterStruct->HighPassFilter_CutOff_Frequency|\
	                  LSM303DLHC_FilterStruct->HighPassFilter_AOI1|\
	                  LSM303DLHC_FilterStruct->HighPassFilter_AOI2);

	/* Write value to ACC MEMS CTRL_REG2 regsister */
	if ( LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg) == false )
		goto done;

	lpfInitOk = true;

done:

	return lpfInitOk;
}


static bool initMag( void )
{
	/* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;

	LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
	LSM303DLHC_InitStructure.MagOutput_DataRate = LSM303DLHC_ODR_30_HZ ;
	LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_5_6_GA;
	LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOUS_CONVERSION;

	return LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);
}

static bool initAcc( void )
{
	/* Fill the accelerometer structure */
	LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
	LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
	LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
	LSM303DLHCAcc_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
	LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_8G;
	LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
	LSM303DLHCAcc_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
	LSM303DLHCAcc_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;

	/* Configure the accelerometer main parameters */
	return LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

}

static bool initAccLpf( void )
{
	/* Fill the accelerometer LPF structure */
	LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
	LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection = LSM303DLHC_HPM_NORMAL_MODE;
	LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_64;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

	/* Configure the accelerometer LPF main parameters */
	return LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

static bool LSM303DLHCConfig(void)
{
	bool lsm303dlhcInitOK = false;

	if ( initMag() == false )
		goto done;

	if ( initAcc() == false )
		goto done;

	if (initAccLpf() == false )
		goto done;

	lsm303dlhcInitOK = true;

done:

	return lsm303dlhcInitOK;
}

static bool readLSM303dlhcMagnetometer( void * pv, accelerometer_sensor_t * magnetometer_values )
{
	lsm303dlhcContext_st *lsmCtx = pv;
	uint8_t buffer[6];
	bool readMagnetometer = false;

	/* NB reads out in order XH, XL, ZH, ZL, YH, YL */
	if ( LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 6) == false )
		goto done;

	magnetometer_values[0] = (float)((int16_t)(((uint16_t)buffer[0] << 8) + buffer[1]))/lsmCtx->magnetometerXYSensitivity;
	magnetometer_values[2] = (float)((int16_t)(((uint16_t)buffer[2] << 8) + buffer[3]))/lsmCtx->magnetometerZSensitivity;
	magnetometer_values[1] = (float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5]))/lsmCtx->magnetometerXYSensitivity;

	readMagnetometer = true;

done:
	return readMagnetometer;
}

static bool readLSM303dlhcAccelerometer( void * pv, accelerometer_sensor_t * accelerometer_values )
{
	lsm303dlhcContext_st *lsmCtx = pv;
	uint8_t buffer[6];
	bool readAccelerometer = false;

	/* Read the register content */
	if ( LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6) == false )
		goto done;

	/* convert from raw to engineering units (g) */
	accelerometer_values[0]=(float)(((int16_t)((uint16_t)buffer[1] << 8) + buffer[0])>>4) * lsmCtx->accelerometerSensitivity;
	accelerometer_values[1]=(float)(((int16_t)((uint16_t)buffer[3] << 8) + buffer[2])>>4) * lsmCtx->accelerometerSensitivity;
	accelerometer_values[2]=(float)(((int16_t)((uint16_t)buffer[5] << 8) + buffer[4])>>4) * lsmCtx->accelerometerSensitivity;

	readAccelerometer = true;

done:
	return readAccelerometer;
}

void * initLSM303DLHC( sensorConfig_st *config, sensorCallback_st *callbacks )
{
	lsm303dlhcContext_st *lsmCtx = &lsm303dlhcCtx;

	/* given the I2C port we expect to find the chip connected to, try and initialise the lsm303dlhc */
	lsmCtx->i2cCtx = config->i2cCtx;
	lsmCtx->accelerometerSensitivity = LSM_Acc_Sensitivity_8g;	// TODO: configurable
	lsmCtx->magnetometerXYSensitivity = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
	lsmCtx->magnetometerZSensitivity = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;

	if ( LSM303DLHCConfig() == false )
		goto error;

	callbacks->readAccelerometer = readLSM303dlhcAccelerometer;
	callbacks->readMagnetometer = readLSM303dlhcMagnetometer;

	return lsmCtx;
error:
	return NULL;
}

