#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <coos.h>
#include <i2c.h>
#include <sensors.h>

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 hardware
#define MPU6050_ADDRESS         0x68

#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6050_SMPLRT_DIV      0       // 8000Hz

enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

typedef struct MPU6050Context_st
{
	void *i2cCtx;
	float accelerometerSensitivity;		/* LSB/g */
	float gyroSensitivity;				/* LSB/dps */
	float zero_bias[3];					/* zero bias for the gyro */
} MPU6050Context_st;

static MPU6050Context_st MPU6050Ctx;

static uint8_t mpuLowPassFilter = INV_FILTER_42HZ;

typedef enum {
    MPU_6050_HALF_RESOLUTION,
    MPU_6050_FULL_RESOLUTION
} mpu6050Resolution_e;

static mpu6050Resolution_e mpuAccelTrim;

static bool mpu6050Detect(MPU6050Context_st * MPUCtx)
{
    bool ack;
    uint8_t sig;

    delay(35);          // datasheet page 13 says 30ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cRead(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_WHO_AM_I, &sig, 1);
    if (!ack)
        return false;

    // So like, MPU6xxx has a "WHO_AM_I" register, that is used to verify the identity of the device.
    // The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0's 7-bit I2C address.
    // The least significant bit of the MPU-60X0's I2C address is determined by the value of the AD0 pin. (we know that already).
    // But here's the best part: The value of the AD0 pin is not reflected in this register.

    if (sig != (MPU6050_ADDRESS & 0x7e))
        return false;

    return true;
}

bool mpu6050AccDetect( MPU6050Context_st * MPUCtx )
{
    uint8_t readBuffer[6];
    uint8_t revision;
    uint8_t productId;

    if (!mpu6050Detect(MPUCtx))
    {
        return false;
    }

    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and accel revision
    i2cRead(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_XA_OFFS_H, readBuffer, 6);
    revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (revision)
    {
        /* Congrats, these parts are better. */
        if (revision == 1)
        {
            mpuAccelTrim = MPU_6050_HALF_RESOLUTION;
        } else if (revision == 2)
        {
            mpuAccelTrim = MPU_6050_FULL_RESOLUTION;
        } else
        {
        	return false;
        }
    } else {
        i2cRead(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_PRODUCT_ID, &productId, 1);
        revision = productId & 0x0F;
        if (!revision)
        {
        	return false;
        } else if (revision == 4)
        {
            mpuAccelTrim = MPU_6050_HALF_RESOLUTION;
        } else
        {
            mpuAccelTrim = MPU_6050_FULL_RESOLUTION;
        }
    }

    return true;
}

bool mpu6050GyroDetect(uint16_t lpf)
{

    if (lpf >= 188)
        mpuLowPassFilter = INV_FILTER_188HZ;
    else if (lpf >= 98)
        mpuLowPassFilter = INV_FILTER_98HZ;
    else if (lpf >= 42)
        mpuLowPassFilter = INV_FILTER_42HZ;
    else if (lpf >= 20)
        mpuLowPassFilter = INV_FILTER_20HZ;
    else if (lpf >= 10)
        mpuLowPassFilter = INV_FILTER_10HZ;
    else
        mpuLowPassFilter = INV_FILTER_5HZ;

    return true;
}

static bool readMPU6050Accelerometer( void * pv, accelerometer_sensor_t * accelerometer_values )
{
	MPU6050Context_st * MPUCtx = pv;
	uint8_t buffer[6];
	bool readAccelerometer = false;

	/* Read the register content */
    if ( i2cRead(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, buffer, 6) == false )
    	goto done;

	/* convert from raw to engineering units (g) */
	accelerometer_values[0]=(float)(((int16_t)((uint16_t)buffer[0] << 8) + buffer[1])) / MPUCtx->accelerometerSensitivity;
	accelerometer_values[1]=(float)(((int16_t)((uint16_t)buffer[2] << 8) + buffer[3])) / MPUCtx->accelerometerSensitivity;
	accelerometer_values[2]=(float)(((int16_t)((uint16_t)buffer[4] << 8) + buffer[5])) / MPUCtx->accelerometerSensitivity;

	readAccelerometer = true;

done:
	return readAccelerometer;
}


static bool mpu6050Init(MPU6050Context_st * MPUCtx)
{
	bool result;

    if ( i2cWrite(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x80) == false )      //PWR_MGMT_1    -- DEVICE_RESET 1
    	goto done;
    delay(100);
    if ( i2cWrite(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, 0x00) == false ) //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    	goto done;
    if ( i2cWrite(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03) == false ) //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    	goto done;
    if ( i2cWrite(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0) == false )  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    	goto done;
    if ( i2cWrite(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_CONFIG, mpuLowPassFilter) == false ) //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    	goto done;
    if ( i2cWrite(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3) == false )   //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
    	goto done;

    // ACC Init stuff. Moved into gyro init because the reset above would screw up accel config. Oops.
    // Accel scale 8g (4096 LSB/g)
    if ( i2cWrite(MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3) == false )
    	goto done;

	result = true;

done:
    return result;
}

static bool mpu6050GyroReadRaw(MPU6050Context_st * MPUCtx, int16_t *gyroData)
{
	bool result = false;
    uint8_t buf[6];

    if ( i2cRead( MPUCtx->i2cCtx, MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_H, buf, 6) == false )
    	goto done;

    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]);

	result = true;

done:
	return result;
}

static bool MPU6050ReadGyro( void * pv, gyrometer_sensor_t *gyrometer )
{
	MPU6050Context_st * MPUCtx = pv;
	bool readGyro = false;
    int16_t buf[3];

	if ( mpu6050GyroReadRaw( MPUCtx, buf ) == false )
		goto done;

    gyrometer[0] = (float)(buf[0] - MPUCtx->zero_bias[0]) / MPUCtx->gyroSensitivity;
    gyrometer[1] = (float)(buf[1] - MPUCtx->zero_bias[1]) / MPUCtx->gyroSensitivity;
    gyrometer[2] = (float)(buf[2] - MPUCtx->zero_bias[2]) / MPUCtx->gyroSensitivity;

	readGyro = true;
done:
	return readGyro;
}

#define NB_GYRO_CAL_SAMPLES			100
static bool calculateMPUGyroZeroBias( MPU6050Context_st * MPUCtx )
{
	bool doneZeroBias = false;
	int i;
	int32_t largebuf[3] = {0,0,0};
	int16_t buf[3];

	for (i = 0; i < NB_GYRO_CAL_SAMPLES; i++ )
	{
		CoTimeDelay( 0, 0, 0, 10 );

		if ( mpu6050GyroReadRaw( MPUCtx, buf ) == false )
			goto done;
		/* any shaking of the board resets things */
		if ( fabs((float)buf[0] / MPUCtx->gyroSensitivity) > 10.0f )
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

	MPUCtx->zero_bias[0] = (largebuf[0]+i/2)/i;
	MPUCtx->zero_bias[1] = (largebuf[1]+i/2)/i;
	MPUCtx->zero_bias[2] = (largebuf[2]+i/2)/i;

	doneZeroBias = true;

done:
	return doneZeroBias;
}


void initMPU6050( sensorConfig_st *config, sensorCallback_st *callbacks )
{
	MPU6050Context_st *MPUCtx = &MPU6050Ctx;

	/* given the I2C port we expect to find the chip connected to, try and initialise the MPU6050 */
	MPUCtx->i2cCtx = config->i2cCtx;
	MPUCtx->accelerometerSensitivity = 4096.0f;	// TODO: configurable
	/* divide by 16.4 to give +-2000dps full range */
	MPUCtx->gyroSensitivity = 16.4f;	// TODO: configurable

	if ( mpu6050AccDetect( MPUCtx ) == false )
		goto error;

	if ( mpu6050GyroDetect( 42 ) == false )
		goto error;

	if ( mpu6050Init( MPUCtx ) == false )
		goto error;

	if ( calculateMPUGyroZeroBias( MPUCtx ) == false )
		goto error;

	callbacks->readAccelerometer = readMPU6050Accelerometer;
	callbacks->accelerometerCtx = MPUCtx;
	callbacks->readGyro = MPU6050ReadGyro;
	callbacks->gyroCtx = MPUCtx;

error:
	return;
}

