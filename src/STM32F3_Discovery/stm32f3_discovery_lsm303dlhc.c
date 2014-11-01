/**
  ******************************************************************************
  * @file    stm32f3_discovery_lsm303dlhc.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   This file provides a set of functions needed to manage the lsm303dlhc
  *          MEMS accelerometer available on STM32F3-Discovery Kit.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stm32f30x_syscfg.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_i2c.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_exti.h>
#include "stm32f3_discovery_lsm303dlhc.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY_LSM303DLHC
  * @{
  */


/** @defgroup STM32F3_DISCOVERY_LSM303DLHC_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_LSM303DLHC_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_LSM303DLHC_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_LSM303DLHC_Private_Variables
  * @{
  */
__IO uint32_t  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_LSM303DLHC_Private_Functions
  * @{
  */

/**
  * @brief  Set LSM303DLHC Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_InitTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void LSM303DLHC_AccInit(LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct)
{
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;

	/* Assumes that the I2C driver has been Initialised */

  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (LSM303DLHC_InitStruct->Power_Mode | LSM303DLHC_InitStruct->AccOutput_DataRate | \
                    LSM303DLHC_InitStruct->Axes_Enable);

  ctrl4 |= (uint8_t) (LSM303DLHC_InitStruct->BlockData_Update | LSM303DLHC_InitStruct->Endianness | \
                    LSM303DLHC_InitStruct->AccFull_Scale|LSM303DLHC_InitStruct->High_Resolution);

  /* Write value to ACC MEMS CTRL_REG1 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1);

  /* Write value to ACC MEMS CTRL_REG4 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4);
}

/**
  * @brief  Reboot memory content of LSM303DLHC
  * @param  None
  * @retval None
  */
void LSM303DLHC_AccRebootCmd(void)
{
  uint8_t tmpreg;

  /* Read CTRL_REG5 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &tmpreg, 1);

  /* Enable or Disable the reboot memory */
  tmpreg |= LSM303DLHC_BOOT_REBOOTMEMORY;

  /* Write value to ACC MEMS CTRL_REG5 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &tmpreg);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  LSM303DLHC_FilterStruct: pointer to a LSM303DLHC_FilterConfigTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void LSM303DLHC_AccFilterConfig(LSM303DLHCAcc_FilterConfigTypeDef *LSM303DLHC_FilterStruct)
{
  uint8_t tmpreg;

  /* Read CTRL_REG2 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);

  tmpreg &= 0x0C;

  /* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
  tmpreg |= (uint8_t) (LSM303DLHC_FilterStruct->HighPassFilter_Mode_Selection |\
                      LSM303DLHC_FilterStruct->HighPassFilter_CutOff_Frequency|\
                      LSM303DLHC_FilterStruct->HighPassFilter_AOI1|\
                      LSM303DLHC_FilterStruct->HighPassFilter_AOI2);

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be:
  *         @arg: LSM303DLHC_HighPassFilter_DISABLE
  *         @arg: LSM303DLHC_HighPassFilter_ENABLE
  * @retval None
  */
void LSM303DLHC_AccFilterCmd(uint8_t HighPassFilterState)
 {
  uint8_t tmpreg;

  /* Read CTRL_REG2 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);

  tmpreg &= 0xF7;

  tmpreg |= HighPassFilterState;

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
}

/**
  * @brief  Enable or Disable High Pass Filter on CLick
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be:
  *         @arg: LSM303DLHC_HighPassFilter_DISABLE
  *         @arg: LSM303DLHC_HighPassFilter_ENABLE
  * @retval None
  */
void LSM303DLHC_AccFilterClickCmd(uint8_t HighPassFilterClickState)
 {
  uint8_t tmpreg;

  /* Read CTRL_REG2 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);

  tmpreg &= 0xFB;

  tmpreg |= HighPassFilterClickState;

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
}

/**
  * @brief Set LSM303DLHC Interrupt1 configuration
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
  *           This parameter can be any combination of the following values:
  *         @arg   LSM303DLHC_IT1_CLICK
  *         @arg   LSM303DLHC_IT1_AOI1
  *         @arg   LSM303DLHC_IT1_AOI2
  *         @arg   LSM303DLHC_IT1_DRY1
  *         @arg   LSM303DLHC_IT1_WTM
  *         @arg   LSM303DLHC_IT1_OVERRUN
  * @param  NewState: new state of the selected LSM303DLHC interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void LSM303DLHC_AccIT1Config(uint8_t LSM303DLHC_IT, FunctionalState NewState)
{
  uint8_t tmpval = 0x00;

  /* Read CTRL_REG3 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval, 1 );

  tmpval &= ~LSM303DLHC_IT;

  if (NewState != DISABLE)
  {
    tmpval |= LSM303DLHC_IT;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =~ LSM303DLHC_IT;
  }

  /* Write value to MEMS CTRL_REG3 register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval);
}

/**
  * @brief Set LSM303DLHC Interrupt2 configuration
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
  *           This parameter can be any combination of the following values:
  *         @arg   LSM303DLHC_IT2_CLICK2
  *         @arg   LSM303DLHC_IT2_INT1
  *         @arg   LSM303DLHC_IT2_INT2
  *         @arg   LSM303DLHC_IT2_BOOT
  *         @arg   LSM303DLHC_IT2_ACT
  *         @arg   LSM303DLHC_IT2_HLACTIVE
  * @param  NewState: new state of the selected LSM303DLHC interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void LSM303DLHC_AccIT2Config(uint8_t LSM303DLHC_IT, FunctionalState NewState)
{
  uint8_t tmpval = 0x00;

  /* Read CTRL_REG3 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval, 1);

  tmpval &= ~LSM303DLHC_IT;

  if (NewState != DISABLE)
  {
    tmpval |= LSM303DLHC_IT;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =~ LSM303DLHC_IT;
  }

  /* Write value to MEMS CTRL_REG3 register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval);
}

/**
  * @brief  INT1 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled
  *         NewState: Enable or Disable
  * @retval None
  */
void LSM303DLHC_AccINT1InterruptConfig(uint8_t ITCombination, uint8_t ITAxes, FunctionalState NewState )
{
  uint8_t tmpval = ITCombination;

  /* Read INT1_CFR register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval, 1);

  if (NewState != DISABLE)
  {
    tmpval |= ITAxes;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =(~ITAxes)|ITCombination;
  }

  /* Write value to MEMS INT1_CFR register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval);
}

/**
  * @brief  INT1 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled
  *         NewState: Enable or Disable
  * @retval None
  */
void LSM303DLHC_AccINT2InterruptConfig(uint8_t ITCombination, uint8_t ITAxes, FunctionalState NewState )
{
  uint8_t tmpval = ITCombination;

  /* Read INT2_CFR register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval, 1);

  if (NewState != DISABLE)
  {
    tmpval |= ITAxes;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =(~ITAxes)|ITCombination;
  }

  /* Write value to MEMS INT2_CFR register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval);
}

/**
  * @brief  INT1 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled
  *         NewState: Enable or Disable
  * @retval None
  */
void LSM303DLHC_AccClickITConfig(uint8_t ITClick, FunctionalState NewState)
{
  uint8_t tmpval;

  /* Read CLICK_CFR register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval, 1);

  if (NewState != DISABLE)
  {
    tmpval |= ITClick;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =~ITClick;
  }

  /* Write value to MEMS CLICK_CFR register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval);
}

/**
  * @brief  Get status for Acc LSM303DLHC data
  * @param  None
  * @retval Data status in a LSM303DLHC Data register
  */
uint8_t LSM303DLHC_AccGetDataStatus(void)
{
  uint8_t tmpreg;

  /* Read Mag STATUS register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_STATUS_REG_A, &tmpreg, 1);

  return tmpreg;
}

/**
  * @brief  Set LSM303DLHC Mag Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_MagInitTypeDef structure
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void LSM303DLHC_MagInit(LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct)
{
  uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;

	/* Assumes that the I2C driver has been Initialised */

  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (LSM303DLHC_InitStruct->Temperature_Sensor | LSM303DLHC_InitStruct->MagOutput_DataRate);

  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (LSM303DLHC_InitStruct->MagFull_Scale);

  /* Configure MEMS: working mode */
  mr_regm |= (uint8_t) (LSM303DLHC_InitStruct->Working_Mode);

  /* Write value to Mag MEMS CRA_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, &cra_regm);

  /* Write value to Mag MEMS CRB_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &crb_regm);

  /* Write value to Mag MEMS MR_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, &mr_regm);
}

/**
  * @brief  Get status for Mag LSM303DLHC data
  * @param  None
  * @retval Data status in a LSM303DLHC Data register
  */
uint8_t LSM303DLHC_MagGetDataStatus(void)
{
  uint8_t tmpreg;

  /* Read Mag STATUS register */
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_SR_REG_M, &tmpreg, 1 );

  return tmpreg;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
