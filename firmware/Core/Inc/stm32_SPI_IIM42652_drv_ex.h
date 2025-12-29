/*
 * stm32IIM42652_drv_ex.h
 *
 *  Created on: Nov 21, 2021
 *      Author: kim yun sik
 */

#ifndef __IIM42652_DRV_EX_H
#define __IIM42652_DRV_EX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32_SPI_IIM42652_drv.h"

#define IIM_REG_VAL_SLEEP_MODE								(IIM_REG_PWR_MGMT0_GYRO_MODE_OFF \
															| IIM_REG_PWR_MGMT0_ACCEL_MODE_OFF)

#define IIM_REG_VAL_STANDBY_MODE							(IIM_REG_PWR_MGMT0_GYRO_MODE_STANDBY \
															| IIM_REG_PWR_MGMT0_ACCEL_MODE_OFF)

#define IIM_REG_VAL_ACCELLOWPOWER_MODE						(IIM_REG_PWR_MGMT0_GYRO_MODE_OFF \
															| IIM_REG_PWR_MGMT0_ACCEL_MODE_LOWPOWER)

#define IIM_REG_VAL_ACCELLOWNOISE_MODE						(IIM_REG_PWR_MGMT0_GYRO_MODE_OFF \
															| IIM_REG_PWR_MGMT0_ACCEL_MODE_LOWNOISE)

#define IIM_REG_VAL_GYROLOWNOISE_MODE						(IIM_REG_PWR_MGMT0_GYRO_MODE_LOWNOISE \
															|IIM_REG_PWR_MGMT0_ACCEL_MODE_OFF)

#define IIM_REG_VAL_ACCELGYROLOWNOISE_MODE					(IIM_REG_PWR_MGMT0_GYRO_MODE_LOWNOISE \
															|IIM_REG_PWR_MGMT0_ACCEL_MODE_LOWNOISE)


#define IIM_REG_REG_BANK_SEL_Pos							(0U)
#define IIM_REG_REG_BANK_SEL_Msk							(7U)
#define IIM_REG_REG_BANK_SEL_BANK0							(0U << IIM_REG_REG_BANK_SEL_Pos)
#define IIM_REG_REG_BANK_SEL_BANK1							(1U << IIM_REG_REG_BANK_SEL_Pos)
#define IIM_REG_REG_BANK_SEL_BANK2							(2U << IIM_REG_REG_BANK_SEL_Pos)
#define IIM_REG_REG_BANK_SEL_BANK3							(3U << IIM_REG_REG_BANK_SEL_Pos)
#define IIM_REG_REG_BANK_SEL_BANK4							(4U << IIM_REG_REG_BANK_SEL_Pos)



/* Define IMU Configuration Options */
/* Bank number option */
#define IIM_Bank0											(IIM_REG_REG_BANK_SEL_BANK0)
#define IIM_Bank1											(IIM_REG_REG_BANK_SEL_BANK1)
#define IIM_Bank2											(IIM_REG_REG_BANK_SEL_BANK2)
#define IIM_Bank3											(IIM_REG_REG_BANK_SEL_BANK3)
#define IIM_Bank4											(IIM_REG_REG_BANK_SEL_BANK4)

/* IMU operation mode option */
#define IIM_SleepMode										(IIM_REG_VAL_SLEEP_MODE)
#define IIM_StandbyMode										(IIM_REG_VAL_STANDBY_MODE)
#define IIM_AccelLowPowerMode								(IIM_REG_VAL_ACCELLOWPOWER_MODE)
#define IIM_AccelLowNoiseMode								(IIM_REG_VAL_ACCELLOWNOISE_MODE)
#define IIM_GyroLowNoiseMode								(IIM_REG_VAL_GYROLOWNOISE_MODE)
#define IIM_AccelGyroLowNoiseMode							(IIM_REG_VAL_ACCELGYROLOWNOISE_MODE)
#define IIM_DefualMode										(IIM_AccelGyroLowNoiseMode)

/* Temperature sensor enable option */
#define IIM_Temp_Disable									(IIM_REG_PWR_MGMT0_TEMP_DISABLE)
#define IIM_Temp_Enable										(IIM_REG_PWR_MGMT0_TEMP_ENABLE)

/* Acceleration Sensing scale option */
#define IIM_AccelFs_16G										(IIM_REG_ACCEL_CONFIG0_ACCEL_FS_SEL_16G)
#define IIM_AccelFs_8G										(IIM_REG_ACCEL_CONFIG0_ACCEL_FS_SEL_8G)
#define IIM_AccelFs_4G										(IIM_REG_ACCEL_CONFIG0_ACCEL_FS_SEL_4G)
#define IIM_AccelFs_2G										(IIM_REG_ACCEL_CONFIG0_ACCEL_FS_SEL_2G)
#define IIM_AccelFs_Default									(IIM_AccelFs_16G)

/* Acceleration Output data rate option */
#define IIM_AccelOdr_32kHz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_32kHz)
#define IIM_AccelOdr_16kHz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_16kHz)
#define IIM_AccelOdr_8kHz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_8kHz)
#define IIM_AccelOdr_4kHz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_4kHz)
#define IIM_AccelOdr_2kHz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_2kHz)
#define IIM_AccelOdr_1kHz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_1kHz)
#define IIM_AccelOdr_200Hz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_200Hz)
#define IIM_AccelOdr_100Hz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_100Hz)
#define IIM_AccelOdr_50Hz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_50Hz)
#define IIM_AccelOdr_25Hz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_25Hz)
#define IIM_AccelOdr_12_5Hz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_12_5Hz)
#define IIM_AccelOdr_6_25Hz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_6_25Hz)
#define IIM_AccelOdr_3_125Hz								(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_3_125Hz)
#define IIM_AccelOdr_1_5625Hz								(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_1_5625Hz)
#define IIM_AccelOdr_500Hz									(IIM_REG_ACCEL_CONFIG0_ACCEL_ODR_500Hz)
#define IIM_AccelOdr_Default								(IIM_AccelOdr_1kHz)

/* Gyroscope Sensing scale option */
#define IIM_GyroFs_2000dps									(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_2000dps)
#define IIM_GyroFs_1000dps									(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_1000dps)
#define IIM_GyroFs_500dps									(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_500dps)
#define IIM_GyroFs_250dps									(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_250dps)
#define IIM_GyroFs_125dps									(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_125dps)
#define IIM_GyroFs_62_5dps									(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_62_5dps)
#define IIM_GyroFs_31_25dps									(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_31_25dps)
#define IIM_GyroFs_15_625dps								(IIM_REG_GYRO_CONFIG0_GYRO_FS_SEL_15_625dps)
#define IIM_GyroFs_Default									(IIM_GyroFs_2000dps)

/* Gyroscope Output data rate option */
#define IIM_GyroOdr_32kHz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_32kHz)
#define IIM_GyroOdr_16kHz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_16kHz)
#define IIM_GyroOdr_8kHz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_8kHz)
#define IIM_GyroOdr_4kHz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_4kHz)
#define IIM_GyroOdr_2kHz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_2kHz)
#define IIM_GyroOdr_1kHz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_1kHz)
#define IIM_GyroOdr_200Hz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_200Hz)
#define IIM_GyroOdr_100Hz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_100Hz)
#define IIM_GyroOdr_50Hz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_50Hz)
#define IIM_GyroOdr_25Hz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_25Hz)
#define IIM_GyroOdr_12_5Hz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_12_5Hz)
#define IIM_GyroOdr_500Hz									(IIM_REG_GYRO_CONFIG0_GYRO_ODR_500Hz)
#define IIM_GyroOdr_Default									(IIM_GyroOdr_1kHz)

/* Timestamp setting option */
#define IIM_TMST_REGVAL_Enable						(IIM_REG_TMST_CONFIG_Reserved \
																					|IIM_REG_TMST_CONFIG_TMST_TO_REGS_EN_Enable \
																					|IIM_REG_TMST_CONFIG_TMST_DELTA_EN_Disable  \
																					|IIM_REG_TMST_CONFIG_TMST_FSYNC_EN_Disable  \
																					|IIM_REG_TMST_CONFIG_TMST_EN_Enable)

#define IIM_TMST_REGVAL_Disable						(IIM_REG_TMST_CONFIG_Reserved \
																					|IIM_REG_TMST_CONFIG_TMST_TO_REGS_EN_Disable \
																					|IIM_REG_TMST_CONFIG_TMST_DELTA_EN_Disable  \
																					|IIM_REG_TMST_CONFIG_TMST_FSYNC_EN_Disable  \
																					|IIM_REG_TMST_CONFIG_TMST_EN_Disable)

#define IIM_TMST_Res1us										(IIM_REG_TMST_CONFIG_TMST_RES_1us)
#define IIM_TMST_Res16us									(IIM_REG_TMST_CONFIG_TMST_RES_16us)

/* Interrupt setting option */
#define IIM_INT_Enable										(1U)
#define IIM_INT_Disable										(0U)

#define IIM_INT1_Mode_Pulsed								(IIM_REG_INT_CONFIG_INT1_MODE_Pulsed)
#define IIM_INT1_Mode_Latched								(IIM_REG_INT_CONFIG_INT1_MODE_Latched)
#define IIM_INT1_DriveCircuit_OpenDrain						(IIM_REG_INT_CONFIG_INT1_DRIVE_CIRCUIT_OpenDrain)
#define IIM_INT1_DriveCircuit_PushPull						(IIM_REG_INT_CONFIG_INT1_DRIVE_CIRCUIT_PushPull)

#define IIM_INT2_Mode_Pulsed								(IIM_REG_INT_CONFIG_INT2_MODE_Pulsed)
#define IIM_INT2_Mode_Latched								(IIM_REG_INT_CONFIG_INT2_MODE_Latched)
#define IIM_INT2_DriveCircuit_OpenDrain						(IIM_REG_INT_CONFIG_INT2_DRIVE_CIRCUIT_OpenDrain)
#define IIM_INT2_DriveCircuit_PushPull						(IIM_REG_INT_CONFIG_INT2_DRIVE_CIRCUIT_PushPull)

#define IIM_INT_ClearOnStatusRead							(IIM_REG_INT_CONFIG0_UI_DRDY_INT_CLEAR_ON_STATUS_READ \
															|IIM_REG_INT_CONFIG0_FIFO_THS_INT_CLEAR_ON_STATUS_READ \
															|IIM_REG_INT_CONFIG0_FIFO_FULL_INT_CLEAR_ON_STATUS_READ)

#define	IIM_INT_ClearOnDataRead								(IIM_REG_INT_CONFIG0_UI_DRDY_INT_CLEAR_ON_SENSOR_READ \
															|IIM_REG_INT_CONFIG0_FIFO_THS_INT_CLEAR_ON_FIFO_READ \
															|IIM_REG_INT_CONFIG0_FIFO_FULL_INT_CLEAR_ON_FIFO_READ )

#define IIM_INT_ClearOnBothRead								(IIM_REG_INT_CONFIG0_UI_DRDY_INT_CLEAR_ON_BOTH_READ\
															|IIM_REG_INT_CONFIG0_FIFO_THS_INT_CLEAR_ON_BOTH_READ\
															|IIM_REG_INT_CONFIG0_FIFO_FULL_INT_CLEAR_ON_BOTH_READ)

#define IIM_INT_SourceFsync									(IIM_REG_INT_SOURCE0_UI_FSYNC_INT1_EN)
#define IIM_INT_SourcePLLRDY								(IIM_REG_INT_SOURCE0_PLL_RDY_INT1_EN)
#define IIM_INT_SourceReset									(IIM_REG_INT_SOURCE0_RESET_DONE_INT1_EN)
#define IIM_INT_SourceDRDY									(IIM_REG_INT_SOURCE0_UI_DRDY_INT1_EN)
#define IIM_INT_SourceFIFOThs								(IIM_REG_INT_SOURCE0_FIFO_THS_INT1_EN)
#define IIM_INT_SourceFIFOFull								(IIM_REG_INT_SOURCE0_FIFO_FULL_INT1_EN)
#define IIM_INT_SourceFIFO									(IIM_REG_INT_SOURCE0_UI_AGC_RDY_INT1_EN)


/* Interrupt type */
#define IIM_INT_UIFSYNC										(IIM_REG_INT_STATUS_UI_FSYNC_INT_Msk)
#define IIM_INT_PLLReady									(IIM_REG_INT_STATUS_PLL_RDY_INT_Msk)
#define IIM_INT_ResetDone									(IIM_REG_INT_STATUS_RESET_DONE_INT_Msk)
#define IIM_INT_DataReady									(IIM_REG_INT_STATUS_DATA_RDY_INT_Msk)
#define IIM_INT_FifoThreshold								(IIM_REG_INT_STATUS_FIFO_THS_INT_Msk)
#define IIM_INT_FifoFull									(IIM_REG_INT_STATUS_FIFO_FULL_INT_Msk)
#define IIM_INT_AGCReady									(IIM_REG_INT_STATUS_AGC_RDY_INT_Msk)

/* FIFO mode option */
#define IIM_FIFO_BypassMode									(IIM_REG_FIFO_CONFIG_FIFO_MODE_Bypass)
#define IIM_FIFO_StreamtoFIFOMode							(IIM_REG_FIFO_CONFIG_FIFO_MODE_StreamtoFIFO)
#define IIM_FIFO_StoponFullMode								(IIM_REG_FIFO_CONFIG_FIFO_MODE_StoponFull)

#define IIM_FIFO_TEMPEnable									(IIM_REG_FIFO_CONFIG1_FIFO_TEMP_EN)
#define IIM_FIFO_ACCELEnable								(IIM_REG_FIFO_CONFIG1_FIFO_ACCEL_EN)
#define IIM_FIFO_GYROEnable									(IIM_REG_FIFO_CONFIG1_FIFO_GYRO_EN)
#define IIM_FIFO_THRSEnable									(IIM_REG_FIFO_CONFIG1_FIFO_WM_GT_TH)

#define Pckt_8byte							(8U)
#define Pckt_16byte							(16U)
#define Cal_Pckt_Size(DataEnable)			((((DataEnable & IIM_FIFO_ACCELEnable) != 0) && ((DataEnable & IIM_FIFO_GYROEnable) != 0)) ? (Pckt_16byte):(Pckt_8byte))



typedef enum {
	IIM_Success = 0,
	IIM_ResetFail = 1,
	IIM_InitFail = 2,
	IIM_MemoryFail = 3,
	I2C_BUSY = 4,
	I2C_Error = 5,
	DMA_PARAM_Error = 6,
	DMA_Error = 7,
} IIMreturnType;

typedef enum {
	IIM_OK,
	IIM_Rst,
	IIM_Ready,
	IIM_ConfigOnGoing
}IIMstatus;


typedef struct {
	uint8_t	Fs;
	uint8_t ODR;

}AccelConfig;

typedef struct {
	uint8_t	Fs;
	uint8_t ODR;

}GyroConfig;

typedef struct {
	uint8_t TmstEnable;
	uint8_t TmstRes;

}TmstConfig;

typedef struct {

	uint8_t Int1Enable;
	uint8_t Int1Mode;
	uint8_t Int1DrvCir;
	uint8_t Int1Source;

	uint8_t Int2Enable;
	uint8_t Int2Mode;
	uint8_t Int2DrvCir;
	uint8_t Int2Source;

	uint8_t ClearOpt;


}IntConfig;

typedef struct {
	uint8_t  FifoMode;
	uint8_t  FifoEnable;
	uint16_t FifoThresPckNb;

}FifoConfig;

typedef struct {
	IIMstatus status;

	uint8_t mode;

	uint8_t tempEnable;

	uint8_t bank;

	AccelConfig aCfg;

	GyroConfig gCfg;

	TmstConfig tCfg;

	FifoConfig fCfg;

	IntConfig iCfg;


}IIMConfigDef;


typedef struct {


}IIMDataType;

typedef struct {
	IIMDataType 	IMData;
	IIMConfigDef 	IMCfg;

}IIM_HandleTypeDef;




//IIMreturnType IIM_ParseData(I2C_HandleTypeDef *hi2c, IIMConfigDef *IIMCfg, IIMDataType *data);
//IIMreturnType IIM_ParseFifoData(I2C_HandleTypeDef *hi2c,  IIM_HandleTypeDef *hiim);
void IIM_EnableIT();
void IIM_DisableIT();



#ifdef __cplusplus
}
#endif

#endif /* __IIM42652_DRV_EX_H */
