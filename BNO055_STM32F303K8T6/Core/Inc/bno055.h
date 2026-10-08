/*
 * bno055.h
 *
 *  Created on: May 17, 2026
 *      Author: raj_2806
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "stm32f3xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

#define BNO055_CHIP_ID_VALUE    0xA0

#define BNO055_ADDRESS         (0x28 <<1)

#define BNO055_CHIP_ID          0x00
#define BNO055_ACC_ID           0x01
#define BNO055_MAG_ID           0x02
#define BNO055_GYRO_ID          0x03
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06

#define BNO055_PAGE_ID          0x07

// BNO055 Page 0
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A

// BNO055 Page 1
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F

#define BNO055_QUAT_SCALE       (1.0f / 16384.0f)

#define BNO055_EUL_SCALE        (1.0f / 16.0f)

/** BNO-55 operation modes **/
typedef enum {
  BNO055_OPERATION_MODE_CONFIG = 0,
  // Sensor Mode
  BNO055_OPERATION_MODE_ACCONLY ,
  BNO055_OPERATION_MODE_MAGONLY ,
  BNO055_OPERATION_MODE_GYRONLY ,
  BNO055_OPERATION_MODE_ACCMAG ,
  BNO055_OPERATION_MODE_ACCGYRO ,
  BNO055_OPERATION_MODE_MAGGYRO ,
  BNO055_OPERATION_MODE_AMG ,  // 0x07
                              // Fusion Mode
  BNO055_OPERATION_MODE_IMU ,
  BNO055_OPERATION_MODE_COMPASS ,
  BNO055_OPERATION_MODE_M4G ,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF ,
  BNO055_OPERATION_MODE_NDOF  // 0x0C
} OPRMode_t;

typedef enum  {
NormalPwr = 0,
LowPwr = 1,
SuspendPwr = 2
}PWRMode_t;

/** Remap settings **/
 typedef enum {
   REMAP_CONFIG_P0 = 0x21,
   REMAP_CONFIG_P1 = 0x24, // default
   REMAP_CONFIG_P2 = 0x24,
   REMAP_CONFIG_P3 = 0x21,
   REMAP_CONFIG_P4 = 0x24,
   REMAP_CONFIG_P5 = 0x21,
   REMAP_CONFIG_P6 = 0x21,
   REMAP_CONFIG_P7 = 0x24
 }axis_remap_config_t;

/** Remap Signs **/
   typedef enum {
     REMAP_SIGN_P0 = 0x04,
     REMAP_SIGN_P1 = 0x00, // default
     REMAP_SIGN_P2 = 0x06,
     REMAP_SIGN_P3 = 0x02,
     REMAP_SIGN_P4 = 0x03,
     REMAP_SIGN_P5 = 0x01,
     REMAP_SIGN_P6 = 0x07,
     REMAP_SIGN_P7 = 0x05
   }axis_remap_sign_t;

   enum Accg_range {
   R_2G = 0,
   R_4G,
   R_8G,
   R_16G
   };

   enum Acc_BandWidth{
   B_7hz = 0,
   B_15hz,
   B_31hz,
   B_62hz,
   B_125hz,
   B_250hz,
   B_500hz,
   B_1000hz
   };

   enum Acc_opr{
   acc_normal = 0,
   acc_suspend,
   acc_lowpower1,
   acc_standby,
   acc_lowpoer2,
   acc_deepsuspend
   };

   enum Gyro_range{
   R_2000,
   R_1000,
   R_500,
   R_250,
   R_125
   };

   enum Gyro_BandWidth{
   B_523hz,
   B_230hz,
   B_116hz,
   B_47hz,
   B_23hz,
   B_12hz,
   B_64hz,
   B_32hz
   };

   enum Gyro_opr{
   Gyro_normal,
   Gyro_FastPower,
   Gyro_DeepSuspend,
   Gyro_Suspend,
   Gyro_AdvancedPowerSaved,
   };


   enum DataOutputRate {
   MB_2hz,
   MB_6hz,
   MB_8hz,
   MB_10hz,
   MB_15hz,
   MB_20hz,
   MB_25hz,
   MB_30hz
   };

   enum Mag_opr{
   Mag_LowPwr,
   Mag_Regular,
   Mag_EnhancedRegular,
   Mag_HighAccuracy
   };

   enum Mag_pwrMode{
   Mag_Normal,
   Mag_Sleep,
   Mag_Suspend,
   Mag_ForceMode
   };

   HAL_StatusTypeDef bno055_Read(uint8_t address, uint8_t *buffer);
   HAL_StatusTypeDef bno055_ReadByte(uint8_t address, uint8_t *buffer, uint8_t size);
   HAL_StatusTypeDef bno055_Write(uint8_t address, uint8_t data);
   HAL_StatusTypeDef bno055_WriteByte(uint8_t address, uint8_t *data, uint8_t size);

   void serialWrite(char *data);

   void bno055_Reset(void);
   void bno055_Scan(void);
   void bno055_Init(void);
   void bno055_Post(void);
   void bno055_SetOprMode(uint8_t mode);

   uint8_t bno055_CheckSysStatus(void);
   void bno055_Calibration_Status(uint8_t *status, uint32_t timeout_ms);

   void bno055_Acc_Calibration(void);
   void bno055_Gyro_Calibration(void);
   void bno055_Mag_Calibration(void);

   void bno055_Offset_Read(uint8_t *data, uint8_t size, uint8_t previousmode) ;
   void bno055_Offset_Write(uint8_t *data, uint8_t size, uint8_t previousmode);

   void bno055_ReadAccData(int16_t *buffer);
   void bno055_ReadGyroData(int16_t *buffer);
   void bno055_ReadMagData(int16_t *buffer);
   void bno055_ReadEulData(int16_t *buffer);
   void bno055_ReadQuatData(float *buffer);

   float bno055_QuatToHeading(float *quat);
   uint8_t bno055_QuatIsValid(float *quat);

   void bno055_Low_Pass_Filter_3_axis_raw(int16_t *filtered, int16_t *raw, float alpha);
   void bno055_Low_Pass_Filter_3_axis_float(float *filtered, float *raw, float alpha);
   void bno055_High_Pass_Filter_3_axis_raw(int16_t *filtered, int16_t *raw, int16_t *prev_raw, float beta);
   void  bno055_QuatSlerp(float *qa, float *qb, float t, float *out);
   float bno055_HeadingLPF(float new_heading, float prev_filtered, float alpha);
   uint8_t bno055_HeadingJumpDetect(float new_heading, float prev_heading, float threshold_deg);

#endif
