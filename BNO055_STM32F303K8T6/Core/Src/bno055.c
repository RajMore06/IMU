/*
 * bno055.c
 *
 *  Created on: May 17, 2026
 *      Author: raj_2806
 */

#include "bno055.h"
#include "string.h"
#include "stdio.h"
#include "main.h"

extern char debug[64];

void serialWrite(char *data) {
	HAL_StatusTypeDef result;
	result = HAL_UART_Transmit(&huart2, (uint8_t*) data, strlen(data), HAL_MAX_DELAY);
	if (result != HAL_OK) {
		Error_Handler();
	}
}

void bno055_Scan(void)
{
    serialWrite("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 10);
        if (result == HAL_OK) {
            uint8_t chip_id = 0;
            HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), BNO055_CHIP_ID, 1, &chip_id, 1, 10);
            if (chip_id == BNO055_CHIP_ID_VALUE) {
                snprintf(debug, 64, "BNO055 confirmed at 0x%02X (chip_id=0x%02X)\n", addr, chip_id);
                serialWrite(debug);
                return;
            }
            else {
                snprintf(debug, 64, "Non-BNO055 device at 0x%02X (chip_id=0x%02X), skipping\n", addr, chip_id);
                serialWrite(debug);
            }
        }
    }
    serialWrite("ERROR: BNO055 not found on I2C bus!\n");
}

float bno055_QuatToHeading(float *quat) {
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    float sin_yaw = 2.0f * (w * z + x * y);
    float cos_yaw = 1.0f - 2.0f * (y * y + z * z);
    float heading_rad = -1*atan2f(sin_yaw, cos_yaw);
    float heading_deg = heading_rad * (180.0f / (float)M_PI);
    /* wrap to [0, 360) — fmodf handles both positive and negative inputs */
    heading_deg = fmodf(heading_deg + 360.0f, 360.0f);
    return heading_deg;
}

uint8_t bno055_CheckSysStatus(void) {
    uint8_t sys_status, sys_err;
    bno055_Read(BNO055_SYS_STATUS, &sys_status);
    bno055_Read(BNO055_SYS_ERR, &sys_err);
    snprintf(debug, 64, "SYS_STATUS=0x%02X SYS_ERR=0x%02X\n", sys_status, sys_err);
    serialWrite(debug);
    /* 0x05 = fusion running normally; anything else is an error state */
    return (sys_status == 0x05 && sys_err == 0x00) ? 1 : 0;
}

void bno055_Reset(void) {
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//    HAL_Delay(20);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
//    HAL_Delay(800);

	uint8_t comm = 0x00 | 0x20;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS,BNO055_SYS_TRIGGER, I2C_MEMADD_SIZE_8BIT, &comm, 1, 1000);
//	if (i2cstatus != HAL_OK) {
//		SerialWrite("fail! ");
//	}
	HAL_Delay(800);
}

void bno055_Init(void)
{
    serialWrite("bno055_Init: setting page 0...\n");
    if (bno055_Write(BNO055_PAGE_ID, 0x00) != HAL_OK)
    {
        serialWrite("ERROR: bno055_Init failed at PAGE_ID write.\n");
        return;
    }
    HAL_Delay(25);

    serialWrite("bno055_Init: setting CONFIG mode...\n");
    bno055_SetOprMode(BNO055_OPERATION_MODE_CONFIG);
    HAL_Delay(25);

    serialWrite("bno055_Init: setting Normal power mode...\n");
    if (bno055_Write(BNO055_PWR_MODE, (uint8_t)NormalPwr) != HAL_OK)
    {
        serialWrite("ERROR: bno055_Init failed at PWR_MODE write.\n");
        return;
    }
    HAL_Delay(25);

    serialWrite("bno055_Init: complete.\n");
}

HAL_StatusTypeDef bno055_Write(uint8_t address, uint8_t data)
{
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, address, 1, &data, 1, HAL_MAX_DELAY);

    if (status != HAL_OK)
    {
        snprintf(debug, 64, "I2C WRITE ERR reg=0x%02X status=%d\n", address, (int)status);
        serialWrite(debug);
    }
    return status;
}

HAL_StatusTypeDef bno055_WriteByte(uint8_t address, uint8_t *data, uint8_t size)
{
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, address, 1, data, size, HAL_MAX_DELAY);

    if (status != HAL_OK)
    {
        snprintf(debug, 64, "I2C WRITE_N ERR reg=0x%02X sz=%u status=%d\n",
                 address, (unsigned)size, (int)status);
        serialWrite(debug);
    }
    return status;
}

HAL_StatusTypeDef bno055_Read(uint8_t address, uint8_t *buffer)
{
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDRESS, address, 1, buffer, 1, HAL_MAX_DELAY);

    if (status != HAL_OK)
    {
        snprintf(debug, 64, "I2C READ ERR reg=0x%02X status=%d\n", address, (int)status);
        serialWrite(debug);
    }
    return status;
}

HAL_StatusTypeDef bno055_ReadByte(uint8_t address, uint8_t *buffer, uint8_t size)
{
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDRESS, address, 1, buffer, size, HAL_MAX_DELAY);

    if (status != HAL_OK)
    {
        snprintf(debug, 64, "I2C READ_N ERR reg=0x%02X sz=%u status=%d\n",
                 address, (unsigned)size, (int)status);
        serialWrite(debug);
    }
    return status;
}

void bno055_SetOprMode(uint8_t mode) {
	bno055_Write(BNO055_OPR_MODE, mode);
	HAL_Delay(25);
}

void bno055_ReadEulData(int16_t *buffer) {
	uint8_t data[6];
	bno055_ReadByte(BNO055_EUL_HEADING_LSB, data, 6);
	buffer[0] = (int16_t) ((uint16_t)data[1] << 8 | data[0]);
	buffer[1] = (int16_t) ((uint16_t)data[3] << 8 | data[2]);
	buffer[2] = (int16_t) ((uint16_t)data[5] << 8 | data[4]);
}

void bno055_ReadMagData(int16_t *buffer) {
	uint8_t data[6];
	bno055_ReadByte(BNO055_MAG_DATA_X_LSB, data, 6);
	buffer[0] = (int16_t) ((uint16_t)data[1] << 8 | data[0]);
	buffer[1] = (int16_t) ((uint16_t)data[3] << 8 | data[2]);
	buffer[2] = (int16_t) ((uint16_t)data[5] << 8 | data[4]);
}

void bno055_ReadAccData(int16_t *buffer) {
	uint8_t data[6];
	bno055_ReadByte(BNO055_ACC_DATA_X_LSB, data, 6);
	buffer[0] = (int16_t) ((uint16_t)data[1] << 8 | data[0]);
	buffer[1] = (int16_t) ((uint16_t)data[3] << 8 | data[2]);
	buffer[2] = (int16_t) ((uint16_t)data[5] << 8 | data[4]);
}

void bno055_ReadGyroData(int16_t *buffer) {
	uint8_t data[6];
	bno055_ReadByte(BNO055_GYR_DATA_X_LSB, data, 6);
	buffer[0] = (int16_t) ((uint16_t)data[1] << 8 | data[0]);
	buffer[1] = (int16_t) ((uint16_t)data[3] << 8 | data[2]);
	buffer[2] = (int16_t) ((uint16_t)data[5] << 8 | data[4]);
}

uint8_t bno055_QuatIsValid(float *quat)
{
    /* All-zero → fusion engine has not started yet */
    if (quat[0] == 0.0f && quat[1] == 0.0f &&
        quat[2] == 0.0f && quat[3] == 0.0f)
    {
        return 0;
    }

    float norm_sq = quat[0]*quat[0] + quat[1]*quat[1]
                  + quat[2]*quat[2] + quat[3]*quat[3];

    /* Valid unit quaternion: norm² within ±10 % of 1.0 */
    return (norm_sq > 0.90f && norm_sq < 1.10f) ? 1U : 0U;
}

void bno055_Calibration_Status(uint8_t *status, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (1)
    {
        bno055_Read(BNO055_CALIB_STAT, status);
        snprintf(debug, 64, "Calib: sys=%d gyr=%d acc=%d mag=%d\n",
                 (*status >> 6) & 0x03,
                 (*status >> 4) & 0x03,
                 (*status >> 2) & 0x03,
                 (*status >> 0) & 0x03);
        serialWrite(debug);

        /* Success: all four sub-systems report level 3 */
        if (*status == 0xFF)
        {
            serialWrite("Device fully calibrated\n");
            return;
        }

        if (timeout_ms != 0 && (HAL_GetTick() - start) >= timeout_ms)
        {
            serialWrite("Calibration timeout — partially calibrated\n");
            return;
        }

        HAL_Delay(500);
    }
}

void bno055_Offset_Read(uint8_t *data, uint8_t size, uint8_t previousmode) {
    bno055_SetOprMode(BNO055_OPERATION_MODE_CONFIG);
    HAL_Delay(25);
    bno055_ReadByte(BNO055_ACC_OFFSET_X_LSB, data, size);
    HAL_Delay(25);
    bno055_SetOprMode(previousmode);
    HAL_Delay(25);
}

void bno055_Offset_Write(uint8_t *data, uint8_t size, uint8_t previousmode) {
    bno055_SetOprMode(BNO055_OPERATION_MODE_CONFIG);
    HAL_Delay(25);
    bno055_WriteByte(BNO055_ACC_OFFSET_X_LSB, data, size);
    HAL_Delay(25);
    bno055_SetOprMode(previousmode);
    HAL_Delay(25);
}

void bno055_Acc_Calibration(void) {
    serialWrite("Place device in 6 stable orientations (10s each)\n");
    HAL_Delay(10000);
    uint8_t data, calib;
    bno055_Read(BNO055_CALIB_STAT, &data);
    calib = (data >> 2) & 0x03;
    while (calib != 3) {
        serialWrite("Acc not calibrated — keep moving\n");
        bno055_Read(BNO055_CALIB_STAT, &data);
        calib = (data >> 2) & 0x03;
        HAL_Delay(500);
    }
    serialWrite("Acc calibrated\n");
}

void bno055_Gyro_Calibration(void) {
    serialWrite("Place device on a stable surface (10s)\n");
    HAL_Delay(10000);
    uint8_t data, calib;
    bno055_Read(BNO055_CALIB_STAT, &data);
    calib = (data >> 4) & 0x03;
    while (calib != 3) {
        serialWrite("Gyro not calibrated — hold still\n");
        bno055_Read(BNO055_CALIB_STAT, &data);
        calib = (data >> 4) & 0x03;
        HAL_Delay(500);
    }
    serialWrite("Gyro calibrated\n");
}

void bno055_Mag_Calibration(void) {
    serialWrite("Move device in figure-8 pattern (10s)\n");
    HAL_Delay(10000);
    uint8_t data, calib;
    bno055_Read(BNO055_CALIB_STAT, &data);
    calib = data & 0x03;
    while (calib != 3) {
        serialWrite("Mag not calibrated — continue movement\n");
        bno055_Read(BNO055_CALIB_STAT, &data);
        calib = data & 0x03;
        HAL_Delay(500);
    }
    serialWrite("Mag calibrated\n");
}

void bno055_Post(void)
{
    uint8_t data = 0;

    if (bno055_Read(BNO055_ST_RESULT, &data) != HAL_OK)
    {
        serialWrite("ERROR: Could not read ST_RESULT — check I2C wiring.\n");
        return;
    }

    snprintf(debug, 64, "Self-test result: 0x%02X\n", data);
    serialWrite(debug);
    serialWrite((data & 0x01) ? "Acc self-test  passed\n" : "Acc self-test  FAILED\n");
    serialWrite((data & 0x02) ? "Mag self-test  passed\n" : "Mag self-test  FAILED\n");
    serialWrite((data & 0x04) ? "Gyro self-test passed\n" : "Gyro self-test FAILED\n");
    serialWrite((data & 0x08) ? "MCU self-test  passed\n" : "MCU self-test  FAILED\n");

    if (data != 0x0F)
    {
        serialWrite("WARNING: One or more self-tests failed.\n");
        bno055_CheckSysStatus();
    }
}

void bno055_Low_Pass_Filter_3_axis_raw(int16_t *filtered, int16_t *raw, float alpha) {
    filtered[0] = (int16_t)(alpha * (float)raw[0] + (1.0f - alpha) * (float)filtered[0]);
    filtered[1] = (int16_t)(alpha * (float)raw[1] + (1.0f - alpha) * (float)filtered[1]);
    filtered[2] = (int16_t)(alpha * (float)raw[2] + (1.0f - alpha) * (float)filtered[2]);
}

void bno055_Low_Pass_Filter_3_axis_float(float *filtered, float *raw, float alpha) {
    filtered[0] = alpha * raw[0] + (1.0f - alpha) * filtered[0];
    filtered[1] = alpha * raw[1] + (1.0f - alpha) * filtered[1];
    filtered[2] = alpha * raw[2] + (1.0f - alpha) * filtered[2];
}

void bno055_High_Pass_Filter_3_axis_raw(int16_t *filtered, int16_t *raw, int16_t *prev_raw, float beta) {
    for (uint8_t i = 0; i < 3; i++) {
        filtered[i] = (int16_t)(0.5f * (2.0f - beta) * (float)(raw[i] - prev_raw[i])
                                + (1.0f - beta) * (float)filtered[i]);
        prev_raw[i] = raw[i];
    }
}

void bno055_ReadQuatData(float *buffer) {
    uint8_t data[8], data2[8];
    uint8_t retries = 0;

    do {
        bno055_ReadByte(BNO055_QUA_DATA_W_LSB, data,  8);
        bno055_ReadByte(BNO055_QUA_DATA_W_LSB, data2, 8);
        retries++;
    } while (memcmp(data, data2, 8) != 0 && retries < 5);

    int16_t raw_w = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    int16_t raw_x = (int16_t)((uint16_t)data[3] << 8 | data[2]);
    int16_t raw_y = (int16_t)((uint16_t)data[5] << 8 | data[4]);
    int16_t raw_z = (int16_t)((uint16_t)data[7] << 8 | data[6]);

    buffer[0] = (float)raw_w * BNO055_QUAT_SCALE;
    buffer[1] = (float)raw_x * BNO055_QUAT_SCALE;
    buffer[2] = (float)raw_y * BNO055_QUAT_SCALE;
    buffer[3] = (float)raw_z * BNO055_QUAT_SCALE;
}

/* ---------------------------------------------------------------
 * bno055_QuatSlerp
 * Spherical linear interpolation between two unit quaternions.
 * t = 0.0 → qa,  t = 1.0 → qb
 * out may alias qa or qb safely (result written to a local first).
 * --------------------------------------------------------------- */
void bno055_QuatSlerp(float *qa, float *qb, float t, float *out)
{
    float dot = qa[0]*qb[0] + qa[1]*qb[1] + qa[2]*qb[2] + qa[3]*qb[3];

    /* Take the shortest arc — negate qb if dot is negative */
    float qb_local[4];
    if (dot < 0.0f) {
        dot = -dot;
        qb_local[0] = -qb[0]; qb_local[1] = -qb[1];
        qb_local[2] = -qb[2]; qb_local[3] = -qb[3];
    } else {
        qb_local[0] = qb[0]; qb_local[1] = qb[1];
        qb_local[2] = qb[2]; qb_local[3] = qb[3];
    }

    /* Clamp dot to valid acos range */
    if (dot > 0.9995f) {
        /* Quaternions nearly identical — plain lerp + normalise */
        float result[4];
        result[0] = qa[0] + t * (qb_local[0] - qa[0]);
        result[1] = qa[1] + t * (qb_local[1] - qa[1]);
        result[2] = qa[2] + t * (qb_local[2] - qa[2]);
        result[3] = qa[3] + t * (qb_local[3] - qa[3]);
        float inv_norm = 1.0f / sqrtf(result[0]*result[0] + result[1]*result[1]
                                     + result[2]*result[2] + result[3]*result[3]);
        out[0] = result[0] * inv_norm;
        out[1] = result[1] * inv_norm;
        out[2] = result[2] * inv_norm;
        out[3] = result[3] * inv_norm;
        return;
    }

    float theta_0   = acosf(dot);
    float theta     = theta_0 * t;
    float sin_theta = sinf(theta);
    float sin_theta_0 = sinf(theta_0);

    float s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    out[0] = s0*qa[0] + s1*qb_local[0];
    out[1] = s0*qa[1] + s1*qb_local[1];
    out[2] = s0*qa[2] + s1*qb_local[2];
    out[3] = s0*qa[3] + s1*qb_local[3];
}

/* ---------------------------------------------------------------
 * bno055_HeadingLPF
 * Low-pass filter that works correctly across the 0/360 wrap.
 * Uses circular mean: average angles via their sin/cos components.
 *
 * alpha = smoothing factor  0.0 = frozen,  1.0 = no filtering
 * Typical values: 0.05 (very smooth/slow) … 0.20 (responsive)
 * --------------------------------------------------------------- */
float bno055_HeadingLPF(float new_heading, float prev_filtered, float alpha)
{
    float new_rad  = new_heading  * ((float)M_PI / 180.0f);
    float prev_rad = prev_filtered * ((float)M_PI / 180.0f);

    float sin_avg = alpha * sinf(new_rad)  + (1.0f - alpha) * sinf(prev_rad);
    float cos_avg = alpha * cosf(new_rad)  + (1.0f - alpha) * cosf(prev_rad);

    float result = atan2f(sin_avg, cos_avg) * (180.0f / (float)M_PI);
    return fmodf(result + 360.0f, 360.0f);
}

/* ---------------------------------------------------------------
 * bno055_HeadingJumpDetect
 * Returns 1 if the angular difference exceeds threshold_deg.
 * Handles the 0/360 boundary correctly.
 * threshold_deg: typically 15–30° for a stationary/slow platform.
 * --------------------------------------------------------------- */
uint8_t bno055_HeadingJumpDetect(float new_heading, float prev_heading,
                                  float threshold_deg)
{
    float diff = new_heading - prev_heading;
    /* Wrap difference to [-180, +180] */
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return (diff > threshold_deg || diff < -threshold_deg) ? 1 : 0;
}
