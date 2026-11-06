// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 8, 11/5/26
// *****************************************************************

#include "BME280.h"
#include "I2C.h"

// BME280 register addresses (from the datasheet memory map)
#define REG_CALIB_00   0x88  // calibration block 1: 0x88 to 0xA1 (26 bytes)
#define REG_CHIP_ID    0xD0  // always reads 0x60 on a BME280
#define REG_RESET      0xE0
#define REG_CALIB_26   0xE1  // calibration block 2: 0xE1 to 0xE7 (7 bytes)
#define REG_CTRL_HUM   0xF2  // humidity oversampling; must be written BEFORE ctrl_meas
#define REG_STATUS     0xF3  // bit 0 = im_update (NVM calibration still copying)
#define REG_CTRL_MEAS  0xF4  // temp + pressure oversampling and the power mode
#define REG_DATA       0xF7  // press msb/lsb/xlsb, temp msb/lsb/xlsb, hum msb/lsb (8 bytes)

#define CHIP_ID_BME280 0x60
#define CTRL_HUM_X1    0x01  // humidity oversampling x1
#define CTRL_MEAS_FORCED_X1 0x25  // temp x1 (001), pressure x1 (001), forced mode (01)

static uint8_t sensorAddr = 0;
static uint8_t chipId = 0;

// Factory calibration coefficients, stored inside every sensor chip
static uint16_t dig_T1; static int16_t dig_T2, dig_T3;
static uint16_t dig_P1; static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3; static int16_t dig_H2, dig_H4, dig_H5; static int8_t dig_H6;

// Carries the fine temperature from the temperature math into the pressure and humidity math
static int32_t t_fine;

// Short busy-wait used between I2C probes so the previous STOP condition finishes (about 1 ms at 180 MHz)
static void shortDelay(void) {
    for (volatile uint32_t i = 0; i < 30000U; i++) { }
}

// Reads the 33 calibration bytes and unpacks them into the dig_ variables (H4 and H5 share a byte)
static void readCalibration(void) {
    uint8_t a[26];
    uint8_t e[7];
    I2C_ReadBurst(sensorAddr, REG_CALIB_00, a, 26);
    I2C_ReadBurst(sensorAddr, REG_CALIB_26, e, 7);

    dig_T1 = (uint16_t)(a[1] << 8 | a[0]);
    dig_T2 = (int16_t)(a[3] << 8 | a[2]);
    dig_T3 = (int16_t)(a[5] << 8 | a[4]);
    dig_P1 = (uint16_t)(a[7] << 8 | a[6]);
    dig_P2 = (int16_t)(a[9] << 8 | a[8]);
    dig_P3 = (int16_t)(a[11] << 8 | a[10]);
    dig_P4 = (int16_t)(a[13] << 8 | a[12]);
    dig_P5 = (int16_t)(a[15] << 8 | a[14]);
    dig_P6 = (int16_t)(a[17] << 8 | a[16]);
    dig_P7 = (int16_t)(a[19] << 8 | a[18]);
    dig_P8 = (int16_t)(a[21] << 8 | a[20]);
    dig_P9 = (int16_t)(a[23] << 8 | a[22]);
    dig_H1 = a[25];                                   // register 0xA1

    dig_H2 = (int16_t)(e[1] << 8 | e[0]);             // 0xE1, 0xE2
    dig_H3 = e[2];                                    // 0xE3
    dig_H4 = (int16_t)(((int16_t)(int8_t)e[3] * 16) | (e[4] & 0x0F));  // 0xE4 and low nibble of 0xE5
    dig_H5 = (int16_t)(((int16_t)(int8_t)e[5] * 16) | (e[4] >> 4));    // 0xE6 and high nibble of 0xE5
    dig_H6 = (int8_t)e[6];                            // 0xE7
}

// Datasheet 32-bit integer formula: raw temperature -> hundredths of a degree C, and sets t_fine
static int32_t compensateTemp(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

// Datasheet 32-bit integer formula: raw pressure -> Pa (needs t_fine from compensateTemp)
static uint32_t compensatePress(int32_t adc_P) {
    int32_t var1 = (t_fine >> 1) - (int32_t)64000;
    int32_t var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
    if (var1 == 0) {
        return 0; // avoid dividing by zero
    }
    uint32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (uint32_t)(var2 >> 12))) * 3125U;
    if (p < 0x80000000U) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2U;
    }
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}

// Datasheet 32-bit integer formula: raw humidity -> %RH in Q22.10 (needs t_fine from compensateTemp)
static uint32_t compensateHum(int32_t adc_H) {
    int32_t v = (t_fine - ((int32_t)76800));
    v = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v)) + ((int32_t)16384)) >> 15) *
         (((((((v * ((int32_t)dig_H6)) >> 10) * (((v * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
           ((int32_t)dig_H2) + 8192) >> 14));
    v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v = (v < 0 ? 0 : v);
    v = (v > 419430400 ? 419430400 : v);
    return (uint32_t)(v >> 12);
}

// Probes the two possible addresses, checks the chip ID, loads calibration, and writes the humidity setting
uint8_t BME280_Init(void) {
    // The BME280's address is 0x76 or 0x77 depending on how its SDO pin is wired, so try both
    if (I2C1_IsDeviceReady(0x76)) {
        sensorAddr = 0x76;
    } else {
        shortDelay();
        if (I2C1_IsDeviceReady(0x77)) {
            sensorAddr = 0x77;
        } else {
            return BME280_ERR_NOT_FOUND;
        }
    }
    shortDelay();

    chipId = I2C_ReadRegister(sensorAddr, REG_CHIP_ID);
    if (chipId != CHIP_ID_BME280) {
        return BME280_ERR_BAD_ID;
    }

    // Wait until the sensor has finished copying its calibration data out of internal memory
    uint32_t guard = 100000U;
    while ((I2C_ReadRegister(sensorAddr, REG_STATUS) & 0x01U) && --guard) { }

    readCalibration();

    // Humidity oversampling x1; the datasheet says this write only takes effect after the next write to ctrl_meas
    I2C_WriteRegister(sensorAddr, REG_CTRL_HUM, CTRL_HUM_X1);
    return BME280_OK;
}

uint8_t BME280_GetAddress(void) { return sensorAddr; }
uint8_t BME280_GetChipId(void)  { return chipId; }

// Forced mode: one measurement now, then the sensor goes back to sleep (about 10 ms, so the caller waits ~20 ms)
void BME280_StartMeasurement(void) {
    I2C_WriteRegister(sensorAddr, REG_CTRL_HUM, CTRL_HUM_X1);
    I2C_WriteRegister(sensorAddr, REG_CTRL_MEAS, CTRL_MEAS_FORCED_X1);
}

// Reads the 8 data bytes, builds the 20-bit and 16-bit raw values, and runs the compensation formulas
void BME280_Read(BME280_Data *out) {
    uint8_t d[8];
    I2C_ReadBurst(sensorAddr, REG_DATA, d, 8);

    int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);  // 20 bits
    int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);  // 20 bits
    int32_t adc_H = ((int32_t)d[6] << 8)  |  (int32_t)d[7];                      // 16 bits

    out->temp_c_x100 = compensateTemp(adc_T);   // must come first: it sets t_fine
    out->press_pa    = compensatePress(adc_P);
    out->hum_q10     = compensateHum(adc_H);
}
