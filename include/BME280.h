// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 8, 11/5/26
// * BME280 temperature / humidity / pressure sensor driver (I2C1)
// *****************************************************************

#ifndef BME280_H
#define BME280_H

#include <stdint.h>

// Return codes for BME280_Init
#define BME280_OK            0
#define BME280_ERR_NOT_FOUND 1  // nothing answered at 0x76 or 0x77
#define BME280_ERR_BAD_ID    2  // something answered, but its chip ID is not 0x60

// One set of compensated readings
typedef struct {
    int32_t  temp_c_x100;  // temperature in hundredths of a degree C (2654 = 26.54 C)
    uint32_t press_pa;     // pressure in Pa (101876 = 1018.76 hPa)
    uint32_t hum_q10;      // relative humidity in Q22.10 format (divide by 1024 for %RH)
} BME280_Data;

// Finds the sensor at 0x76 or 0x77, checks the chip ID, reads the calibration data and sets up forced mode
uint8_t BME280_Init(void);

// Returns the I2C address the sensor was found at (valid after a successful BME280_Init)
uint8_t BME280_GetAddress(void);

// Returns the chip ID that was read (0x60 for a BME280); useful for showing an error
uint8_t BME280_GetChipId(void);

// Tells the sensor to take one measurement; it needs about 10 ms, then BME280_Read can be called
void BME280_StartMeasurement(void);

// Reads the raw measurement registers and converts them with the calibration data
void BME280_Read(BME280_Data *out);

#endif
