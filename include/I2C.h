#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void I2C1_Init(void);
uint8_t I2C1_IsDeviceReady(uint8_t addr);
void delay_ms(uint32_t ms);
void I2C_Start(void);
void I2C_Write(uint8_t data);
void I2C_Address(uint8_t addr, uint8_t direction);
void I2C_Stop(void);
void I2C_WriteRegister(uint8_t addr, uint8_t reg, uint8_t data);
void I2C_WriteBurstRegister(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t length);
uint8_t I2C_ReadRegister(uint8_t addr, uint8_t reg);
void I2C_ReadBurst(uint8_t addr, uint8_t start_reg, uint8_t *buffer, uint8_t length);

#endif // I2C_H