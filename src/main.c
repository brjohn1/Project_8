// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 8, 11/5/26
// * NucleoF446ZE + CPEG222 Shield - CMSIS bare metal
// *
// * Reads a BME280 over I2C1 once per second (SysTick starts each reading)
// * and shows the results:
// *   OLED       temperature (C or F by slide switch S1), pressure, humidity
// *   SSD        temperature as xx.xx
// *   LEDs D1-D8 relative humidity, one LED per 12.5 %RH
// *   NeoPixels  temperature status: blue < 70 F, green 70-80 F, red > 80 F
// *****************************************************************


/*
 * PROJECT 8 - PIN REFERENCE
 *
 * BME280 SENSOR (I2C1):
 *   PB8  - SCL (clock)
 *   PB9  - SDA (data)
 *   VCC  - Sensor power
 *   GND  - Ground
 *   I2C Address: 0x76 (alternative: 0x77)
 *   Chip ID: 0x60
 *
 * S1 BUTTON:
 *   PC8  - HIGH = Celsius, LOW = Fahrenheit
 *
 * HUMIDITY LEDs:
 *   D1-D8 - PD0-PD7
 *   Each LED represents 12.5% humidity
 *
 * SSD (4-digit seven-segment display):
 *   Digit 1 - PE10
 *   Digit 2 - PE7
 *   Digit 3 - PB5
 *   Digit 4 - PB3
 *
 *   Segments:
 *   A  - PG9
 *   B  - PF12
 *   C  - PF13
 *   D  - PG14
 *   E  - PE8
 *   F  - PF15
 *   G  - PB4
 *   DP - PF14
 *
 * NEOPIXELS:
 *   PB11 - TIM2 CH4 (data output)
 *
 * OLED (SPI1):
 *   PA5  - SCK
 *   PA7  - MOSI
 *   PA6  - MISO
 *   PA15 - CS
 *   PG2  - DC
 *   PG3  - RESET
 */




#include "stm32f4xx.h"
#include "OLED.h"
#include "SysClock180MHz.h"
#include "I2C.h"
#include "BME280.h"
#include "ssd.h"
#include "NeoPixel.h"
#include <stdint.h>
#include <stdio.h>

#define S1_PORT            GPIOC
#define S1_PIN             8U      // S1 = PC8: 1 = Celsius, 0 = Fahrenheit
#define LED_PORT           GPIOD   // LEDs D1 to D8 = PD0 to PD7
#define LED_ACTIVE_HIGH    1       // set to 0 if the LEDs turn out to light on a LOW pin
#define MEASURE_PERIOD_MS  1000U   // one reading per second
#define CONVERSION_MS      20U     // the BME280 needs about 10 ms to finish a forced measurement

// Temperature thresholds in hundredths of a degree F
#define TEMP_COLD_X100     7000    // below 70 F = blue
#define TEMP_HOT_X100      8000    // above 80 F = red, 70 to 80 F = green

// Variables shared between the interrupts and the main loop
volatile uint32_t msTicks = 0;      // counts milliseconds, updated by SysTick
volatile uint8_t  measureFlag = 0;  // set by SysTick every second to start a reading

static BME280_Data data;            // most recent readings
static uint8_t haveData = 0;        // 0 until the first reading has been taken
static uint8_t pending = 0;         // 1 while waiting for a started measurement to finish
static uint32_t readyAtMs = 0;      // msTicks value at which the measurement will be finished
static uint8_t lastColor = 0xFF;    // last NeoPixel color sent, so we only resend when it changes

// Turns on the floating point unit; this code uses only integer math, but it keeps the build safe if hard-float flags are in platformio.ini
static void FPU_Enable(void) {
    SCB->CPACR |= (0xFU << 20);
    __DSB();
    __ISB();
}

// Sets slide switch S1 (PC8) up as a plain digital input
static void S1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    S1_PORT->MODER &= ~(0x3U << (S1_PIN * 2U));
}

// Sets PD0 to PD7 up as outputs for the humidity LEDs
static void LEDs_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    LED_PORT->MODER &= ~0xFFFFU;   // clear the mode bits for pins 0 to 7
    LED_PORT->MODER |=  0x5555U;   // 01 = general purpose output for each of the 8 pins
}

// Configures SysTick to interrupt every 1 ms (SysTick alone cannot count 1 s, its limit is about 93 ms)
static void SysTick_Init(void) {
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 1);
}

// Configures TIM5 to interrupt at 400 Hz (every 2.5 ms) so each of the 4 SSD digits refreshes at 100 Hz
static void TIM5_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    TIM5->PSC = 8999;  // TIM5 clock is 90 MHz (2 x 45 MHz APB1), so 90 MHz / 9000 = 10 kHz -> 0.1 ms per tick
    TIM5->ARR = 24;    // 25 ticks = 2.5 ms period -> 400 Hz
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR  = ~TIM_SR_UIF;
    TIM5->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM5_IRQn, 2);
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM5->CR1 = TIM_CR1_CEN;
}

// Every 1 ms: counts time, and every second raises the flag that starts a new reading
void SysTick_Handler(void) {
    static uint32_t periodCount = 0;
    msTicks++;
    if (++periodCount >= MEASURE_PERIOD_MS) {
        periodCount = 0;
        measureFlag = 1;
    }
}

// Every 2.5 ms: refreshes one SSD digit
void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR = ~TIM_SR_UIF;
        SSD_Refresh();
    }
}

// Prints a line on the OLED padded to 21 characters so leftovers from the previous reading are erased
static void showLine(uint8_t row, const char *text) {
    char padded[24];
    snprintf(padded, sizeof(padded), "%-21s", text);
    OLED_Print(row, 0, padded);
}

// Reads slide switch S1: 1 = Celsius, 0 = Fahrenheit
static uint8_t S1_IsCelsius(void) {
    return (S1_PORT->IDR & (1U << S1_PIN)) != 0U;
}

// Converts hundredths of a degree C to hundredths of a degree F: F = C x 9/5 + 32
static int32_t cToF_x100(int32_t c_x100) {
    return (c_x100 * 9) / 5 + 3200;
}

// Shows the temperature on the OLED (row 2) and the SSD in the unit chosen by S1
static void showTemperature(void) {
    char line[32];
    uint8_t celsius = S1_IsCelsius();
    int32_t t = celsius ? data.temp_c_x100 : cToF_x100(data.temp_c_x100);

    // OLED: split into whole and hundredths as integers, with a minus sign if needed
    int32_t a = (t < 0) ? -t : t;
    snprintf(line, sizeof(line), "Temp: %s%ld.%02ld %c", (t < 0) ? "-" : "", (long)(a / 100), (long)(a % 100), celsius ? 'C' : 'F');
    showLine(2, line);

    // SSD: the 4 digits show xx.xx (the decimal point is after the 2nd digit); it cannot show a negative number, so clamp at 0
    if (t < 0) t = 0;
    if (t > 9999) t = 9999;
    SSD_SetValueInt((uint16_t)t);
}

// Lights one LED per 12.5 %RH: LEDs D1 to Dn on, the rest off
static void showHumidityLEDs(uint32_t hum_q10) {
    // hum_q10 is %RH x 1024, so one LED = 12.5 x 1024 = 12800 counts; 38 %RH = 38912 -> 3 LEDs
    uint32_t n = hum_q10 / 12800U;
    if (n > 8U) n = 8U;
    uint32_t mask = (1U << n) - 1U;           // n ones in the low bits, e.g. 3 LEDs -> 0b00000111
#if LED_ACTIVE_HIGH
    LED_PORT->ODR = (LED_PORT->ODR & ~0xFFU) | mask;
#else
    LED_PORT->ODR = (LED_PORT->ODR & ~0xFFU) | (~mask & 0xFFU);
#endif
}

// Shows blue / green / red on all four NeoPixels by the temperature in F, only resending when the color changes
static void showNeoPixels(int32_t tempF_x100) {
    uint8_t color;   // 0 = blue, 1 = green, 2 = red
    if (tempF_x100 < TEMP_COLD_X100) {
        color = 0;
    } else if (tempF_x100 <= TEMP_HOT_X100) {
        color = 1;
    } else {
        color = 2;
    }
    if (color == lastColor) return;
    lastColor = color;

    if (color == 0)      NeoPixel_SetAll(0, 0, 255);
    else if (color == 1) NeoPixel_SetAll(0, 255, 0);
    else                 NeoPixel_SetAll(255, 0, 0);
    NeoPixel_Show_PB11();
}

// Updates every output from the newest readings
static void updateAll(void) {
    char line[32];

    showTemperature();

    // Pressure: the driver gives Pa, and hPa with 2 decimals is the same number split at the hundreds digit
    snprintf(line, sizeof(line), "Press: %lu.%02lu hPa", (unsigned long)(data.press_pa / 100U), (unsigned long)(data.press_pa % 100U));
    showLine(3, line);

    // Humidity: Q22.10 -> hundredths of a percent = hum x 100 / 1024
    uint32_t h100 = (data.hum_q10 * 100U) >> 10;
    snprintf(line, sizeof(line), "Hum: %lu.%02lu %%RH", (unsigned long)(h100 / 100U), (unsigned long)(h100 % 100U));
    showLine(4, line);

    showHumidityLEDs(data.hum_q10);
    showNeoPixels(cToF_x100(data.temp_c_x100));
}

int main(void) {
    FPU_Enable();

    // Same clock setup as Project 5: 16 MHz at reset, then 180 MHz
    SystemCoreClockUpdate();
    SystemClock_Config();
    SystemCoreClockUpdate();

    // Peripherals are set up after the clock change so the I2C, SysTick and timer values are correct
    OLED_Init();
    OLED_Clear();
    OLED_Print(0, 0, "CPEG222-F26 Project 8");
    OLED_Print(1, 0, "Brian Johnson");

    S1_Init();
    LEDs_Init();
    SSD_Init();
    SSD_SetDecimalPoint(1);   // xx.xx format: the decimal point sits after the 2nd digit
    SSD_SetValueInt(0);
    NeoPixel_Init_PB11();
    TIM5_Init();

    I2C1_Init();              // professor's driver: PB8 = SCL, PB9 = SDA, 100 kHz
    uint8_t status = BME280_Init();
    if (status == BME280_ERR_NOT_FOUND) {
        showLine(3, "BME280 not found");
        showLine(4, "(0x76 / 0x77)");
        while (1) { }
    } else if (status == BME280_ERR_BAD_ID) {
        char line[32];
        snprintf(line, sizeof(line), "Bad chip ID: 0x%02X", BME280_GetChipId());
        showLine(3, line);
        while (1) { }
    }

    char addrLine[32];
    snprintf(addrLine, sizeof(addrLine), "BME280 at 0x%02X", BME280_GetAddress());
    showLine(5, addrLine);

    measureFlag = 1;          // take the first reading right away instead of waiting a second
    SysTick_Init();

    uint8_t lastCelsius = S1_IsCelsius();

    while (1) {
        // Once per second SysTick raises measureFlag: tell the sensor to start a measurement
        if (measureFlag) {
            measureFlag = 0;
            BME280_StartMeasurement();
            readyAtMs = msTicks + CONVERSION_MS;
            pending = 1;
        }

        // When the sensor has had time to finish, read it and update every output (no waiting loop)
        if (pending && (int32_t)(msTicks - readyAtMs) >= 0) {
            pending = 0;
            BME280_Read(&data);
            haveData = 1;
            updateAll();
        }

        // If S1 was flipped between readings, redraw the temperature right away
        uint8_t celsius = S1_IsCelsius();
        if (celsius != lastCelsius) {
            lastCelsius = celsius;
            if (haveData) showTemperature();
        }
    }
    return 0;
}
