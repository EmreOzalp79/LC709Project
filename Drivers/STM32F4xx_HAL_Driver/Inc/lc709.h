/**
  ******************************************************************************
  * @file    lc709204FuelGauge.h
  * @author  Emre Özalp
  * @brief   Batery management module
  *https://github.com/controllerstech/STM32/blob/master/LCD1602_I2C_NOHAL/I2C.c
  */
#ifndef __lc709_batery_management_H
#define __lc70_H


#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

/*Define */
#define I2Cx                              I2C1 //hangi i2c hattı kullanılıyorsa o aktif edilir
#define LC709204F_TIME_TO_EMPTY           0x03
#define LC709204F_BEFORE_RSOC             0x04
#define LC709204F_TIME_TO_FULL            0x05
#define LC709204F_TSENSE1_THERMB          0x06
#define LC709204F_INITIAL_RSOC            0x07
#define LC709204F_CELL_TEMP               0x08
#define LC709204F_CELL_VOLTAGE            0x09
#define LC709204F_CURRENT_DIR             0x0A
#define LC709204F_APA                     0x0B
#define LC709204F_APT                     0x0C
#define LC709204F_RSOC                    0x0D
#define LC709204F_TSENSE2_THERMB          0x0E
#define LC709204F_ITE                     0x0F
#define LC709204F_IC_VERSION              0x11
#define LC709204F_CHANGE_PARAM            0x12
#define LC709204F_ALARM_LOW_RSOC          0x13
#define LC709204F_ALARM_LOW_CELL_VLT      0x14
#define LC709204F_IC_POWERMODE            0x15
#define LC709204F_STATUS_BIT              0x16
#define LC709204F_CYCLE_COUNT             0x17
#define LC709204F_BATTERY_STATUS          0x19
#define LC709204F_NUMBER_PARAM            0x1A
#define LC709204F_TERM_CURRENT_RATE       0x1C
#define LC709204F_EMPTY_CELL_VOLTAGE      0x1D
#define LC709204F_ITE_OFFSET              0x1E
#define LC709204F_ALARM_HIGH_CELL_VLT     0x1F
#define LC709204F_ALARM_LOW_TEMP          0x20
#define LC709204F_ALARM_HIGH_TEMP         0x21
#define LC709204F_TOTAL_RUN_TIME_L        0x24
#define LC709204F_TOTAL_RUN_TIME_H        0x25
#define LC709204F_ACC_TEMPERATURE_L       0x26
#define LC709204F_ACC_TEMPERATURE_H       0x27
#define LC709204F_ACC_RSOC_L              0x28
#define LC709204F_ACC_RSOC_H              0x29
#define LC709204F_MAX_CELL_VOLTAGE        0x2A
#define LC709204F_MIN_CELL_VOLTAGE        0x2B
#define LC709204F_MAX_CELL_TEMP_TSENSE1   0x2C
#define LC709204F_MIN_CELL_TEMP_TSENSE1   0x2D
#define LC709204F_AMB_TEMP_TSENSE2        0x30
#define LC709204F_STATE_OF_HEALTH         0x32
#define LC709204F_USER_ID_L               0x36
#define LC709204F_USER_ID_H               0x37

typedef enum {
  LC709204F_TEMPERATURE_I2C = 0x0000,
  LC709204F_TEMPERATURE_THERMISTOR = 0x0001,
} lc709204_tempmode_t;

typedef enum {
  LC709204F_POWER_OPERATE = 0x0001,
  LC709204F_POWER_SLEEP = 0x0002,
} lc709204_powermode_t;

typedef enum {
  LC709204F_APA_100MAH = 0x08,
  LC709204F_APA_200MAH = 0x0B,
  LC709204F_APA_500MAH = 0x10,
  LC709204F_APA_1000MAH = 0x19,
  LC709204F_APA_2000MAH = 0x2D,
  LC709204F_APA_3000MAH = 0x36,
} lc709204_adjustment_t;

typedef enum {
  LC709204F_AUTO_MODE = 0x0000,
  LC709204F_CHARGE_MODE = 0x0001,
  LC709204F_DISCHARGE = 0xFFFF,
} lc709204_current_mode_t;
/*Function */
uint8_t lc709_crc8(uint8_t *data, int len);
void I2C_Start (void);	
void I2C_Write (uint8_t data);
void I2C_Address (uint8_t Address);
void I2C_Stop (void);
void I2C_WriteMulti (uint8_t *data, uint8_t size);
void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size);
void writeWord (uint8_t Reg, uint16_t data);
void readWord (uint8_t Reg, uint16_t *data);

  
void setBattProfile(uint16_t b);
uint16_t getBattProfile(void);
void setThermistorB(uint16_t b);
uint16_t getThermistorB(void) ;
void setPowerMode(lc709204_powermode_t t);
void setAlarmVoltage(float voltage);
void setAlarmRSOC(uint8_t percent);
void setPackAPA(uint16_t apa_value);
void setPackSize(lc709204_adjustment_t apa);
void setTemperatureMode(lc709204_tempmode_t t);
float timeToFull(void);
float timeToEmpty (void);
float getCellTemperature(void);
float cellPercent(void);
float cellVoltage(void) ;
void initRSOC(void);
uint16_t getICversion(void);
void currentDirection(lc709204_current_mode_t t);
void emptyCellVoltage(uint16_t a);
uint16_t getStateOfHealth(void);



#endif /* __lc709_H */
