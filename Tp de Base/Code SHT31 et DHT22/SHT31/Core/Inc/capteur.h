#ifndef CAPTEUR_H_
#define CAPTEUR_H_


#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

//devices
#define SENSOR_ADDRESS     0x44


//commande


#define SHT31_MEAS_HIGHREP_STRETCH    0x2C06
#define SHT31_MEAS_MEDREP_STRETCH     0x2C0D
#define SHT31_MEAS_LOWREP_STRETCH     0x2C10
#define SHT31_MEAS_HIGHREP            0x2400
#define SHT31_MEAS_MEDREP             0x240B
#define SHT31_MEAS_LOWREP             0x2416
#define SHT31_READSTATUS              0xF32D
#define SHT31_CLEARSTATUS             0x3041
#define SHT31_SOFTRESET               0x30A2
#define SHT31_HEATEREN                0x306D
#define SHT31_HEATERDIS               0x3066




void EnvoyezCommande(I2C_HandleTypeDef* I2Cx, uint16_t cmd);



#endif
