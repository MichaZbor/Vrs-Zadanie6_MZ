#include "main.h"
#include "i2c.h"

#ifndef LPS25HB_H

#define LPS25HB_H

#define 	LPS25HB_DEVICE_ADDRESS_WRITE_0		0xB8
#define 	LPS25HB_DEVICE_ADDRESS_WRITE_1		0xBA

#define 	LPS25HB_WHO_AM_I_VALUE				0xBD
#define 	LPS25HB_WHO_AM_I_ADDRESS			0x0F

#define 	LPS25HB_CTRL1						0x20

#define 	LPS25HB_PRESSURE_OUT_XL				0x28
#define 	LPS25HB_PRESSURE_OUT_L				0x29
#define 	LPS25HB_PRESSURE_OUT_H				0x2A

#define		LPS25HB_RPDS_L						0x39
#define		LPS25HB_RPDS_H						0x3A

static uint8_t (* i2c_mread_single)(uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag) = 0;
static void (* i2c_mread_multi)(uint8_t*buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag) = 0;
static void (* i2c_mwrite)(uint8_t *buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag) = 0;

void LPS25HB_RegisterCallback_i2c_mread_single(void *callback);
void LPS25HB_RegisterCallback_i2c_mread_multi(void *callback);
void LPS25HB_RegisterCallback_i2c_mwrite(void *callback);

uint8_t LPS25HB_read_single(uint8_t reg_addr);
void LPS25HB_read_multi(uint8_t reg_addr, uint8_t* data, uint8_t len);

void LPS25HB_write_single(uint8_t reg_addr, uint8_t data);
void LPS25HB_write_multi(uint8_t reg_addr, uint8_t* data, uint8_t len);


void LPS25HB_init(void);

void LPS25HB_get_pressure_calib(void);
void HTS221_get_pressure(float* pressure_out);
#endif /* LPS25HB_H */
