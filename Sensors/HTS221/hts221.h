#include "main.h"
#include "i2c.h"

#ifndef HTS221_H

#define HTS221_H


#define 	HTS221_DEVICE_ADDRESS_READ		0xBF
#define 	HTS221_DEVICE_ADDRESS_WRITE		0xBE

#define 	HTS221_WHO_AM_I_VALUE			0xBC
#define 	HTS221_WHO_AM_I_ADDRESS			0x0F

#define 	HTS221_CTRL1					0x20
#define 	HTS221_CTRL2					0x21
#define 	HTS221_CTRL3					0x22

#define 	HTS221_STATUS					0x27

#define 	HTS221_HUMIDITY_OUT_L 			0x28
#define 	HTS221_HUMIDITY_OUT_H 			0x29

#define 	HTS221_TEMP_OUT_L				0x2A
#define 	HTS221_TEMP_OUT_H				0x2B

#define 	HTS221_CALIB_H0_rH_x2			0x30
#define		HTS221_CALIB_H1_rH_x2			0x31
#define 	HTS221_CALIB_T0_degC_x8			0x32
#define		HTS221_CALIB_T1_degC_x8			0x33
#define 	HTS221_CALIB_T1_T0_MSB			0x35
#define 	HTS221_CALIB_H0_T0_OUT_L		0x36
#define		HTS221_CALIB_H0_T0_OUT_H		0x37
#define 	HTS221_CALIB_H1_T0_OUT_L		0x3A
#define		HTS221_CALIB_H1_T0_OUT_H		0x3B
#define 	HTS221_CALIB_T0_OUT_L			0x3C
#define		HTS221_CALIB_T0_OUT_H			0x3D
#define 	HTS221_CALIB_T1_OUT_L			0x3E
#define		HTS221_CALIB_T1_OUT_H			0x3F

static uint8_t (* HTS221_i2c_mread_single)(uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag) = 0;
static void (* HTS221_i2c_mread_multi)(uint8_t*buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag) = 0;
static void (* HTS221_i2c_mwrite)(uint8_t *buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag) = 0;

void HTS221_RegisterCallback_i2c_mread_single(void *callback);
void HTS221_RegisterCallback_i2c_mread_multi(void *callback);
void HTS221_RegisterCallback_i2c_mwrite(void *callback);

uint8_t HTS221_read_single(uint8_t reg_addr);
void HTS221_read_multi(uint8_t reg_addr, uint8_t* data, uint8_t len);

void HTS221_write_single(uint8_t reg_addr, uint8_t data);
void HTS221_write_multi(uint8_t reg_addr, uint8_t* data, uint8_t len);
void HTS221_init(void);

void HTS221_get_humidity_calib(void);
void HTS221_get_humidity(float* humidity_out);

void HTS221_get_temperature_calib(void);
void HTS221_get_temperature(float* temperature_out);

#endif  // HTS221_H
