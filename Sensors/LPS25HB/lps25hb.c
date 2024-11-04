#include "LPS25HB.h"

uint8_t LPS25HB_addr = LPS25HB_DEVICE_ADDRESS_WRITE_0;
int16_t LPS25HB_PressureOffset;

void LPS25HB_RegisterCallback_i2c_mread_single(void *callback){
        if(callback != 0) LPS25HB_i2c_mread_single = callback;
        return;
}
void LPS25HB_RegisterCallback_i2c_mread_multi(void *callback){
        if(callback != 0) LPS25HB_i2c_mread_multi = callback;
        return;
}
void LPS25HB_RegisterCallback_i2c_mwrite(void *callback){
        if(callback != 0) LPS25HB_i2c_mwrite = callback;
        return;
}

uint8_t LPS25HB_read_single(uint8_t reg_addr) {
	if (LPS25HB_i2c_mread_single == 0) return 0;
    return LPS25HB_i2c_mread_single(reg_addr, LPS25HB_addr, 1);
}

void LPS25HB_read_multi(uint8_t reg_addr, uint8_t* data, uint8_t len) {
	if (LPS25HB_i2c_mread_multi == 0) return;
	LPS25HB_i2c_mread_multi(data, len, reg_addr | 0x80, LPS25HB_addr, 1);
	return;
}

void LPS25HB_write_single(uint8_t reg_addr, uint8_t data) {
	if (LPS25HB_i2c_mwrite == 0) return;
	LPS25HB_i2c_mwrite(&data, 1, reg_addr, LPS25HB_addr, 0);
	return;
}

void LPS25HB_write_multi(uint8_t reg_addr, uint8_t* data, uint8_t len) {
	if (LPS25HB_i2c_mwrite == 0) return;
	LPS25HB_i2c_mwrite(data, len, reg_addr, LPS25HB_addr | 0x80, 0);
	return;
}


void LPS25HB_init() {
	uint8_t whoAmI = LPS25HB_read_single(LPS25HB_WHO_AM_I_ADDRESS);

	if (!(whoAmI == LPS25HB_WHO_AM_I_VALUE)) {
		LPS25HB_addr = LPS25HB_DEVICE_ADDRESS_WRITE_1;
	   	whoAmI = LPS25HB_read_single(LPS25HB_WHO_AM_I_ADDRESS);
	   	if (!(whoAmI == LPS25HB_WHO_AM_I_VALUE))
	   		1/0;	// if none address is WHO_AM_I_VALUE, raise error
	}

	LPS25HB_write_single(LPS25HB_CTRL1, 0b10100100); // 164 set settings:	PD(active), BDU(manual), ODR(10-> 7Hz)
	LPS25HB_get_pressure_calib();
	return;
}


void LPS25HB_get_pressure_calib(void){
	uint8_t p_out_data[2];

	LPS25HB_read_multi(LPS25HB_RPDS_L, p_out_data, 2);
	LPS25HB_PressureOffset = (int16_t)(((uint16_t)p_out_data[1]) << 8 | ((uint16_t)p_out_data[0]));
	return;
}

void LPS25HB_get_pressure(float* pressure_out) {
    uint8_t p_out_data[3];

    LPS25HB_read_multi(LPS25HB_PRESSURE_OUT_XL, p_out_data, 3);
    uint32_t p_out = ((((uint32_t)p_out_data[1]) << 16) | (((uint32_t)p_out_data[1]) << 8) | ((uint32_t)p_out_data[0]));

    // Calculate the actual pressure value
    *pressure_out = (p_out / 4096.0f) + LPS25HB_PressureOffset;
    return;
}
