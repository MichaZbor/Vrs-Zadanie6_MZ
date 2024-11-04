#include "HTS221.h"

float HTS221_HumidityK;
float HTS221_HumidityShift;
float HTS221_TemperatureK;
float HTS221_TemperatureShift;

uint8_t HTS221_addr = HTS221_DEVICE_ADDRESS_WRITE;

void HTS221_RegisterCallback_i2c_mread_single(void *callback){
        if(callback != 0) i2c_mread_single = callback;
}
void HTS221_RegisterCallback_i2c_mread_multi(void *callback){
        if(callback != 0) i2c_mread_multi = callback;
}
void HTS221_RegisterCallback_i2c_mwrite(void *callback){
        if(callback != 0) i2c_mwrite = callback;
}

uint8_t HTS221_read_single(uint8_t reg_addr) {
	if (i2c_mread_single) return 0;
    return i2c_mread_single(reg_addr, HTS221_addr, 1);
}

void HTS221_read_multi(uint8_t reg_addr, uint8_t* data, uint8_t len) {
	if (i2c_mread_multi) return;
    i2c_master_read_multi(data, len, reg_addr | 0x80, HTS221_addr, 1);
}

void HTS221_write_single(uint8_t reg_addr, uint8_t data) {
	if (i2c_mwrite) return;
	i2c_mwrite(&data, 1, reg_addr, HTS221_addr, 0);
}

void HTS221_write_multi(uint8_t reg_addr, uint8_t* data, uint8_t len) {
	i2c_mwrite(data, len, reg_addr, HTS221_addr | 0x80, 0);
}

void HTS221_init(void) {
    uint8_t whoAmI = HTS221_read_single(HTS221_WHO_AM_I_ADDRESS);

    if (!(whoAmI == HTS221_WHO_AM_I_VALUE)) {
    	HTS221_addr = HTS221_DEVICE_ADDRESS_READ;
    	whoAmI = HTS221_read_single(HTS221_WHO_AM_I_ADDRESS);
    	if (!(whoAmI == HTS221_WHO_AM_I_VALUE))
    		1/0;	// if none address is WHO_AM_I_VALUE, raise error
    }

    HTS221_write_single(HTS221_CTRL1, 0b10000110); // 134 set settings:	PD(active), BDU(manual), ODR(10-> 7Hz)

    HTS221_get_humidity_calib();
    HTS221_get_temperature_calib();
}

void HTS221_get_humidity_calibration(void) {
    uint8_t calibration_data[2]; 	// rH -> H0 & H1
    uint8_t H0_T0_Out_data[2], H1_T0_Out_data[2];  // T0_OUT -> H0 & H1
    int16_t H0_T0_Out, H1_T0_Out;

    HTS221_read_multi(HTS221_CALIB_H0_rH_x2, calibration_data, 2);

    uint8_t H0_rH = calibration_data[0] >> 1; // H0_rH is stored as (value / 2)
    uint8_t H1_rH = calibration_data[1] >> 1; // H1_rH is stored as (value / 2)

    HTS221_read_multi(HTS221_CALIB_H0_T0_OUT_H, H0_T0_Out_data, 2);
    HTS221_read_multi(HTS221_CALIB_H0_T0_OUT_L, H1_T0_Out_data, 2);

    H0_T0_Out = (int16_t)((uint16_t)H0_T0_Out_data[1] << 8 | (uint16_t)H0_T0_Out_data[0]);
    H1_T0_Out = (int16_t)((uint16_t)H1_T0_Out_data[1] << 8 | (uint16_t)H1_T0_Out_data[0]);

    // Humidity calibration coefficient K and shift
    HTS221_HumidityK = (float)(H1_rH - H0_rH) / (H1_T0_Out - H0_T0_Out);
    HTS221_HumidityShift = H0_rH - HTS221_HumidityK * H0_T0_Out;
}

void HTS221_get_humidity(float* humidity_out) {
    uint8_t H_Out_data[2];
    int16_t H_Out;

    HTS221_read_multi(HTS221_HUMIDITY_OUT_L, H_Out_data, 2);
    H_Out = H_Out_data[0] | (H_Out_data[1] << 8);

    *humidity_out = (H_Out * HTS221_HumidityK + HTS221_HumidityShift);
}




void HTS221_get_temperature_calibration(void) {
    uint8_t t0_out_data[2];
    uint8_t t1_out_data[2];
    int16_t t0_out, t1_out;

    uint8_t t1t0_msb = HTS221_read_single(HTS221_CALIB_T1_T0_MSB);

    uint16_t t0_degC = (((uint16_t)(t1t0_msb & 0x03) << 8) | (uint16_t)(HTS221_read_single(HTS221_CALIB_T0_degC_x8)));
    uint16_t t1_degC = (((uint16_t)(t1t0_msb & 0x0C) << 8) | (uint16_t)(HTS221_read_single(HTS221_CALIB_T1_degC_x8)));

    //int16_t t0_out = (int16_t)(((uint16_t)(HTS221_read_single(HTS221_CALIB_T0_OUT_L))) | (((unt16_t)(HTS221_read_single(HTS221_CALIB_T0_OUT_H))) << 8));
    //int16_t t1_out = (int16_t)(((uint16_t)(HTS221_read_single(HTS221_CALIB_T1_OUT_L))) | (((unt16_t)(HTS221_read_single(HTS221_CALIB_T1_OUT_H))) << 8));
    HTS221_read_multi(HTS221_CALIB_T0_OUT_L, t0_out_data, 2);
    t0_out = (int16_t)(((uint16_t)t0_out_data[1]) << 8 | ((uint16_t)t0_out_data[0]));

    HTS221_read_multi(HTS221_CALIB_T1_OUT_L, t1_out_data, 2);
    t1_out = (int16_t)(((uint16_t)t1_out_data[1]) << 8 | ((uint16_t)t1_out_data[0]));

    // Calculate the temperature calibration coefficient K and shift
    HTS221_TemperatureK = (t1_degC - t0_degC) / (8.0 * (t1_out - t0_out));
    HTS221_TemperatureShift = (t0_degC / 8.0) - HTS221_TemperatureK * t0_out;
}


void HTS221_get_temperature(float* temperature_out) {
    uint8_t t_out_data[2];
    int16_t t_out;

    HTS221_read_multi(HTS221_TEMP_OUT_L, t_out_data, 2);
    t_out = (int16_t)(((uint16_t)t_out_data[1]) << 8 | ((uint16_t)t_out_data[0]));

    *temperature_out = (t_out * (-HTS221_TemperatureK)) + HTS221_TemperatureShift;
}
