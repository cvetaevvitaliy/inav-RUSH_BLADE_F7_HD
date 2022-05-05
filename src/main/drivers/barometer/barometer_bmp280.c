/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "build/build_config.h"
#include "build/debug.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp280.h"

#if defined(USE_BARO_BMP280)

// BMP280, address 0x76

#ifndef RUSH_BLADE_F7_HD
typedef struct bmp280_calib_param_s {
    uint16_t dig_T1; /* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
    int32_t t_fine; /* calibration t_fine data */
} bmp280_calib_param_t;

STATIC_UNIT_TESTED bmp280_calib_param_t bmp280_cal;
// uncompensated pressure and temperature
int32_t bmp280_up = 0;
int32_t bmp280_ut = 0;
#endif

#ifdef RUSH_BLADE_F7_HD
//  QMP6988
typedef struct qmp6988_calib_param_s {
		float Coe_a0;
		float Coe_a1;
		float Coe_a2;
		float Coe_b00;
		float Coe_bt1;
		float Coe_bt2;
		float Coe_bp1;
		float Coe_b11;
		float Coe_bp2;
		float Coe_b12;
		float Coe_b21;
		float Coe_bp3;
} qmp6988_calib_param_t;
STATIC_UNIT_TESTED qmp6988_calib_param_t qmp6988_cal;
// uncompensated pressure and temperature
int32_t qmp6988_up = 0;
int32_t qmp6988_ut = 0;
uint8_t sensor_data[QMP6988_DATA_FRAME_SIZE];
#endif

static bool bmp280_start_ut(baroDev_t * baro)
{
    UNUSED(baro);
    return true;
}

static bool bmp280_get_ut(baroDev_t * baro)
{
    UNUSED(baro);
    return true;
}

static bool bmp280_start_up(baroDev_t * baro)
{
#ifndef RUSH_BLADE_F7_HD
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    busWrite(baro->busDev, BMP280_CTRL_MEAS_REG, BMP280_MODE);
#else
		// start measurement
		busWrite(baro->busDev, QMP6988_CTRL_MEAS_REG, QMP6988_PWR_SAMPLE_MODE);
		
#endif
    return true;
}

static bool bmp280_get_up(baroDev_t * baro)
{
#ifndef RUSH_BLADE_F7_HD
    uint8_t data[BMP280_DATA_FRAME_SIZE];

    //error free measurements
    static int32_t bmp280_up_valid;
    static int32_t bmp280_ut_valid;

    //read data from sensor
    bool ack = busReadBuf(baro->busDev, BMP280_PRESSURE_MSB_REG, data, BMP280_DATA_FRAME_SIZE);

    //check if pressure and temperature readings are valid, otherwise use previous measurements from the moment
    if (ack) {
        bmp280_up = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
        bmp280_ut = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
        bmp280_up_valid = bmp280_up;
        bmp280_ut_valid = bmp280_ut;
    }
    else {
        //assign previous valid measurements
        bmp280_up = bmp280_up_valid;
        bmp280_ut = bmp280_ut_valid;
    }
		
		return ack;
#else
	// read data from sensor
	 bool ack = busReadBuf(baro->busDev, QMP6988_PRESSURE_MSB_REG, sensor_data, QMP6988_DATA_FRAME_SIZE);
	
	//error free measurements
    static int32_t qmp6988_up_valid;
    static int32_t qmp6988_ut_valid;
	
	UNUSED(baro);
	if (ack) {
		qmp6988_up = sensor_data[0] << 16 | sensor_data[1] << 8 | sensor_data[2];
		qmp6988_ut = sensor_data[3] << 16 | sensor_data[4] << 8 | sensor_data[5];
		qmp6988_up_valid = qmp6988_up;
		qmp6988_ut_valid = qmp6988_ut;
	} else {
		//assign previous valid measurements
        qmp6988_up = qmp6988_up_valid;
        qmp6988_ut = qmp6988_ut_valid;
	}
	
	return true;

#endif
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
#ifndef RUSH_BLADE_F7_HD
static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
    bmp280_cal.t_fine = var1 + var2;
    T = (bmp280_cal.t_fine * 5 + 128) >> 8;

    return T;
}
#else
static float bmp280_compensate_T(int32_t adc_T) {
		int32_t var1;
		float T;
		
		var1=adc_T-1024*1024*8;
		T= qmp6988_cal.Coe_a0+qmp6988_cal.Coe_a1*(float)var1+qmp6988_cal.Coe_a2*(float)var1*(float)var1;
		
		return T;
}
#endif

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
#ifndef RUSH_BLADE_F7_HD
static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_P1) >> 33;
    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);
    return (uint32_t)p;
}
#endif

STATIC_UNIT_TESTED bool bmp280_calculate(baroDev_t * baro, int32_t * pressure, int32_t * temperature)
{
    UNUSED(baro);
#ifndef RUSH_BLADE_F7_HD
    int32_t t = bmp280_compensate_T(bmp280_ut);
    uint32_t p = bmp280_compensate_P(bmp280_up);

    if (pressure) {
        *pressure = (int32_t)(p / 256);
    }

    if (temperature) {
        *temperature = t;
    }
#else
		float tr,pr;
		float Dp;

		tr = bmp280_compensate_T(qmp6988_ut);
		Dp = (float)qmp6988_up - 1024*1024*8;

		pr = qmp6988_cal.Coe_b00+qmp6988_cal.Coe_bt1*tr+qmp6988_cal.Coe_bp1*Dp+qmp6988_cal.Coe_b11*tr*Dp+qmp6988_cal.Coe_bt2*tr*tr+qmp6988_cal.Coe_bp2*Dp*Dp+qmp6988_cal.Coe_b12*Dp*tr*tr
				 +qmp6988_cal.Coe_b21*Dp*Dp*tr+qmp6988_cal.Coe_bp3*Dp*Dp*Dp;

		if (pr)
			*pressure = (int32_t)(pr);
		if (tr)
			*temperature = (int32_t)tr/256;

#endif
    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(busDevice_t * busDev)
{
#ifndef RUSH_BLADE_F7_HD
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        uint8_t chipId = 0;

        delay(100);

        bool ack = busRead(busDev, BMP280_CHIP_ID_REG, &chipId);

        if ((ack && chipId == BMP280_DEFAULT_CHIP_ID)  || (ack && chipId == BME280_DEFAULT_CHIP_ID)){
            return true;
        }
    };

    return false;
#else
	
	uint8_t state = 0;
	delay(100);
	busRead(busDev, 0xE0, &state);  /* reset */
	
	for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
		uint8_t qmp6988_chip_id = 0;
		delay(100);
		busRead(busDev, QMP6988_CHIP_ID_REG, &qmp6988_chip_id);  /* read Chip Id */
		if (qmp6988_chip_id == QMP6988_DEFAULT_CHIP_ID){
			return true;
		}
	}
		return false;
#endif
}

bool bmp280Detect(baroDev_t *baro)
{
    baro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_BMP280, 0, OWNER_BARO);
    if (baro->busDev == NULL) {
        return false;
    }
		delay(100);

    busSetSpeed(baro->busDev, BUS_SPEED_STANDARD);

    if (!deviceDetect(baro->busDev)) {
        busDeviceDeInit(baro->busDev);
        return false;
    }
#ifndef RUSH_BLADE_F7_HD
    // read calibration
    busReadBuf(baro->busDev, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (uint8_t *)&bmp280_cal, 24);

    //set filter setting
    busWrite(baro->busDev, BMP280_CONFIG_REG, BMP280_FILTER);

    // set oversampling + power mode (forced), and start sampling
    busWrite(baro->busDev, BMP280_CTRL_MEAS_REG, BMP280_MODE);

    baro->ut_delay = 0;
    baro->get_ut = bmp280_get_ut;
    baro->start_ut = bmp280_start_ut;

    baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
    baro->start_up = bmp280_start_up;
    baro->get_up = bmp280_get_up;

    baro->calculate = bmp280_calculate;

    return true;
#else
		uint8_t databuf[25] = {0};
		int Coe_a0_;
		int Coe_a1_;
		int Coe_a2_;
		int Coe_b00_;
		int Coe_bt1_;
		int Coe_bt2_;
		int Coe_bp1_;
		int Coe_b11_;
		int Coe_bp2_;
		int Coe_b12_;
		int Coe_b21_;
		int Coe_bp3_;
		uint16_t lb=0,hb=0;
		uint32_t lw=0,hw=0,temp1,temp2;
		// SetIIR
		busWrite(baro->busDev, QMP6988_SET_IIR_REG, 0x05);
		//read OTP
		busReadBuf(baro->busDev, QMP6988_COE_B00_1_REG, databuf, 25);
		
		//algo OTP
		hw = databuf[0];
		lw =  databuf[1];
		temp1 = hw<<12 | lw<<4;
		
		hb = databuf[2];
		lb = databuf[3];
		Coe_bt1_ = hb<<8 | lb;
		
		hb = databuf[4];
		lb = databuf[5];
		Coe_bt2_ = hb<<8 | lb;
		
		hb = databuf[6];
		lb = databuf[7];
		Coe_bp1_ = hb<<8 | lb;
		
		hb = databuf[8];
		lb = databuf[9];
		Coe_b11_ = hb<<8 | lb;
		
		hb = databuf[10];
		lb = databuf[11];
		Coe_bp2_ = hb<<8 | lb;
		
		hb = databuf[12];
		lb = databuf[13];
		Coe_b12_ = hb<<8 | lb;
		
		hb = databuf[14];
		lb = databuf[15];
		Coe_b21_ = hb<<8 | lb;
		
		hb = databuf[16];
		lb = databuf[17];
		Coe_bp3_ = hb<<8 | lb;
		
		hw = databuf[18];
		lw = databuf[19];
		temp2 = hw<<12 | lw<<4;
		
		hb = databuf[20];
		lb = databuf[21];
		Coe_a1_ = hb<<8 | lb;
		
		hb = databuf[22];
		lb = databuf[23];
		Coe_a2_ = hb<<8 | lb;
		
		hb = databuf[24];
		
		temp1 = temp1|((hb&0xf0)>>4);
		if(temp1&0x80000)
			Coe_b00_ = ((int)temp1 - (int)0x100000);
		else
			Coe_b00_ = temp1;
		
		temp2 = temp2|(hb&0x0f);
		if(temp2&0x80000)
			Coe_a0_  = ((int)temp2 - (int)0x100000);
		else
			Coe_a0_ = temp2;
	
		qmp6988_cal.Coe_a0=(float)Coe_a0_/16.0;
		qmp6988_cal.Coe_a1=(-6.30E-03)+(4.30E-04)*(float)Coe_a1_/32767.0;
		qmp6988_cal.Coe_a2=(-1.9E-11)+(1.2E-10)*(float)Coe_a2_/32767.0;
		
		qmp6988_cal.Coe_b00 = Coe_b00_/16.0;
		qmp6988_cal.Coe_bt1 = (1.00E-01)+(9.10E-02)*(float)Coe_bt1_/32767.0;
		qmp6988_cal.Coe_bt2= (1.20E-08)+(1.20E-06)*(float)Coe_bt2_/32767.0;
		
		qmp6988_cal.Coe_bp1 = (3.30E-02)+(1.90E-02)*(float)Coe_bp1_/32767.0;
		qmp6988_cal.Coe_b11= (2.10E-07)+(1.40E-07)*(float)Coe_b11_/32767.0;
		
		qmp6988_cal.Coe_bp2 = (-6.30E-10)+(3.50E-10)*(float)Coe_bp2_/32767.0;
		qmp6988_cal.Coe_b12= (2.90E-13)+(7.60E-13)*(float)Coe_b12_/32767.0;
		
		qmp6988_cal.Coe_b21 = (2.10E-15)+(1.20E-14)*(float)Coe_b21_/32767.0;
		qmp6988_cal.Coe_bp3= (1.30E-16)+(7.90E-17)*(float)Coe_bp3_/32767.0;
	
		// Set power mode and sample times
		busWrite(baro->busDev, QMP6988_CTRL_MEAS_REG, QMP6988_PWR_SAMPLE_MODE);
		
	
		baro->ut_delay = 0;
		baro->get_ut = bmp280_get_ut;
		baro->start_ut = bmp280_start_ut;
	
		baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << QMP6988_TEMPERATURE_OSR) >> 1) + ((1 << QMP6988_PRESSURE_OSR) >> 1)) + (QMP6988_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
	
		baro->start_up = bmp280_start_up;
		baro->get_up = bmp280_get_up;
		
		baro->calculate = bmp280_calculate;
	
	return true;
#endif
}

#endif