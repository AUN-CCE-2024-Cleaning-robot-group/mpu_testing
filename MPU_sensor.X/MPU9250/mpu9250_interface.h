/* 
 * File:   mpu9250_interface.h
 * Author: Mahmoud
 *
 * Created on November 22, 2023, 3:17 PM
 */

#ifndef MPU9250_INTERFACE_H
#define	MPU9250_INTERFACE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include"configure.h"
#include "mpu9250_config.h"
#include "mpu9250_private.h"
#include "mpu9250_interface.h"

    //define array of 3 elements to store the gyro values
    extern float gyro[3];
    //define array of 3 elements to store the mag values
    extern  float mag[3];
    //define array of 3 elements to store the accel values
    extern float accel[3];

    struct PitchRollYaw_t {
        float pitch;
        float roll;
        float yaw;
    } ;
typedef struct PitchRollYaw_t RateCalibration_t ;
typedef struct PitchRollYaw_t Angle_t ;


void mpu9250_writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) ;
void mpu9250_readByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data) ;
void read_word(uint8_t deviceAddress, uint8_t registerHigh, uint8_t registerLow, int16_t *data) ;
void read_gyro_word (uint8_t registerHigh,uint8_t registerLow, int16_t *data) ;
void read_accel_word(uint8_t registerHigh,uint8_t registerLow, int16_t *data) ;
void read_mag_word  (uint8_t registerHigh,uint8_t registerLow, int16_t *data) ;
void mpu9250_mag_init() ;
void mpu9250_init() ;
void mpu9250_print_initRegisters(void) ;
void mpu9250_read_gyro(float gyroData[]) ;
void mpu9250_read_magnometer(float magData[]) ;
void mpu9250_read_accelerometer(float accelData[]) ;

void mpu9250_gyro_calibration(  RateCalibration_t *RateCalibration ) ;

void mpu9250_print_module_values(void) ;

void mpu9250_calculate_angles(Angle_t *Angle ,float accelData[]);

void kalman_1d(float KalmanState,float KalmanUncertainty, float KalmanInput,float KalmanMeasurement,float Kalman1DOutput[]);

#ifdef	__cplusplus
}
#endif

#endif	/* MPU9250_INTERFACE_H */

