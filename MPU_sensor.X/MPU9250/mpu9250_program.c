#include "configure.h"
#include "mpu9250_config.h"
#include "mpu9250_private.h"
#include "mpu9250_interface.h"
//#include "../MyMath.h"
#include <stdlib.h>
#include <MATH.H>

// Function to write a byte to a specific register of a device
void mpu9250_writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
    i2c_start();
    i2c_write(deviceAddress << 1);
    i2c_write(registerAddress);
    i2c_write(data);
    i2c_stop();
}

// Function to read a byte from a specific register of a device
void mpu9250_readByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data) {
    i2c_start();
    i2c_write(deviceAddress << 1);
    i2c_write(registerAddress);
    i2c_start();
    i2c_write(deviceAddress << 1 | 0x01);
    *data = i2c_read(0);
    i2c_stop();
}

// Function to read a 16-bit value from two consecutive registers of a device
void read_word(uint8_t deviceAddress, uint8_t registerHigh, uint8_t registerLow, int16_t *data) {
    uint8_t highByte, lowByte;
    mpu9250_readByte(deviceAddress, registerLow, &lowByte);
    mpu9250_readByte(deviceAddress, registerHigh, &highByte);

        *data = make16(highByte,lowByte);

}

// Function to read a 16-bit value from two consecutive gyro registers
void read_gyro_word(uint8_t registerHigh, uint8_t registerLow, int16_t *data) {
    read_word(MPU9250_ADDRESS, registerHigh, registerLow, data);
}
// Function to read a 16-bit value from two consecutive accelerometer registers
void read_accel_word(uint8_t registerHigh, uint8_t registerLow, int16_t *data) {
    read_word(MPU9250_ADDRESS, registerHigh, registerLow, data);
}
// Function to read a 16-bit value from two consecutive magnetometer registers
void read_mag_word(uint8_t registerHigh, uint8_t registerLow, int16_t *data) {
    read_word(AK8963_ADDRESS, registerHigh, registerLow, data);
}

// Function to initialize MPU9250
void mpu9250_init() {

    mpu9250_writeByte(MPU9250_ADDRESS, CONFIG, CONFIG_VALUE);                    // Set the digital low pass filter to 41 Hz for gyro and 44 Hz for accel
    mpu9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_1, PWR_MGMT_1_VALUE);            // Set the clock source to PLL with X axis gyroscope reference and disable sleep mode
    mpu9250_writeByte(MPU9250_ADDRESS, PWR_MGMT_2, PWR_MGMT_2_VALUE);            // Enable all sensors and disable standby mode for accel and gyro only
    mpu9250_writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, AFS_8G);                    // Set the accelerometer range to ±8 g
    mpu9250_writeByte(MPU9250_ADDRESS, GYRO_CONFIG, GFS_500);                    // Set the gyro range to ±500 dps


    mpu9250_writeByte(MPU9250_ADDRESS, INT_PIN_CFG, INT_PIN_CFG_VALUE1);         // Enable the I2C bypass mode of the MPU-9250
    mpu9250_writeByte(MPU9250_ADDRESS,MPU9250_I2C_MST_CTRL, I2C_MST_CLK_400kHz); // Set bits 3:0 to 1101 for 400kHz I2C master clock speed

    mpu9250_writeByte(AK8963_ADDRESS, AK8963_CNTL2_Reg , 0x01);                  // Reset the AK8963 magnetometer
    mpu9250_writeByte(AK8963_ADDRESS, AK8963_CNTL1_Reg, 0x12);                   // Set the magnetometer to 16 bit resolution, 100 Hz update rate

    delay_ms(100);

    uint8_t status = 0;
    mpu9250_readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250, &status);
    if (status == MPU9250_ADDRESS_ID) {
        printf( "MPU9250 is online...\n\r" );
    } else {
        printf( "MPU9250 is not online...\n\r" );
    }
    mpu9250_readByte(AK8963_ADDRESS, WHO_AM_I_AK8963, &status);
    if (status == AK8963_ADDRESS_ID) {
        printf( "AK8963 is online...\n\r" );
    } else {
        printf( "AK8963 is not online...\n\r" );
    }

}

// Function to read and print the initial register values of the MPU9250
void mpu9250_print_initRegisters(void) {
    uint8_t data = 0;
    mpu9250_readByte(MPU9250_ADDRESS, CONFIG, &data);
    printf("0x03->> CONFIG: %x\n\r", data);
    mpu9250_readByte(MPU9250_ADDRESS, PWR_MGMT_1, &data);
    printf("0x01->> PWR_MGMT_1: %x\n\r", data);
    mpu9250_readByte(MPU9250_ADDRESS, PWR_MGMT_2, &data);
    printf("0x00->> PWR_MGMT_2: %x\n\r", data);
    mpu9250_readByte(MPU9250_ADDRESS, INT_PIN_CFG, &data);
    printf("0x02->> INT_PIN_CFG: %x\n\r", data);
    mpu9250_readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250, &data);
    printf("0x071->> WHO_AM_I_MPU9250: %x\n\r", data);
    mpu9250_readByte(AK8963_ADDRESS, WHO_AM_I_AK8963, &data);
    printf("048->>  WHO_AM_I_AK8963: %x\n\r", data);
}

// Function to read gyro data and convert it to degrees per second
void mpu9250_read_gyro(float gyroData[]) {
    int16_t gyroX, gyroY, gyroZ = 0;
    read_gyro_word(GYRO_XOUT_H, GYRO_XOUT_L, &gyroX);
    read_gyro_word(GYRO_YOUT_H, GYRO_YOUT_L, &gyroY);
    read_gyro_word(GYRO_ZOUT_H, GYRO_ZOUT_L, &gyroZ);
    gyroData[0] = gyroX / GYRO_SCALE;
    gyroData[1] = gyroY / GYRO_SCALE;
    gyroData[2] = gyroZ / GYRO_SCALE;
}

// Function to read accelerometer data and convert it to g
void mpu9250_read_accelerometer(float accelData[] ) {
    int16_t accelX, accelY, accelZ = 0;
    read_accel_word(ACCEL_XOUT_H, ACCEL_XOUT_L, &accelX);
    read_accel_word(ACCEL_YOUT_H, ACCEL_YOUT_L, &accelY);
    read_accel_word(ACCEL_ZOUT_H, ACCEL_ZOUT_L, &accelZ);
    accelData[0] = (accelX / ACCEL_SCALE) +ACCEL_X_OFFSET;
    accelData[1] = (accelY / ACCEL_SCALE) -ACCEL_Y_OFFSET;
    accelData[2] = (accelZ / ACCEL_SCALE) +ACCEL_Z_OFFSET;
}

//Function to read magnetometer data and convert it to uT
void mpu9250_read_magnometer(float magData[]) {

    int16_t magX, magY, magZ = 0;
    uint8_t x_axis , y_axis , z_axis ;
    uint8_t status = 0xF0;

    mpu9250_readByte( MPU9250_ADDRESS,AK8963_ASAX , &x_axis );
    mpu9250_readByte( MPU9250_ADDRESS,AK8963_ASAY , &y_axis );
    mpu9250_readByte( MPU9250_ADDRESS,AK8963_ASAZ , &z_axis );

    
    
    // Check if the data is ready polling ST1
    while(status & 0x01  == 0x01)
    {
        mpu9250_readByte(AK8963_ADDRESS, AK8963_ST1_REG, &status);
    
    }
    
    read_mag_word(MAG_XOUT_H, MAG_XOUT_L, &magX);
        read_mag_word(MAG_YOUT_H, MAG_YOUT_L, &magY);
        read_mag_word(MAG_ZOUT_H, MAG_ZOUT_L, &magZ);
        // Convert the magnetometer data to uT
        magData[0] = magX *((x_axis-128)*0.5 /128 +1);
        magData[1] = magY *((y_axis-128)*0.5 /128 +1) ;
        magData[2] =  magZ *((z_axis-128)*0.5 /128 +1) ;
    mpu9250_readByte(AK8963_ADDRESS, AK8963_ST2_REG, &status);
    
    //mpu9250_writeByte(AK8963_ADDRESS, AK8963_CNTL1_Reg, 0x11);                   // Set the magnetometer to 16 bit resolution, 100 Hz update rate

//    if (status==0x01) {
//        // Read the magnetometer data
//        read_mag_word(MAG_XOUT_H, MAG_XOUT_L, &magX);
//        read_mag_word(MAG_YOUT_H, MAG_YOUT_L, &magY);
//        read_mag_word(MAG_ZOUT_H, MAG_ZOUT_L, &magZ);
//        // Convert the magnetometer data to uT
//        magData[0] = magX / MAG_SCALE;
//        magData[1] = magY / MAG_SCALE;
//        magData[2] = magZ / MAG_SCALE;
//    }
//    else {
//        // Handle the error and notify the user
//        printf( "Magnetometer data is not ready! -> %d \n\r" , status);
//        
//        // Read the magnetometer data
//        read_mag_word(MAG_XOUT_H, MAG_XOUT_L, &magX);
//        read_mag_word(MAG_YOUT_H, MAG_YOUT_L, &magY);
//        read_mag_word(MAG_ZOUT_H, MAG_ZOUT_L, &magZ);
//        // Convert the magnetometer data to uT
//        magData[0] = magX / MAG_SCALE;
//        magData[1] = magY / MAG_SCALE;
//        magData[2] = magZ / MAG_SCALE;
//    }

}

void mpu9250_print_module_values() {

    // Read the gyro values
    mpu9250_read_gyro(gyro);
    mpu9250_read_magnometer(mag);
    mpu9250_read_accelerometer(accel);

    // Read the mag values
    printf(ANSI_COLOR_YELLOW"Gyro : X = %.5f, Y = %.5f, Z = %.5f (deg/s)   \t"ANSI_COLOR_RESET, gyro[0], gyro[1], gyro[2]);
    printf(ANSI_COLOR_YELLOW"Mag  : X = %.5f, Y = %.5f, Z = %.5f (uT)      \t"ANSI_COLOR_RESET,  mag[0],  mag[1],  mag[2]);
    printf(ANSI_COLOR_YELLOW"Accel: X = %.5f, Y = %.5f, Z = %.5f (g)     \n\r"ANSI_COLOR_RESET,accel[0],accel[1],accel[2]);
    
}



// Function to calibrate the accelerometer
void mpu9250_gyro_calibration( RateCalibration_t *RateCalibration )
{

    for (uint16_t RateCalibrationNumber=0; RateCalibrationNumber<4000; RateCalibrationNumber ++) {
        mpu9250_read_gyro(gyro);
        RateCalibration->roll += gyro[0];     //calculate average value of gyro in x axis
        RateCalibration->pitch  += gyro[1];     //calculate average value of gyro in y axis
        RateCalibration->yaw   += gyro[2];     //calculate average value of gyro in z axis

        delay_ms(1) ;
    }
    RateCalibration->roll /= 4000;     //calculate average value of gyro in x axis
    RateCalibration->pitch  /= 4000;     //calculate average value of gyro in y axis
    RateCalibration->yaw   /= 4000;     //calculate average value of gyro in z axis


}


// Function to calculate the yaw, pitch and roll angles
void mpu9250_calculate_angles(Angle_t *Angle ,float accelData[])
{

        // Read the accelerometer and magnetometer values
        mpu9250_read_accelerometer(accelData);

        // Calculate the roll angle
        Angle->roll = atan( accelData[1] / sqrt(  (float)(accelData[0] * accelData[0] + accelData[2] * accelData[2]) )) * 180.0 / PI;
        // Calculate the pitch angle
        Angle->pitch = atan( accelData[0] / sqrt( (float) (accelData[1] * accelData[1] + accelData[2] * accelData[2]) )) * (-180.0) / PI;
//Angle->roll = atan(0.1);Angle->pitch= 1;

}


void kalman_1d(float KalmanState,float KalmanUncertainty, float KalmanInput,float KalmanMeasurement,float Kalman1DOutput[])
{
  KalmanState=KalmanState+0.0192*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.0192 * 0.0192 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0]=KalmanState;
    Kalman1DOutput[1]=KalmanUncertainty;
}

// Calculate the roll angle


// Calculate the pitch angle

