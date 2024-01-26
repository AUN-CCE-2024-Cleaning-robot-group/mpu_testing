/* 
 * File:   mpu9250_private.h
 * Author: Mahmoud
 *
 * Created on November 22, 2023, 3:18 PM
 */

#ifndef MPU9250_PRIVATE_H
#define	MPU9250_PRIVATE_H

#ifdef	__cplusplus
extern "C" {
#endif
//Sensitivity= Full Scale Range/Digital Range
#define WHO_AM_I_MPU9250    0x75
#define WHO_AM_I_AK8963     0x00
#define MPU9250_ADDRESS_ID 0x71
#define AK8963_ADDRESS_ID  0x48

// Sensitivity for GYROSCOPE
#define GYRO_SCALE_250      131.0
#define GYRO_SCALE_500      65.5
#define GYRO_SCALE_1000     32.8
#define GYRO_SCALE_2000     16.4
#define GYRO_SCALE          GYRO_SCALE_500

// Sensitivity for ACCELEROMETER
#define ACCEL_SCALE_2G      16384.0
#define ACCEL_SCALE_4G      8192.0
#define ACCEL_SCALE_8G      4096.0
#define ACCEL_SCALE_16G     2048.0
#define ACCEL_SCALE         ACCEL_SCALE_8G


    // Resolution for MAGNETOMETER
#define AK8963_CNTL1_RESOLUTION_14BIT   0x00
#define AK8963_CNTL1_RESOLUTION_16BIT   0x10
#define AK8963_CNTL1_RESOLUTION         AK8963_CNTL1_RESOLUTION_16BIT
// MPU9250 I2C address pin is connected to GND
#define MPU9250_ADDRESS 0x68


// Define the registers for configuration and power management
#define CONFIG          0x1A
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C

// Define the values for configuration and power management
#define CONFIG_VALUE     0x03 // set the digital low pass filter to 41 Hz for gyro and 44 Hz for accel
#define PWR_MGMT_1_VALUE 0x01 // set the clock source to PLL with X axis gyroscope reference
#define PWR_MGMT_2_VALUE 0x00 // enable all sensors

// The register address of the interrupt pin configuration
#define INT_PIN_CFG         0x37
// The value to enable the bypass mode of the MPU9250
#define INT_PIN_CFG_VALUE2    0x30 // enable the I2C bypass mode of the MPU-9250
#define INT_PIN_CFG_VALUE1    0x02 // enable the interrupt active high

#define AK8963_ADDRESS      0x0C    // The address of the AK8963 device on the I2C bus
#define AK8963_ST1_REG      0x02    // Define the registers for magnetometer data
#define AK8963_ST2_REG      0x09    // Define the registers for magnetometer data

#define AK8963_CNTL1_Reg    0x0A    // Control 1 register of AK8963
#define AK8963_CNTL2_Reg    0x0B    // Control 2 register of AK8963


// Define the MPU9250 address and the register addresses
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration   used to set the accelerometer range
#define GYRO_CONFIG  0x1B // Gyroscope configuration        used to set the gyro range



    // Define the Resolution of the magnetometer
#define MFS_14BITS   0 // 0.6 mG per LSB
#define MFS_16BITS   1 // 0.15 mG per LSB



// Define the values for the accelerometer and gyro configuration
#define AFS_2G          0x00 // ±2 g
#define AFS_4G          0x08 // ±4 g
#define AFS_8G          0x10 // ±8 g
#define AFS_16G         0x18 // ±16 g

// Define the values for the gyro configuration
#define GFS_250         0x00 // ±250 dps
#define GFS_500         0x08 // ±500 dps
#define GFS_1000        0x10 // ±1000 dps
#define GFS_2000        0x18 // ±2000 dps

// Define the registers for gyroscope data
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

// Define the registers for magnometer data
#define MAG_XOUT_L      0x03
#define MAG_XOUT_H      0x04
#define MAG_YOUT_L      0x05
#define MAG_YOUT_H      0x06
#define MAG_ZOUT_L      0x07
#define MAG_ZOUT_H      0x08

// Define the registers for accelerometer data
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

#define MPU9250_SMPLRT_DIV 0x19


#define MPU9250_USER_CTRL       0x6A
#define MPU9250_I2C_MST_CTRL    0x24
#define MPU9250_I2C_SLV0_CTRL   0x27

#define USER_CTRL_I2C_MST_EN    0x20  // Bit 5 to enable the I2C master mode
#define I2C_MST_CLK_400kHz      0x0D  // which enables the I2C master mode and sets the I2C speed to 400 kHz. TThis allows the sensor to control the magnetometer as a slave device
#define I2C_MST_DELAY_CTRL      0x67  // This register allows the user to delay the data ready interrupt signal, which is useful when using the magnetometer data as a clock reference for the gyroscope and accelerometer data
#define AK8963_MAG_MODE_FUSEROM 0x0F // Bits 3:0 for 400kHz I2C master clock speed

// Define the registers for magnetometer sensitivity adjustment
#define AK8963_ASAX             0x10 // X-axis sensitivity adjustment value
#define AK8963_ASAY             0x11 // Y-axis sensitivity adjustment value
#define AK8963_ASAZ             0x12 // Z-axis sensitivity adjustment value

#define  AK8963_MAG_OUTPUT_16BIT            0x16
#define  AK8963_MAG_OUTPUT_14BIT            0x12
#define AK8963_MAG_MODE_CONTINUOUS_100HZ    0x06

    // Define the sensitivity adjustment values for the magnetometer
#define  AK8963_MAG_SCALE_14BIT             4912.0f/8190.0f  // 4912 uT for 14-bit resolution
#define  AK8963_MAG_SCALE_16BIT             4912.0f/32760.0f // 4912 uT for 16-bit resolution
#define  MAG_SCALE                          AK8963_MAG_SCALE_16BIT

// Accelerometer offsets values for calibration
#define ACCEL_X_OFFSET 0.005
#define ACCEL_Y_OFFSET 0.0144
#define ACCEL_Z_OFFSET 0.06006

#ifdef	__cplusplus
}
#endif

#endif	/* MPU9250_PRIVATE_H */

