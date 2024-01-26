/* 
 * File:   mpu9250_config.h
 * Author: Mahmoud
 *
 * Created on November 22, 2023, 3:19 PM
 */

#include "configure.h"

#include "mpu9250_interface.h"
#include "mpu9250_private.h"
#include"SSD1780OLED_interface.h"




static void imu_update(void);
// Declare variables to store the sensor values
float gyro[3] = {0.0, 0.0, 0.0} ;
float mag[3] = {0.0, 0.0, 0.0}   ;
float accel[3] = {0.0, 0.0, 0.0}    ;

RateCalibration_t RateCalibration = {0.0, 0.0, 0.0} ;
Angle_t Angle = {0.0, 0.0, 0.0} ;

float KalmanAngleRoll=0 ;
int KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0  ;
int KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};       //angle predict, angle uncertainty predict

#INT_TIMER2
static void imu_update(void)
{
       mpu9250_read_gyro(gyro);
       mpu9250_calculate_angles(&Angle,accel,mag);

       kalman_1d(KalmanAngleRoll,KalmanUncertaintyAngleRoll,gyro[0] - RateCalibration.roll,Angle.roll,Kalman1DOutput);

       KalmanAngleRoll=Kalman1DOutput[0];
       KalmanUncertaintyAngleRoll=Kalman1DOutput[1] ;
       kalman_1d(KalmanAnglePitch,KalmanUncertaintyAnglePitch,gyro[1] - RateCalibration.pitch ,Angle.pitch,Kalman1DOutput);

       KalmanAnglePitch=Kalman1DOutput[0];
       KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
       
       clear_interrupt(INT_TIMER2);
}


void i2c_reset()
{

output_high(PIN_B0);
for (int i = 0; i < 10; i++) 
    { //9nth cycle acts as NACK
        output_high(PIN_B1);
        delay_us(5);
         output_low(PIN_B1);
        delay_us(5);
    }
  output_low(PIN_B0);
  delay_us(5);
  output_high(PIN_B1);
  delay_us(5);
  output_high(PIN_B0);
  delay_us(5);
  
  output_float(PIN_B1);                           // SCL
  output_float(PIN_B0);                           // SDA
  i2c_init(1);

}


void main() {
    
    
    
  output_float(PIN_B1);                           // SCL
  output_float(PIN_B0);                           // SDA

    delay_ms(500);

    printf(ANSI_COLOR_YELLOW"start of init  \n\r"ANSI_COLOR_RESET );
    
    mpu9250_init();                                 // Init the MPU9250

    mpu9250_print_initRegisters() ;                  // Print the registers of the MPU9250
    printf("Calibration of Gyro is started don't move the sensor\n\r");
    delay_ms(2000);
    mpu9250_gyro_calibration(&RateCalibration);     // Calibrate the gyro
    printf("Calibration of Gyro is finished\n\r");
    printf("RateCalibrationPitch = %.5f , RateCalibrationRoll = %.5f , RateCalibrationYaw = %.5f \n\r",RateCalibration.pitch,RateCalibration.roll,RateCalibration.yaw);


   // Declare a variable to store the loop counter

    enable_interrupts(GLOBAL);
    setup_timer_2(T2_DIV_BY_16, 255, 1);
    clear_interrupt(INT_TIMER2);
    enable_interrupts(INT_TIMER2);
    // Loop forever
     while (1)
     {
      printf("yaw= %d  Pitch = %d  \n\r",  (int)Angle.yaw,(int) KalmanAnglePitch);
       
     }
}











