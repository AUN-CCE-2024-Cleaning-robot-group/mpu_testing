/* 
 * File:   mpu9250_config.h
 * Author: Mahmoud
 *
 * Created on November 22, 2023, 3:19 PM
 */
#include "configure.h"
#include "MPU9250/mpu9250_interface.h"
#include "mpu9250_private.h"


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
       output_toggle(PIN_E2);
       mpu9250_read_gyro(gyro);
       mpu9250_calculate_angles(&Angle,accel);

       kalman_1d(KalmanAngleRoll,KalmanUncertaintyAngleRoll,gyro[0] - RateCalibration.roll,Angle.roll,Kalman1DOutput);

       KalmanAngleRoll=Kalman1DOutput[0];
       KalmanUncertaintyAngleRoll=Kalman1DOutput[1] ;
       kalman_1d(KalmanAnglePitch,KalmanUncertaintyAnglePitch,gyro[1] - RateCalibration.pitch ,Angle.pitch,Kalman1DOutput);

       KalmanAnglePitch=Kalman1DOutput[0];
       KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
       
       clear_interrupt(INT_TIMER2);
}



void main() {
    
    
    delay_ms(3000);
    //set_tris_c(0x80);                               // RC7 is RX input    RC6 is TX output
                        //Enable global interrupts
    //set_uart_speed(9600)     ;                      //Init the uart
    //delay_ms(4000);
    
    
    output_float(PIN_B1);                           // SCL
    output_float(PIN_B0);                           // SDA
    

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
    setup_timer_2(T2_DIV_BY_1, 255, 1);
    clear_interrupt(INT_TIMER2);
    enable_interrupts(INT_TIMER2);
    float mag[3] = {0,0,0};
    // Loop forever
     while (1)
     {
        mpu9250_read_magnometer(mag);
       //printf("magx = %f  magy = %f magz = %f  \n\r",  mag[0],mag[1],mag[2]);
        
       printf("Roll = %d  Pitch = %d  \n\r",  (int)KalmanAngleRoll,(int) KalmanAnglePitch);
       printf("Rollacc = %d  Pitchacc = %d  \n\r",  (int)(gyro[0] - RateCalibration.roll), (int)(gyro[1] - RateCalibration.pitch));
       
     }
}











