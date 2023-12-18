/* 
 * File:   mpu9250_interface.h
 * Author: Mahmoud
 *
 * Created on November 22, 2023, 3:17 PM
 */

#ifndef CONFIGURE_H
#define	CONFIGURE_H


#include <18F4550.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>




#fuses NOMCLR HS CPUDIV1 NOPBADEN//INTRC_IO
#use delay(clock = 8000000)
/*
 * Important notice: See clock diagram page 26 of the data sheet, and set PLLx & CPUDIVx
 * accodingly. Such that F_CPU = F_USB = .
 */


#use rs232(uart1, baud = 9600)                // Initialize UART module
#use i2c(MASTER, SDA=PIN_B0, SCL=PIN_B1,SMBUS,FAST = 400000,FORCE_HW)   // Setup I2C communication


typedef void (*CallbackPtr_t)(void*);

// ANSI escape codes for colors
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#endif	/* CONFIGURE_H */