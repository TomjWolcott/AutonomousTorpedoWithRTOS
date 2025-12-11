/*
 * registers.h
 *
 *  Created on: Jul 25, 2025
 *      Author: tomwolcott
 */

#ifndef AK09940A_REGISTERS_H_
#define AK09940A_REGISTERS_H_

// Company and Device ID
#define AK09940A_REG_WIA1       0x00
#define AK09940A_REG_WIA2       0x01

// Reserved
#define AK09940A_REG_RSV1       0x02
#define AK09940A_REG_RSV2       0x03

// Status
#define AK09940A_REG_ST         0x0F
#define AK09940A_REG_ST1        0x10

// Magnetic Data - X Axis
#define AK09940A_REG_HXL        0x11
#define AK09940A_REG_HXM        0x12
#define AK09940A_REG_HXH        0x13

// Magnetic Data - Y Axis
#define AK09940A_REG_HYL        0x14
#define AK09940A_REG_HYM        0x15
#define AK09940A_REG_HYH        0x16

// Magnetic Data - Z Axis
#define AK09940A_REG_HZL        0x17
#define AK09940A_REG_HZM        0x18
#define AK09940A_REG_HZH        0x19

// Temperature Data
#define AK09940A_REG_TMPS       0x1A

// Additional Status
#define AK09940A_REG_ST2        0x1B

// Self Test Data - X Axis
#define AK09940A_REG_SXL        0x20
#define AK09940A_REG_SXH        0x21

// Self Test Data - Y Axis
#define AK09940A_REG_SYL        0x22
#define AK09940A_REG_SYH        0x23

// Self Test Data - Z Axis
#define AK09940A_REG_SZL        0x24
#define AK09940A_REG_SZH        0x25

// Control Registers
#define AK09940A_REG_CNTL1      0x30
#define AK09940A_REG_CNTL2      0x31
#define AK09940A_REG_CNTL3      0x32
#define AK09940A_REG_CNTL4      0x33

// I2C Disable
#define AK09940A_REG_I2CDIS     0x36

// Test Register (Do Not Access)
#define AK09940A_REG_TS         0x37

#endif /* AK09940A_REGISTERS_H_ */
