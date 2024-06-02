#ifndef LIS3DHTR_H_
#define LIS3DHTR_H_

// LIS3DHTR I2C address is 0x18 (default) or 0x19 (if SDO pin is high)
#define LIS3DHTR_ADDRESS 0x19

// Control register addresses
#define LIS3DHTR_CTRL_REG1 0x20
#define LIS3DHTR_CTRL_REG4 0x23

// Acceleration data registers
#define LIS3DHTR_OUT_X_L 0x28

#endif