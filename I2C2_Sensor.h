/* 
 * File:   I2C2_Sensor.h
 * Author: kangl
 *
 * Created on April 19, 2024, 3:27 PM
 */

#ifndef I2C2_SENSOR_H
#define	I2C2_SENSOR_H
#include <xc.h>
#ifdef	__cplusplus
extern "C" {
#endif


void I2C_start();
void I2C_stop();
void I2C_write(uint8_t data);
uint32_t I2C_read();
void I2C_repeated_start();
void I2C_ACK();
void I2C_NACK();
void I2C_read_multiple_bytes_params(uint8_t reg, int numSamples);
uint8_t I2C_read_with_params(uint8_t reg);
uint8_t I2C_write_with_params(uint8_t reg, uint8_t val);
#ifdef	__cplusplus
}
#endif

#endif	/* I2C2_SENSOR_H */

