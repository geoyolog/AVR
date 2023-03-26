/*
 * i2c_twi.h
 *
 *  Created on: 2010-09-07
 *       Autor: Miros³aw Kardaœ
 */

#ifndef I2C_TWI_H_
#define I2C_TWI_H_

#define ACK 1
#define NACK 0

// funkcje

void i2cSetBitrate(uint16_t bitrateKHz);

void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8_t bajt);
uint8_t TWI_read(uint8_t ack);

void TWI_write_buf( uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf);
void TWI_read_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf);
void TWI_write_int( uint8_t SLA, uint8_t data);
uint8_t TWI_read_int(uint8_t SLA);

uint8_t PCF8574_read( uint8_t SLA );

#endif /* I2C_TWI_H_ */

