#ifndef HAL_I2S_H
#define HAL_I2S_H


#include "driver/i2s.h"
void hal_i2s_init(uint8_t i2s_num,uint8_t dout,uint8_t lrc, uint8_t bclk, uint8_t din,uint8_t ch);
int hal_i2s_read(uint8_t i2s_num,char* dest,size_t size,TickType_t timeout);
int hal_i2s_write(uint8_t i2s_num,char* dest,size_t size,TickType_t timeout);
#endif





