/*
 * i2c.c
 *
 *  Created on: Apr 11, 2019
 *      Author: srina
 */
#include "i2c.h"

void i2c_init()
{
	I2CSPM_Init_TypeDef I2CSPMinit = I2CSPM_INIT_DEFAULT;
	I2CSPM_Init(&I2CSPMinit);
}

void transfer(uint32_t *data)
{
	I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_read_data[2];
	uint8_t i2c_write_data[1];

	seq.addr = 0x57<<1;
	seq.flags = I2C_FLAG_WRITE;
	i2c_write_data[0] = 0xFF;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 1;

	ret = I2CSPM_Transfer(i2c, &seq);

	LOG_INFO("\n the return value is : %d", ret);

	while( ret != 0)
	{
		ret = I2CSPM_Transfer(i2c, &seq);
		//LOG_ERROR("Error in transfer loop");
	}
	if(ret == -1)
	{
		LOG_ERROR("Error in transfer");
	}

	seq.addr = 0x57<<1;
	seq.flags = I2C_FLAG_READ;
	i2c_read_data[0] = 0xFF;
	seq.buf[0].data = i2c_read_data;
	seq.buf[0].len = 2;

	do{
		ret = I2CSPM_Transfer(i2c, &seq);

	} while (ret == -1);

	// setting the data after receiving the from I2CSPM_Transfer function
	if(ret != i2cTransferDone)
	{
		*data = 0;
	}
	else
	{
		*data = ((uint32_t)i2c_read_data[0]);
	}


}
