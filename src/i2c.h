/*
 * i2c.h
 *
 *  Created on: Apr 12, 2019
 *      Author: srina
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

/*
 * Include Files
 */
#include "i2cspm.h"
#include "em_i2c.h"
#include <stdlib.h>
#include "log.h"


// Setting the i2c channel
#define i2c (I2C0)


void i2c_init();


void transfer(uint32_t *data);

#endif /* SRC_I2C_H_ */
