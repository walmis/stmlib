#ifndef __I2CBB_H
#define __I2CBB_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct i2cbb {
	uint32_t port;
	uint32_t scl;
	uint32_t sda;
};

//Function prototypes
void i2c_bb_setup(const struct i2cbb* i2c);
short i2c_bb_xfer(const struct i2cbb* i2c, unsigned char addr,
		const unsigned char *wr, size_t wn, uint8_t *r, size_t rn);


#ifdef __cplusplus
}
#endif



#endif
