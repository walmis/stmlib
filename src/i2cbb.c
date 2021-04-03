#include "i2cbb.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/cortex.h>

#define SDA_LOW() GPIO_BRR(i2c->port) = i2c->sda
#define SDA_HIGH() GPIO_BSRR(i2c->port) = i2c->sda

#define SCL_LOW() GPIO_BRR(i2c->port) = i2c->scl
#define SCL_HIGH() GPIO_BSRR(i2c->port) = i2c->scl

#define SDA_READ() (GPIO_IDR(i2c->port) & i2c->sda)

//wait for 1/2 of I2C clock period
static inline void i2c_bb_hc(void){
 uint32_t s = DWT_CYCCNT;
 while(DWT_CYCCNT - s < (72/5)) {}
}

//setup bit bang I2C pins
void i2c_bb_setup(const struct i2cbb* i2c){
	SCL_HIGH();

	SDA_HIGH();
#ifdef STM32F1
	gpio_set_mode(i2c->port, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
			  i2c->scl);

	gpio_set_mode(i2c->port, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_OPENDRAIN,
		      i2c->sda);
#elif defined(STM32L4)
	gpio_mode_setup(i2c->port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, i2c->scl);
  gpio_mode_setup(i2c->port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, i2c->sda);
  gpio_set_output_options(i2c->port, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, i2c->sda);
#else
#error "no impl"
#endif
    //if sda is low on startup, try clocking scl to reset the device
	if(SDA_READ() == 0) {
		for(int i = 0; i < 8; i++) {
			SCL_LOW();
			i2c_bb_hc();
			SCL_HIGH();
			i2c_bb_hc();
		}
	}
}



static void i2c_bb_start(const struct i2cbb* i2c){
  i2c_bb_hc();

  SCL_HIGH();
  //wait for 1/2 clock first
  i2c_bb_hc();
  //pull SDA low
  SDA_LOW();
  //wait for 1/2 clock for end of start
  i2c_bb_hc();
}

static void i2c_bb_stop(const struct i2cbb* i2c){
  //pull SDA low
  SDA_LOW();
  //wait for 1/2 clock for end of start
  i2c_bb_hc();
  //float SCL
  SCL_HIGH();
  //wait for 1/2 clock
  i2c_bb_hc();
  //float SDA
  SDA_HIGH();
  //wait for 1/2 clock
  i2c_bb_hc();

}

//send value over I2C return 1 if slave ACKed
static short i2c_bb_tx_byte(const struct i2cbb* i2c, unsigned char val){
  int i;
  cm_disable_interrupts();
  //shift out bits
  for(i=0;i<8;i++){
    //pull SCL low
    SCL_LOW();
    //check bit
    if(val&0x80){
      //float SDA
      SDA_HIGH();
    }else{
      //pull SDA low
      SDA_LOW();
    }
    //shift
    val<<=1;
    //wait for 1/2 clock
    i2c_bb_hc();
    //float SCL
    SCL_HIGH();
    //wait for 1/2 clock
    i2c_bb_hc();
  }
  //check ack bit
  //pull SCL low
  SCL_LOW();
  //float SDA
  SDA_HIGH();
  //wait for 1/2 clock
  i2c_bb_hc();
  //float SCL
  SCL_HIGH();
  //wait for 1/2 clock
  i2c_bb_hc();
  //sample SDA
  val= SDA_READ();
  //pull SCL low
  SCL_LOW();
  //return sampled value
  cm_enable_interrupts();
  return val == 0;
}
//send value over I2C return 1 if slave ACKed
unsigned char i2c_bb_rx_byte(const struct i2cbb* i2c, unsigned short ack){
  unsigned char val;
  int i;
  cm_disable_interrupts();
  //shift out bits
  for(i=0;i<8;i++){
    //pull SCL low
    SCL_LOW();
    //wait for 1/2 clock
    i2c_bb_hc();
    //float SCL
    SCL_HIGH();
    //wait for 1/2 clock
    //i2c_bb_hc();

    #pragma GCC unroll 30
    for(int i = 0; i < 4; i++) {
    	asm("nop");
    }

    //shift value to make room
    val<<=1;
    //sample data
    if(SDA_READ()){
      val|=1;
    }

    //pull SCL low
    SCL_LOW();


  }
  //check ack bit
  //pull SCL low
  SCL_LOW();
  //check if we are ACKing this byte
  if(ack){
    //pull SDA low for ACK
    SDA_LOW();
  }else{
    //float SDA for NACK
    SDA_HIGH();
  }
  //wait for 1/2 clock
  i2c_bb_hc();
  //float SCL
  SCL_HIGH();
  //wait for 1/2 clock
  i2c_bb_hc();
  //pull SCL low
  SCL_LOW();
  //float SDA
  SDA_HIGH();

  cm_enable_interrupts();
  //return value
  return val;
}

short i2c_bb_xfer(const struct i2cbb* i2c, unsigned char addr,
		const unsigned char *wr, size_t wn, uint8_t *r, size_t rn){
  short ack = 0;
  int i;

  if(SDA_READ() == 0) {
	  return 0;
  }

  if(wn) {
	  //send start
	  i2c_bb_start(i2c);
	  //send address with W bit
	  ack=i2c_bb_tx_byte(i2c, (addr<<1));
	  if(!ack) {
		  goto err;
	  }
	  //send data bytes
	  for(i=0;i<wn && ack;i++){
		//transmit next byte
		ack=i2c_bb_tx_byte(i2c, wr[i]);
	  }
  }

  if(rn) {
	  //send start
	  i2c_bb_start(i2c);
	  //send address with R bit
	  if(!i2c_bb_tx_byte(i2c, (addr<<1)|1)){
	    //got NACK return error
		  goto err;
	  }
	  //send data bytes
	  for(i=0;i<rn;i++){
	    //transmit next byte
	    r[i]=i2c_bb_rx_byte(i2c, i!=rn-1);
	  }
  }
err:
  i2c_bb_stop(i2c);
  return ack;
}

short i2c_bb_rx(const struct i2cbb* i2c, unsigned char addr,unsigned char *dest,unsigned short len){
  int i;
  //send start
  i2c_bb_start(i2c);
  //send address with R bit
  if(!i2c_bb_tx_byte(i2c, (addr<<1)|BIT0)){
    //got NACK return error
    return 0;
  }
  //send data bytes
  for(i=0;i<len;i++){
    //transmit next byte
    dest[i]=i2c_bb_rx_byte(i2c, i==len-1);
  }
  //transmit stop
  i2c_bb_stop(i2c);
  //return if slave NACKed
  return 1;
}
