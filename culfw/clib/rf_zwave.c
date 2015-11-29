#include "board.h"
#ifdef HAS_ZWAVE
#include <string.h>
#include <avr/pgmspace.h>
#include "cc1100.h"
#include "delay.h"
#include "display.h"

#include "rf_zwave.h"

uint8_t zwave_on = 0;
#define MAX_ZWAVE_MSG 60

const uint8_t PROGMEM ZWAVE_CFG[] = {
  CC1100_IOCFG2,    0x01, // 00 GDO2 pin config:               00:FIFOTHR or end
  CC1100_IOCFG0,    0x2e, // 02 GDO0 pin config:                  2e:three state
  CC1100_FIFOTHR,   0x01, // 03 FIFO Threshhold                  01:RX:8, TX:61
  CC1100_SYNC1,     0xaa, // 04 Sync word, high byte
  CC1100_SYNC0,     0x0f, // 05 Sync word, low byte
  CC1100_PKTLEN,    0xff, // 06 Packet length
  CC1100_PKTCTRL1,  0x00, // 07 Packet automation control         00:no crc/addr
  CC1100_PKTCTRL0,  0x00, // 08 Packet automation control         00:fixlen,fifo
  CC1100_FSCTRL1,   0x06, // 0B Frequency synthesizer control
  CC1100_FREQ2,     0x21, // 0D Frequency control word, high byte       868.4MHz
  CC1100_FREQ1,     0x66, // 0E Frequency control word, middle byte
  CC1100_FREQ0,     0x66, // 0F Frequency control word, low byte
  CC1100_MDMCFG4,   0xca, // 10 Modem configuration                  bW 101.5kHz
  CC1100_MDMCFG3,   0x93, // 11 Modem configuration                     dr 40kHz
  CC1100_MDMCFG2,   0x06, // 12 Modem configuration                 2-FSK/16sync
  CC1100_MDMCFG1,   0x72, // 13 Modem configuration                  24 preamble
  CC1100_DEVIATN,   0x35, // 15 Modem deviation setting                dev:4394Hz
  CC1100_MCSM0,     0x18, // 18 Main Radio Cntrl State Machine config
  CC1100_FOCCFG,    0x16, // 19 Frequency Offset Compensation config
  CC1100_AGCCTRL2,  0x03, // 1B AGC control
  CC1100_FSCAL3,    0xe9, // 23 Frequency synthesizer calibration
  CC1100_FSCAL2,    0x2a, // 24 Frequency synthesizer calibration
  CC1100_FSCAL1,    0x00, // 25 Frequency synthesizer calibration
  CC1100_FSCAL0,    0x1f, // 26 Frequency synthesizer calibration
  CC1100_TEST2,     0x81, // 2C Various test settings
  CC1100_TEST1,     0x35, // 2D Various test settings
  CC1100_TEST0,     0x09, // 2E Various test settings
  // CC1100_LQI     0x7f, // 33 LingQuality Estimate - READ ONLY
  CC1100_PATABLE,   0x50  // 3E
};


void
rf_zwave_init(void)
{

  EIMSK &= ~_BV(CC1100_INT);                 // disable INT - we'll poll...
  SET_BIT( CC1100_CS_DDR, CC1100_CS_PIN );

  CC1100_DEASSERT;                           // Toggle chip select signal
  my_delay_us(30);
  CC1100_ASSERT;
  my_delay_us(30);
  CC1100_DEASSERT;
  my_delay_us(45);

  ccStrobe( CC1100_SRES );
  my_delay_us(100);

  for (uint8_t i = 0; i < sizeof(ZWAVE_CFG); i += 2) {
    cc1100_writeReg( pgm_read_byte(&ZWAVE_CFG[i]),
                     pgm_read_byte(&ZWAVE_CFG[i+1]) );
  }

  ccStrobe( CC1100_SCAL );

  my_delay_ms(4);

  uint8_t cnt = 0xff;
  do {
    ccStrobe(CC1100_SRX);
    my_delay_us(10);
  } while (cc1100_readReg(CC1100_MARCSTATE) != MARCSTATE_RX && cnt--);
  DC('z');
  DC('i');
  DNL();
}

static void
rf_zwave_reset_rx(void)
{
  ccStrobe( CC1100_SFRX  );
  ccStrobe( CC1100_SIDLE );
  ccStrobe( CC1100_SNOP  );
  ccStrobe( CC1100_SRX   );
}

void
rf_zwave_task(void)
{
  uint8_t msg[MAX_ZWAVE_MSG];

  if(!zwave_on)
    return;

  if(bit_is_set( CC1100_IN_PORT, CC1100_IN_PIN )) {

    CC1100_ASSERT;
    cc1100_sendbyte( CC1100_READ_BURST | CC1100_RXFIFO );
    for(uint8_t i=0; i<8; i++) {
       msg[i] = cc1100_sendbyte( 0 ) ^ 0xff;
    }
    CC1100_DEASSERT;

    uint8_t len=msg[7], off=8;
    if(len < 8 || len > MAX_ZWAVE_MSG) {
      rf_zwave_reset_rx();
      return;
    }

    cc1100_writeReg(CC1100_PKTLEN, len );
    for(uint8_t mwait=110; mwait > 0; mwait--) { // 52 bytes @ 40kBaud
      my_delay_us(100);
      uint8_t flen = cc1100_readReg( CC1100_RXBYTES );
      if(flen == 0)
        continue;
      CC1100_ASSERT;
      cc1100_sendbyte( CC1100_READ_BURST | CC1100_RXFIFO );
      if(off+flen > len)
        flen = len-off;
      for(uint8_t i=0; i<flen; i++) {
         msg[off++] = cc1100_sendbyte( 0 ) ^ 0xff;
      }
      CC1100_DEASSERT;
      if(off >= len)
        break;
    }

    uint8_t cnt = 0xff;
    do {
      ccStrobe(CC1100_SRX);
      my_delay_us(10);
    } while (cc1100_readReg(CC1100_MARCSTATE) != MARCSTATE_RX && cnt--);

    uint8_t cs = 0xff;          // CheckSum
    for(uint8_t i=0; i<len-1; i++)
      cs ^= msg[i];
    if(cs == msg[len-1]) {
      DC('z');
      for(uint8_t i=0; i<len; i++)
        DH2(msg[i]);
      DNL();
    }

    cc1100_writeReg(CC1100_PKTLEN, 0xff );
  }

  switch(cc1100_readReg( CC1100_MARCSTATE )) {
    case MARCSTATE_RXFIFO_OVERFLOW:
      ccStrobe( CC1100_SFRX  );
    case MARCSTATE_IDLE:
      ccStrobe( CC1100_SIDLE );
      ccStrobe( CC1100_SNOP  );
      ccStrobe( CC1100_SRX   );
      break;
  }
}

void
zwave_func(char *in)
{
  if(in[1] == 'r') {                // Reception on
    rf_zwave_init();
    zwave_on = 1;

  } else if(in[1] == 's') {         // Send
    //zwave_send(in+1);

  } else {                          // Off
    zwave_on = 0;

  }
}

#endif
