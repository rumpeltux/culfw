/* 
 * Copyright by R.Koenig
 * Inspired by code from Dirk Tostmann
 * License: GPL v2
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/parity.h>
#include <string.h>

#include "board.h"
#include "delay.h"
#include "rf_send.h"
#include "rf_receive.h"
#include "led.h"
#include "cc1100.h"
#include "display.h"
#include "fncollection.h"
#include "fht.h"

#ifdef HAS_DMX
#include "dmx.h"
#endif

#ifdef HAS_MORITZ
#include "rf_moritz.h"
#endif

// For FS20 we time the complete message, for KS300 the rise-fall distance
// FS20  NULL: 400us high, 400us low
// FS20  ONE:  600us high, 600us low
// KS300 NULL  854us high, 366us low
// KS300 ONE:  366us high, 854us low

#define FS20_ZERO      400     //   400uS
#define FS20_ONE       600     //   600uS
#define FS20_PAUSE      10     // 10000mS
#define EM_ONE         800     //   800uS
#define EM_ZERO        400     //   400uS

uint16_t credit_10ms;

void
send_bit(uint16_t hightime, uint16_t lowtime)
{
  CC1100_OUT_PORT |= _BV(CC1100_OUT_PIN);         // High
  my_delay_us(hightime);

  CC1100_OUT_PORT &= ~_BV(CC1100_OUT_PIN);       // Low
  my_delay_us(lowtime);
}

#ifdef HAS_RAWSEND

#define MAX_SNDMSG 12
#define MAX_SNDRAW 12

#define TMUL(x) ((x)<<4)
#define TDIV(x) ((x)>>4)

static uint8_t zerohigh, zerolow, onehigh, onelow;

static void
send_default_bit(uint8_t bit) {
  send_bit(TMUL(bit ? onehigh : zerohigh), TMUL(bit ? onelow : zerolow));
}

#else
#define MAX_SNDMSG 6    // FS20: 4 or 5 + CRC, FHT: 5+CRC
#define MAX_SNDRAW 7    // MAX_SNDMSG*9/8 (parity bit)

static void
send_default_bit(uint8_t bit)
{
  uint16_t time = bit ? FS20_ONE : FS20_ZERO;
  send_bit(time, time);
}

#endif

/**
 * Send raw data using the following specifications. Timings for 0 and 1 are
 * used from the global onehigh, zerohigh, onelow and zerolow settings.
 * msg: The raw data to send. Most significant bits will be sent first.
 * sync: Number of 0 bits to send as sync-sequence. Will be followed by exactly
 *     one 1 bit unless sync is set to 0 in which case no sync sequence will
 *     be sent.
 * nbyte: Number of complete bytes in msg.
 * bitoff: bit index in the additional byte that should not be sent anymore.
 *     E.g. to send 2 additional bits, bitoff would have to be 5 (7-2).
 *     Specify 7 if you don't want anything sent of the additional byte.
 * repeat: Number of times the packet shall be repeated.
 * pause: Pause in ms after each packet.
 * finalhigh: length of the final high bit before going into pause.
 *     Specify 0 if you don't need this.
 */
static void sendraw(uint8_t *msg, uint8_t sync, uint8_t nbyte, uint8_t bitoff, 
                uint8_t repeat, uint8_t pause, uint8_t finalhigh);

// msg is with parity/checksum already added
static void
sendraw(uint8_t *msg, uint8_t sync, uint8_t nbyte, uint8_t bitoff,
                uint8_t repeat, uint8_t pause, uint8_t finalhigh)
{
  // 12*800+1200+nbyte*(8*1000)+(bits*1000)+800+10000 
  // message len is < (nbyte+2)*repeat in 10ms units.
  int8_t i, j, sum = (nbyte+2)*repeat;
  if (credit_10ms < sum) {
    DS_P(PSTR("LOVF\r\n"));
    return;
  }
  credit_10ms -= sum;

  LED_ON();

  #if defined (HAS_IRRX) || defined (HAS_IRTX) //Blockout IR_Reception for the moment
    cli();
  #endif

#ifdef HAS_MORITZ
  uint8_t restore_moritz = 0;
  if(moritz_on) {
    restore_moritz = 1;
    moritz_on = 0;
    set_txreport("21");
  }
#endif

  if(!cc_on)
    set_ccon();
  ccTX();                                       // Enable TX 
  do {
    for(i = 0; i < sync; i++)                   // sync
      send_default_bit(0);
    if(sync)
      send_default_bit(1);
    
    for(j = 0; j < nbyte; j++) {                // whole bytes
      for(i = 7; i >= 0; i--)
        send_default_bit(msg[j] & _BV(i));
    }
    for(i = 7; i > bitoff; i--)                 // broken bytes
      send_default_bit(msg[j] & _BV(i));
    if (finalhigh)
      send_bit(TMUL(finalhigh), 0);

    my_delay_ms(pause);                         // pause

  } while(--repeat > 0);

  if(tx_report) {                               // Enable RX
    ccRX();
  } else {
    ccStrobe(CC1100_SIDLE);
  }

  #if defined (HAS_IRRX) || defined (HAS_IRTX) //Activate IR_Reception again
    sei(); 
  #endif

#ifdef HAS_MORITZ
    if(restore_moritz)
      rf_moritz_init();
#endif

  LED_OFF();
}

static int
abit(uint8_t b, uint8_t *obuf, uint8_t *obyp, uint8_t obi)
{
  uint8_t oby = * obyp;
  if(b)
    obuf[oby] |= _BV(obi);
  if(obi-- == 0) {
    oby++;
    if(oby < MAX_SNDRAW)
      *obyp = oby;
    obi = 7; obuf[oby] = 0;
  }
  return obi;
}

void
addParityAndSendData(uint8_t *hb, uint8_t hblen,
                uint8_t startcs, uint8_t repeat)
{
  uint8_t iby, obuf[MAX_SNDRAW], oby;
  int8_t ibi, obi;

  hb[hblen] = cksum1(startcs, hb, hblen);
  hblen++;

  // Copy the message and add parity-bits
  iby=oby=0;
  ibi=obi=7;
  obuf[oby] = 0;

  while(iby<hblen) {
    obi = abit(hb[iby] & _BV(ibi), obuf, &oby, obi);
    if(ibi-- == 0) {
      obi = abit(parity_even_bit(hb[iby]), obuf, &oby, obi);
      ibi = 7; iby++;
    }
  }
  if(obi-- == 0) {                   // Trailing 0 bit: no need for a check
    oby++; obi = 7;
  }

#ifdef HAS_RAWSEND
  zerohigh = zerolow = TDIV(FS20_ZERO);
  onehigh = onelow = TDIV(FS20_ONE);
#endif
  sendraw(obuf, 12, oby, obi, repeat, FS20_PAUSE, 0);
}

void
addParityAndSend(char *in, uint8_t startcs, uint8_t repeat)
{
  uint8_t hb[MAX_SNDMSG], hblen;
  hblen = fromhex(in+1, hb, MAX_SNDMSG-1);
  addParityAndSendData(hb, hblen, startcs, repeat);
}


void
fs20send(char *in)
{
#ifdef HAS_DMX
  if (dmx_fs20_emu( in ))
	return;
#endif
  addParityAndSend(in, 6, 3);
}

#ifdef HAS_RAWSEND
//G0843E54020204000A78DE81D80
//G0843E54020204000A78DE80F80

void
rawsend(char *in)
{
  uint8_t hb[16]; // 33/2: see ttydata.c
  uint8_t nby, nbi, pause, repeat, sync, finalhigh;

  fromhex(in+1, hb, sizeof(hb));
  sync = hb[0];
  nby = (hb[1] >> 4);
  nbi = 7 - (hb[1] & 0xf);
  pause = (hb[2] >> 4);
  repeat = (hb[2] & 0xf);
  zerohigh = hb[3];
  zerolow  = hb[4];
  onehigh  = hb[5];
  onelow   = hb[6];
  finalhigh = hb[7];
  sendraw(hb+8, sync, nby, nbi, repeat, pause, finalhigh);
}


// E0205E7000000000000
void
em_send(char *in)
{
  uint8_t iby, obuf[MAX_SNDRAW], oby;
  int8_t  ibi, obi;
  uint8_t hb[MAX_SNDMSG];

  uint8_t hblen = fromhex(in+1, hb, MAX_SNDMSG-1);

  // EM is always 9 bytes payload!
  if (hblen != 9) {
//  DS_P(PSTR("LENERR\r\n"));
    return;
  }
  
  onehigh = zerohigh = zerolow = TDIV(EM_ZERO);
  onelow  = TDIV(EM_ONE);

  // calc checksum
  hb[hblen] = cksum2( hb, hblen );
  hblen++;

  // Copy the message and add parity-bits
  iby=oby=0;
  ibi=obi=7;
  obuf[oby] = 0;
  
  while(iby<hblen) {
    obi = abit(hb[iby] & _BV(7-ibi), obuf, &oby, obi);
    if(ibi-- == 0) {
      obi = abit(1, obuf, &oby, obi); // always 1
      ibi = 7; iby++;
    }
  }
  if(obi-- == 0) {                   // Trailing 0 bit: indicating EOM
    oby++; obi = 7;
  }

  sendraw(obuf, 12, oby, obi, 3, FS20_PAUSE, 0);
}

#endif
