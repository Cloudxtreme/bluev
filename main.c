#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#define F_CPU  	20000000
#define BAUD 	57600

#include <util/delay.h>

/*-------------------------------------------------------------------------*/
#define PRESCALE1 8

/* timing formulas */
#define BITTIME(x)		( ((x) * F_CPU) / PRESCALE1 / BAUD)

/*-------------------------------------------------------------------------*/
#define MIDSIZE 24
static unsigned char midbuf[MIDSIZE], midlen;
static unsigned int frametime;
static unsigned char slice = 4;

int ckcksum( unsigned char *buf, unsigned char len ) {
    unsigned char ix, cks = 0;
    for( ix = 0; ix < len; ix++ )
	cks += buf[ix];
    return cks != buf[len];
}

// Received character from bluetooth
static unsigned char inmsgstate;
static int validate() {
    // device send ID should be valid for 3rd party
    unsigned char sl = midbuf[2] & 0x0f;
    if( sl < 3 || sl > 5 )
	return 0;
    // checksum test - note midlen is one short since this is called before 0xab is stored
    if( ckcksum(midbuf, midlen-1) )
	return 0;
    slice = sl;
    return 1;
}

ISR(USART_RX_vect)
{
    unsigned char c = UDR;

    switch( inmsgstate ) {
    case 0:
	if (c == 0xaa) {
	    midlen = 0;
	    inmsgstate++;
	}
	break;
    case 1:
	if ((c & 0xf0) == 0xd0)
	    inmsgstate++;
	else
	    inmsgstate = 0;
	break;
    case 2:
	if ((c & 0xf0) == 0xe0)
	    inmsgstate++;
	else
	    inmsgstate = 0;
	break;
    case 3:
	if( midlen == 4 && c > 22 ) { // length
	    inmsgstate = 0;
	    break;
	}	    
	if ( c == 0xab && midlen > 4 && midbuf[4] + 5 == midlen) {
	    midbuf[midlen] = c; // avoid race 
	    if( validate() )
		inmsgstate = 4;
	    else
		inmsgstate = 0;
	}
	break;
    case 4: // waiting for transmit
	return;
    default:
	break;
    }
    if( inmsgstate && midlen < MIDSIZE )
	midbuf[midlen++] = c;
    else
	inmsgstate = 0;
}

/*-------------------------------------------------------------------------*/
static unsigned char polarity;
//static unsigned int hitime;
static unsigned int bitcnt;


/*-------------------------------------------------------------------------*/
#if 0
ISR(TIMER1_OVF_vect)
{
    hitime++;
}
#endif

// This counts character times to find the slot to transmit.  
// Each slot is 45 character times wide
// FIXME - doesn't try to resync or otherwise avoid collisions with other devices
ISR(TIMER1_COMPB_vect)
{
    OCR1B += BITTIME(10);
    frametime++;

    if( frametime < 45*slice )
	return;

    if( inmsgstate != 4 ) { // nothing for this frame
	TIMSK &= ~_BV(OCIE1B);
	UCSRB &= ~_BV(TXEN);      /* disable tx */
	frametime = 0;
	return;
    }

    if( frametime == 45*slice ) {
	UCSRB |= _BV(TXEN);      /* enable tx */
	return;
    }

    if( frametime >= 45*(slice+1) ) {
	TIMSK &= ~_BV(OCIE1B);
	UCSRB &= ~_BV(TXEN);      /* disable tx */
	frametime = 0;
	inmsgstate = 0;
	return;
    }

    if( !(frametime & 1) ) { // Data Out Pacing, every other frame until done
	unsigned char ptr = (frametime - (45*slice+1)) >> 1;
	if( ptr < midlen )
	    UDR = midbuf[ptr];
	if( ptr > midlen )	
	    UCSRB &= ~_BV(TXEN);      /* disable tx */
    }
}

// This tracks the V1 infDisplayData packet to sync the ESP cycle
static unsigned char v1state, thislen;
const unsigned char infDisp[] = "\xaa\xd8\xea\x31\x09"; // put in Flash?
void dostate(unsigned char val) {
    if( v1state < 5 ) {
	if (val == infDisp[v1state] ) {
	    v1state++;
	    thislen = v1state;
	}
	else
	    v1state = 0;
	return;
    }
    thislen++;
    if( val == 0xab && thislen == 15 ) {
	frametime = 0;
	v1state = 0;
#if 0
	// FIXME maybe checksum v1 receives
	if( ckcksum(inbuf, thislen-2) )
	    return 0;
#endif
	OCR1B = OCR1A + BITTIME(10) + BITTIME(1) / 2;
	TIFR |= _BV(OCF1B);         /* clear compare match interrupt */
	TIMSK |= _BV(OCIE1B);       /* enable compare match interrupt */
    }
}

static unsigned char outchar;
//Stopbit for software UART
ISR(TIMER1_COMPA_vect)
{
    TIMSK &= ~_BV(OCIE1A); // disable
    if( !polarity ) { // not break condition
	while( bitcnt < 10 ) {
	    bitcnt++;
	    outchar >>= 1;
	    outchar |= 0x80;
	}
	dostate(outchar);
	//	UDR = 0x30 + thislen;
    }
    else { // break, reset things for next start bit
	TCCR1B &= ~_BV(ICES1);
	polarity = 0;
    }
    bitcnt = 0;
}

// Software UART via edges
ISR(TIMER1_CAPT_vect)
{
static unsigned lastedge;
    TCCR1B ^= _BV(ICES1);
    /* toggle interrupt on rising/falling edge */
    if( !polarity && !bitcnt ) { // start bit
	lastedge = ICR1;
	OCR1A = lastedge + BITTIME(9) + BITTIME(1) / 2;
	TIFR |= _BV(OCF1A);         /* clear compare match interrupt */
	TIMSK |= _BV(OCIE1A);       /* enable compare match interrupt */
	polarity = 1;
	bitcnt = 1;
	return;
    }
    unsigned thisedge = ICR1;
    unsigned width = thisedge - lastedge;
    lastedge = thisedge;
    width += BITTIME(1) / 2; // round up
    while( width >= BITTIME(1) ) {
	width -= BITTIME(1);
	bitcnt++;
	outchar >>= 1;
	if( !polarity )
	    outchar |= 0x80;
    }
    polarity ^= 1;
}

/*-------------------------------------------------------------------------*/
int main(void)
{
    /* power savings */
    PRR = _BV(PRUSI);           /* shut down USI */
    DIDR = _BV(AIN0D) | _BV(AIN1D);     /* disable digital input on analog */
    ACSR = _BV(ACD);            /* disable analog comparator */

    /* for testing
    inmsgstate = 4;
    strcpy_P( midbuf, PSTR("\xaa\xda\xe4\x41\x01\xaa\xab") );
    midlen = 7;
    */
    v1state = inmsgstate = midlen = 0;

    /* setup serial port */
#include <util/setbaud.h>
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
#if USE_2X
    UCSRA |= (1 << U2X);
#else
    UCSRA &= ~(1 << U2X);
#endif
    UCSRC = _BV(UCSZ0) | _BV(UCSZ1);    /* 8N1 */
    UCSRB = _BV(RXEN) | _BV(RXCIE);

    GTCCR = _BV(PSR10);         /* reset prescaler */
    TCNT1 = 0;         /* reset counter value */
    // hitime = 0;
    /* activate noise canceller, */
    /* trigger on falling edge, clk/1 */
    // lower 3 bits is div, off,1,8,64,256,1024,extfall,extris ; CS12,11,10
    polarity = bitcnt = 0;
    TCCR1B = _BV(ICNC1) | _BV(CS11);

    // clear and enable Input Capture interrupt 
    OCR1A = OCR1B = 32768;
    TIFR |= _BV(ICF1) | _BV(TOV1) | _BV(OCF1A) | _BV(OCF1B);
    TIMSK |= _BV(ICIE1);

    //TIMSK |= _BV(TOIE1); 	/* enable Overflow interrupt */

    sei();

    set_sleep_mode(SLEEP_MODE_IDLE);
    for (;;)
        sleep_mode();

}
