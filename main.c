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

// Received character from bluetooth
static unsigned char inmsg;
static int validate() {
    if( frametime ) // sending in progress - treat as overrun and ignore
	return 0;
    // device send ID should be valid for 3rd party
    unsigned char sl = midbuf[2] & 0x0f;
    if( sl < 3 || sl > 5 )
	return 0;
#if 1
    // checksum test - note midlen is one short since this is called before 0xab is stored
    unsigned char ix, cks = 0;
    for( ix = 0; ix < midlen - 1; ix++ )
	cks += midbuf[ix];
    if( cks != midbuf[midlen - 1] )
	return 0;
#endif
    slice = sl;
    return 1;
}

ISR(USART_RX_vect)
{
    unsigned char c = UDR;

    switch( inmsg ) {
    case 0:
	if (c == 0xaa) {
	    midlen = 0;
	    inmsg++;
	}
	break;
    case 1:
	if ((c & 0xf0) == 0xd0)
	    inmsg++;
	else
	    inmsg = 0;
	break;
    case 2:
	if ((c & 0xf0) == 0xe0)
	    inmsg++;
	else
	    inmsg = 0;
	break;
    case 3:
	if( midlen == 4 && c > 22 ) { // length
	    inmsg = 0;
	    break;
	}	    
	if ( c == 0xab && midlen > 4 && midbuf[4] + 5 == midlen) {
	    midbuf[midlen] = c; // avoid race 
	    if( validate() )
		inmsg = 4;
	    else
		inmsg = 0;
	}
	break;
    case 4: // waiting for transmit
	return;
    default:
	break;
    }
    if( inmsg && midlen < MIDSIZE )
	midbuf[midlen++] = c;
    else
	inmsg = 0;
}

/*-------------------------------------------------------------------------*/
static unsigned char polarity;
//static unsigned int hitime;
static unsigned int bitcnt;

// General setup of timing and J1850 - for out of reset and out of deep sleep
void hardwareinit(void)
{
    inmsg = 4;
    strcpy_P( midbuf, PSTR("\xaa\xda\xe4\x41\x01\xaa\xab") );
    midlen = 7;

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
    /* edge input - PD6 as input without pullup */
    //DDRD &= ~_BV(PD6);
    //    PORTD &= ~_BV(PD6);

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
    //TIMSK |= _BV(OCIE1A);     /* enable compare match interrupt */
    //TIMSK |= _BV(OCIE1B);     /* enable compare match interrupt */
}

/*-------------------------------------------------------------------------*/
#if 0
ISR(TIMER1_OVF_vect)
{
    hitime++;
}
#endif
ISR(TIMER1_COMPB_vect)
{
    OCR1B += BITTIME(10);
    frametime++;

    if( frametime < 45*slice )
	return;

    if( inmsg != 4 ) { // nothing for this frame
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
	inmsg = 0;
	return;
    }

    if( !(frametime & 1) ) {
	unsigned char ptr = (frametime - (45*slice+1)) >> 1;
	if( ptr < midlen )
	    UDR = midbuf[ptr];
	else
	    UCSRB &= ~_BV(TXEN);      /* disable tx */
    }
}

static unsigned char v1state = 0, thislen;
void dostate(unsigned char val) {
    switch( v1state ) {
    case 0:
	if (val == 0xaa )
	    v1state++;
	break;
    case 1:
	if (val == 0xd8 )
	    v1state++;
	else
	    v1state = 0;
	break;
    case 2:
	if (val == 0xea )
	    v1state++;
	else
	    v1state = 0;
	break;
    case 3:
	if (val == 0x31 )
	    v1state++;
	else
	    v1state = 0;
	break;
    case 4:
	if (val == 0x09 ) {
	    v1state++;
	    thislen = 5;
	}
	else
	    v1state = 0;
	break;
    case 5:
	thislen++;
	if( val == 0xab && thislen == 15 ) {
	    frametime = 0;
	    OCR1B = OCR1A + BITTIME(10) + BITTIME(1) / 2;
	    TIFR |= _BV(OCF1B);         /* clear compare match interrupt */
	    TIMSK |= _BV(OCIE1B);       /* enable compare match interrupt */
	    v1state = 0;
	}
	break;
    default:
	v1state = 0;
	break;
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

    /* setup uart and icp*/
    hardwareinit();
    sei();

    set_sleep_mode(SLEEP_MODE_IDLE);
    for (;;)
        sleep_mode();

}
