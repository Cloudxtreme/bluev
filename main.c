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
#define OUTSIZE 64
#define MIDSIZE 32
static unsigned char midbuf[MIDSIZE], outbuf[OUTSIZE], outhead, outtail,  midlen;
#define UPUTC(x) { outbuf[outhead++] = (x); if (outhead >= OUTSIZE) outhead = 0; }
#define UGETC(x) { (x) = outbuf[outtail++]; if (outtail >= OUTSIZE) outtail = 0; }

// add buffered send data to output buffer
static void sendmid()
{
    unsigned char *mp = midbuf;
    while (midlen) {
	UPUTC(*mp++);
        midlen--;
    }
}

// Received character from GPS - put to transmit buffer, but also handle other cases
unsigned char inmsg;
ISR(USART_RX_vect)
{
    unsigned char c = UDR;

    switch( inmsg ) {
    case 0:
	if (c == 0xaa) {
	    midlen = 0;
	    inmsg = 1;
	}
	break;
    case 5:
	if (c == 0xab) {
	    inmsg = 6;

	}
    default:
	break;
    }
    midbuf[midlen++] = c;
    if( inmsg == 6 ) { // message complete, queue for send
	sendmid();
	inmsg = 0;
    }
}

/*-------------------------------------------------------------------------*/
static unsigned char polarity;
//static unsigned int hitime;
static unsigned int bitcnt;

// General setup of timing and J1850 - for out of reset and out of deep sleep
void hardwareinit(void)
{
    inmsg = 0;
    outhead = outtail = midlen = 0;

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
    UCSRB = _BV(RXEN);

    //    UCSRB &= ~_BV(TXEN);      /* disable tx */
    //    DDRD &= ~_BV(PD1); // set to input
    //    PIND &= ~_BV(PD1); // no pullup

    /* edge input - PD6 as input without pullup */
    DDRD &= ~_BV(PD6);
    PORTD &= ~_BV(PD6);

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
static unsigned int frametime;
static unsigned char slice = 4;
ISR(TIMER1_COMPB_vect)
{
    OCR1B += BITTIME(10);
    frametime++;
    if( frametime < 45*slice )
	return;
    if( frametime == 45*slice ) {
	UCSRB |= _BV(TXEN);      /* enable tx */
	return;
    }
    if( frametime >= 45*(slice+1) ) {
	TIMSK &= ~_BV(OCIE1B);
	UCSRB &= ~_BV(TXEN);      /* disable tx */
	return;
    }
    if( !(frametime & 1) ) {
	unsigned char ptr = (frametime - (45*slice+1)) >> 1;
	if( ptr < 7 ) {
	    unsigned char *p = PSTR("\xaa\xda\xe4\x41\x01\xaa\xab");
	    UDR = pgm_read_byte(p+ptr);
	}
	else {
	    //UCSRB &= ~_BV(TXEN);      /* disable tx */
	}
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
    //    uart_puts_P(PSTR("=Harley J1850-GPS\r\n"));

    //    UCSRB |= _BV(RXCIE);
    set_sleep_mode(SLEEP_MODE_IDLE);
    for (;;) {
        sleep_mode();
    }
}
