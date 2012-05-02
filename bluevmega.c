#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>

//#define F_CPU         18432000
#ifndef F_CPU
#define F_CPU  	16000000
#endif
#define BAUD 	57600

#define PRESCALE1 8
#define BITTIME(x)		( ((x) * F_CPU) / PRESCALE1 / BAUD)

//#include <util/delay.h>

/*-------------------------------------------------------------------------*/
#define INMSGSIZE 24
static unsigned char inmsgbuf[INMSGSIZE];
static volatile unsigned char inmsglen, inmsgcks;
static volatile unsigned char slice = 4;
static volatile unsigned char transp = 0;

static volatile unsigned char nochecksum = 0;
#define ANYSLICE
// Received character from bluetooth
static volatile unsigned char inmsgstate;
static void inchar(unsigned char c)
{
    switch (inmsgstate) {
    case 0:                    // SOF
        if (c == 0xaa) {
            inmsgcks = 0;
            inmsglen = 0;
            inmsgstate++;
        }
        break;
    case 1:                    // destination
        if ((c & 0xf0) == 0xd0)
            inmsgstate++;
        else
            inmsgstate = 0;
        break;
    case 2:                    // source
        if ((c & 0xf0) == 0xe0
#ifndef ANYSLICE
          && (c & 0x0f) <= 5 && (c & 0x0f) >= 3
#endif
          )                     // valid source ?
            inmsgstate++;
        else
            inmsgstate = 0;
        break;
    case 3:                    // ID through EOF
        if (inmsglen == 4 && c > 20) {  // validate length
            inmsgstate = 0;
            break;
        }
        if (inmsglen < 5)       // later tests require length
            break;

        if (!nochecksum)
            if (inmsgbuf[4] + 4 == inmsglen) {  // checksum byte
                if (inmsgcks != c)
                    inmsgstate = 0;
                break;
            }
        if (c == 0xab && inmsgbuf[4] + 5 == inmsglen) { // EOF - queue for send
            inmsgbuf[inmsglen] = c;     // avoid race 
            slice = inmsgbuf[2] & 0x0f;
            inmsgstate = 4;
        }
        break;
    case 4:                    // waiting for transmit - discard
        return;
    default:
        break;
    }
    if (inmsgstate && inmsglen < INMSGSIZE) {
        inmsgbuf[inmsglen++] = c;
        inmsgcks += c;
    }
    else                        // overflow - shouldn't happen given state machine
        inmsgstate = 0;
}

// Save off characters until a valid V1 ESP packet is complete
ISR(USART2_RX_vect)
{
    inchar(UDR2);
}

static unsigned char inbuf[256], v1buf[256];
static volatile unsigned char v1head = 0, v1tail = 0, inhead = 0, intail = 0;
ISR(USART0_RX_vect)
{
    if (transp) {
        inchar(UDR0);
        return;
    }
    inbuf[inhead++] = UDR0;
}

/*-------------------------------------------------------------------------*/
static volatile unsigned int bitcnt;
static unsigned int frametime;
// Time Slice processor
// This counts character times to find the slot to transmit.  
// Each slot is 45 character times wide
// FIXME - doesn't try to resync or otherwise avoid collisions with other devices
ISR(TIMER4_COMPB_vect)
{
    OCR4B += BITTIME(10);
    frametime++;

    if (frametime < 45 * slice)
        return;

    if (inmsgstate != 4) {      // nothing for this frame
        TIMSK4 &= ~_BV(OCIE4B); // slice processor off
        UCSR1B &= ~_BV(TXEN0);  // TX off - just in case
        frametime = 0;
        return;
    }

    if (
#ifdef ANYSLICE
      (slice == 0 && frametime == 1) ||
#endif
      frametime == 45 * slice) {        // At current time slice

        // Holdoff for late previous time slice
        if (bitcnt)
            frametime--;
        else
            UCSR1B |= _BV(TXEN0);       // TX on
        return;
    }

    if (frametime >= 45 * (slice + 1)) {        // end of time slice
        TIMSK4 &= ~_BV(OCIE4B); // slice processor off
        UCSR1B &= ~_BV(TXEN0);  // TX off
        frametime = 0;
        inmsgstate = 0;
        return;
    }

    if (!(frametime & 1)) {     // Data Out Pacing, every other frame until done
        unsigned char ptr = (frametime - (45 * slice + 1)) >> 1;
        if (ptr < inmsglen)
            UDR1 = inmsgbuf[ptr];
        if (ptr > inmsglen)
            UCSR1B &= ~_BV(TXEN0);      // TX off
    }
}

// This tracks the V1 infDisplayData packet to sync the ESP cycle
static volatile unsigned char v1state, thislen;
static unsigned char infDisp[] = "\xaa\xd8\xea\x31\x09";        // put in Flash?
static void dostate(unsigned char val)
{
    // FIXME - hardcoded packet length
    // on the fly comparison of the first 5 bytes of the infDisplay packet
    if (v1state < 5) {
        v1state++;
        if (v1state == 3) {     // Checksum or not?
            if (val == 0xea)
                infDisp[4] = 9, nochecksum = 0;
            else if (val == 0xe9)
                infDisp[4] = 8, nochecksum = 1;
            else
                v1state = 0;
            return;
        }
        if (val == infDisp[v1state - 1])
            thislen = v1state;
        else
            v1state = 0;
        return;
    }
    thislen++;
    if (thislen == 11 && (val & 2)) {   // V1 TimeSlice holdoff
        v1state = 0;
        return;
    }
#if 0
    // FIXME? maybe validate checksum?
    if (thislen == 14 && val != ckcksum(inbuf, 14))
        return 0;
#endif
    if (val == 0xab && thislen == 6 + infDisp[4]) {     // EOF - start time slice sync
        frametime = 0;
        v1state = 0;
        OCR4B = OCR4A + BITTIME(10) + BITTIME(1) / 2;
        TIFR4 |= _BV(OCF4B);    /* clear compare match interrupt */
        TIMSK4 |= _BV(OCIE4B);  /* enable compare match interrupt */
        return;
    }
    if (thislen > 6 + infDisp[4])       // too long
        v1state = 0;
}

static unsigned char outchar;   // bitbang UART receive register
static unsigned char polarity;  // which edge are we looking for
//Stopbit for software UART
ISR(TIMER4_COMPA_vect)
{
    TIMSK4 &= ~_BV(OCIE4A);     // disable
    if (!polarity) {            // not break condition
        while (bitcnt < 10) {   // fill in one bits up to stop bit
            bitcnt++;
            outchar >>= 1;
            outchar |= 0x80;
        }
        UDR2 = outchar;
        // UDR0=64|63&outchar;
        if (transp)
            UDR0 = outchar;
        else
            v1buf[v1head++] = outchar;
        dostate(outchar);
    }
    else {                      // break, reset things for next start bit
        TCCR4B &= ~_BV(ICES1);
        polarity = 0;
    }
    bitcnt = 0;
}

// Software UART via edges
ISR(TIMER4_CAPT_vect)
{
    static unsigned lastedge;
    TCCR4B ^= _BV(ICES1);
    /* toggle interrupt on rising/falling edge */
    if (!polarity && !bitcnt) { // start bit
        lastedge = ICR4;
        OCR4A = lastedge + BITTIME(9) + BITTIME(1) / 2;
        TIFR4 |= _BV(OCF4A);    /* clear compare match interrupt */
        TIMSK4 |= _BV(OCIE4A);  /* enable compare match interrupt */
        polarity = 1;
        bitcnt = 1;
        return;
    }
    unsigned thisedge = ICR4;
    unsigned width = thisedge - lastedge;
    lastedge = thisedge;
    width += BITTIME(1) / 2;    // round up
    while (width >= BITTIME(1)) {       // Shift in bits based on width
        width -= BITTIME(1);
        bitcnt++;
        outchar >>= 1;
        if (!polarity)
            outchar |= 0x80;
    }
    polarity ^= 1;
}

/*========================================================================*/
// V1 Command, Response, and Info packet Processing

/*========================================================================*/

// LOW LEVEL ROUTINES FOR ARDUINO

// Read one character from the V1 data stream
static int readv1rx(void)
{
    while (v1head == v1tail)
        sleep_mode();
    return v1buf[v1tail++];
}

static char serbuf[256];
// print out strings to the main (USB) serial port
static void printser(char *str)
{
    while (*str) {
        while (!(UCSR0A & _BV(UDRE0)));
        UDR0 = *str++;
    }
}

// print string from progmem
static void printser_P(const char *p)
{
    char c;
    while ((c = pgm_read_byte(p++))) {
        while (!(UCSR0A & _BV(UDRE0)));
        UDR0 = c;
    }
}

#define REQVERSION (1)
#define RESPVERSION (2)
#define REQSERIALNUMBER (3)
#define RESPSERIALNUMBER (4)

#define REQUSERBYTES (0x11)
#define RESPUSERBYTES (0x12)
#define REQWRITEUSERBYTES (0x11)
#define REQFACTORYDEFAULT (0x14)

#define REQWRITESWEEPDEFINITIONS (0x15)
#define REQALLSWEEPDEFINITIONS (0x16)
#define RESPSWEEPDEFINITION (0x17)
#define REQDEFAULTSWEEPS (0x18)

#define REQMAXSWEEPINDEX (0x19)
#define RESPMAXSWEEPINDEX (0x20)
#define RESPSWEEPWRITERESULT (0x21)
#define REQSWEEPSECTIONS (0x22)
#define RESPSWEEPSECTIONS (0x23)

#define INFDISPLAYDATA (0x31)
#define REQTURNOFFMAINDISPLAY (0x32)
#define REQTURNONMAINDISPLAY (0x33)
#define REQMUTEON (0x34)
#define REQMUTEOFF (0x35)
#define REQCHANGEMODE (0x36)

#define REQSTARTALERTDATA (0x41)
#define REQSTOPALERTDATA (0x42)
#define RESPALERTDATA (0x43)

#define RESPDATARECEIVED (0x61)
#define REQBATTERYVOLTAGE (0x62)
#define RESPBATTERYVOLTAGE (0x63)
#define RESPUNSUPPORTEDPACKET (0x64)
#define RESPREQUESTNOTPROCESSED (0x65)
#define INFV1BUSY (0x66)
#define RESPDATAERROR (0x67)

#define REQSAVVYSTATUS (0x71)
#define RESPSAVVYSTATUS (0x72)
#define REQVEHICLESPEED (0x73)
#define RESPVEHICLESPEED (0x74)
#define REQOVERRIDETHUMBWHEEL (0x75)
#define REQSETSAVVYMUTEENABLE (0x76)

#define NORESPONSE (0xfe)

static unsigned char tstnocks = 0;      // is this a checksummed V1?

// Create a valid V1 command in BUF, return length.  Returns 0 if something is invalid
// Note it permits source and dest 0-15, not just a source of 3,4,5.
// pkt is the ID of the command.
// len is size of param, param points to the bytes to go after the length
// (before and not including the optional checksum which will be handled
// according to what the infDisplay is doing.
static int makecmd(unsigned char *buf, unsigned char src, unsigned char dst, unsigned char pkt,
  unsigned char len, unsigned char *param)
{
    if (len > 16)
        return 0;
    if (src > 15 || dst > 15)
        return 0;
    if (len && !param)
        return 0;
    unsigned char *b = buf, l = len;
    *b++ = 0xaa;
    if (tstnocks && dst == 0x0a)
        dst = 9;
    if (dst == 9 && !tstnocks)
        dst = 0x0a;
    *b++ = 0xd0 + dst;
    *b++ = 0xe0 + src;
    *b++ = pkt;
    *b++ = len + !tstnocks;
    while (l--)
        *b++ = *param++;
    if (!tstnocks) {
        unsigned char cks = 0, ix = 0;
        for (ix = 0; ix < len + 5; ix++)
            cks += buf[ix];
        *b++ = cks;
    }
    *b++ = 0xab;
    return b - buf;
};

static unsigned char v1idd = 0, v1infdisplaydata[8];
static unsigned char v1alerts = 0, v1alerttemp[16][7], v1alertout[16][7];
static unsigned char cddr = 0;

// Get a full, valid packet from the V1 - SOF, EOF, source, destination, and length bytes checked.
// For unsolicited packets (infdisp, respalert, respdatarx), process internally
static int readpkt(unsigned char *buf)
{
    unsigned char len, ix;
    buf[0] = 0;
    for (;;) {
        while (buf[0] != 0xaa)  // SOF
        {
            buf[0] = readv1rx();
            //sprintf( serbuf, "%02x", buf[0] );
            //printser( serbuf );
        }
        buf[1] = readv1rx();
        if ((buf[1] & 0xf0) != 0xd0)
            continue;
        buf[2] = readv1rx();
        if ((buf[2] & 0xf0) != 0xe0)
            continue;
        if ((buf[2] & 15) == 0xa)
            tstnocks = 0;
        else if ((buf[2] & 15) == 9)
            tstnocks = 1;
        buf[3] = readv1rx();
        buf[4] = readv1rx();
        len = 5;
        for (ix = 0; ix < buf[4]; ix++) {
            buf[len] = readv1rx();
            len++;
        }
        if (!tstnocks) {
            unsigned char cks = 0;
            for (ix = 0; ix < len - 1; ix++)
                cks += buf[ix];
            if (buf[len - 1] != (cks & 0xff))
                return -2;      // continue; ???
        }
        buf[len] = readv1rx();
        if (buf[len++] != 0xab)
            continue;

        // save off current alert or inf packet separately
        if (buf[3] == INFDISPLAYDATA) {
            // maybe verify rest of packet
            v1idd++;
            memcpy(v1infdisplaydata, buf + 5, 8);
        }
        else if (buf[3] == RESPALERTDATA) {
            //sprintf(serbuf, "a: %d\n", buf[1] & 0xf);
            //printser(serbuf);
            // copy to temp until index == total, then move to out and inc
            if (!buf[5]) {      // no alerts
                if (!v1alertout[0][0]) {        // already zero?
                    memset(v1alertout, 0, 7);
                    v1alerts++;
                }
            }
            else {              // accumulate and copy block
                memcpy(v1alerttemp[(buf[5] >> 4) - 1], buf + 5, 7);
                if (buf[5] >> 4 == (buf[5] & 15)) {
                    memcpy(v1alertout, v1alerttemp, 7 * (buf[5] & 15));
                    v1alerts++;
                }
            }
        }
        else if (buf[3] == RESPDATARECEIVED) {
            cddr++;
        }
        else {
#if 0
            // show nonstream traffic
            sprintf(serbuf, "r: %d:", len);
            printser(serbuf);
            for (ix = 1; ix < len - 1; ix++) {
                sprintf(serbuf, " %02x", buf[ix]);
                printser(serbuf);
            }
            printser_P(PSTR("\r\n"));
#endif
        }
        return len;
    }
}

// send a command, wait until it goes out, then look for a response packet (by ID), placing it in buf.
static int sendcmd(unsigned char *thiscmd, unsigned char resp, unsigned char *buf)
{
    unsigned char ix;
    int ret;

    memcpy(inmsgbuf, thiscmd, thiscmd[4] + 6);
    inmsglen = thiscmd[4] + 6;
    inmsgstate = 4;

    // wait this many packets max for the command to go out on the bus
#define ECHOTIME 8
    for (ix = 0; ix < ECHOTIME; ix++) { // look for command on bus
        ret = readpkt(buf);
        //      sprintf( serbuf, "%d: %02x %02x %02x %02x %02x %02x %02x\r\n", ret, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6] );
        if (ret < thiscmd[4] + 6)
            continue;
        if (!memcmp(thiscmd, buf, ret))
            break;
    }
    if (ix == ECHOTIME)
        return -1;

    if (resp == NORESPONSE)
        return 0;

    // FIXME - we don't check that the destination is our ID.
    // look for response, max packets to wait - busy will reset.
#define RESPTIME 20
    for (ix = 0; ix < RESPTIME; ix++) {
        ret = readpkt(buf);
        if (ret < 6)
            continue;
        switch (buf[3]) {
        case RESPUNSUPPORTEDPACKET:
            sprintf(serbuf, "Unsupported Packet\r\n");
            printser(serbuf);
            break;
        case RESPREQUESTNOTPROCESSED:
            sprintf(serbuf, "Request Not Processed %02x\r\n", buf[5]);
            printser(serbuf);
            // maybe abort, return -3?
            break;
        case INFV1BUSY:
            // FIXME, we don't check packet ID
#if 0
            printser_P(PSTR("V1Busy:"));
            for (ix = 0; ix < buf[4] - 1; ix++) {
                sprintf(serbuf, " %02x", buf[5 + ix]);
                printser(serbuf);
            }
            printser(serbuf, "\r\n"));
#endif
            ix = 0;             // reset timer
            break;
        case RESPDATAERROR:
            sprintf(serbuf, "Data Error %02x\r\n", buf[5]);
            printser(serbuf);
            break;
        }

        if (buf[3] == resp)
            break;
    }
    if (ix == RESPTIME)
        return -2;
    return 0;
}

// command and response buffers
static unsigned char respget[22], cmdsend[22];
static unsigned int maxswp = 5;

// make sure the buffer is clear by getting an echo.
// FIXME - should have long timeout and return status.
static void syncresp() {
    int iy;
    // send one request version to clear out the incoming packet respgetfer
    makecmd(cmdsend, slice, 0xa, REQVERSION, 0, NULL);
    sendcmd(cmdsend, NORESPONSE, respget);
    for (;;) {
        iy = readpkt(respget);
        if (iy < 6)
            continue;
        if (respget[3] == RESPVERSION)
            break;
    }
}

// This is the first part of the sweep probe getting the sections and maximum index
static void sweep1() {
    int ix, ret;
    // Sweep Sections and Definitions
    makecmd(cmdsend, slice, 0xa, REQSWEEPSECTIONS, 0, NULL);
    sendcmd(cmdsend, RESPSWEEPSECTIONS, respget);
    printser_P(PSTR("SweepSections:\r\n"));
    sprintf(serbuf, "+%d/%d %5u - %5u\r\n", respget[5] >> 4, respget[5] & 15, respget[8] << 8 | respget[9],
      respget[6] << 8 | respget[7]);
    printser(serbuf);
    if (respget[4] > 6) {
        sprintf(serbuf, "+%d/%d %5u -:%5u\r\n", respget[10] >> 4, respget[10] & 15, respget[13] << 8 | respget[14],
          respget[11] << 8 | respget[12]);
        printser(serbuf);
    }
    if (respget[4] > 11) {
        sprintf(serbuf, "+%d/%d %5u - %5u\r\n", respget[15] >> 4, respget[15] & 15, respget[18] << 8 | respget[19],
          respget[16] << 8 | respget[17]);
        printser(serbuf);
    }
    unsigned int nswppkt = (respget[5] & 15);
    for (ix = 1; ix < nswppkt / 3;) {
        // read additional 0x23 packet, print it out
        ret = readpkt(respget);
        if (ret < 5)
            continue;
        if (respget[3] != 0x23)
            continue;
        ix++;
        sprintf(serbuf, "+%d/%d %5u - %5u\r\n", respget[5] >> 4, respget[5] & 15, respget[8] << 8 | respget[9],
          respget[6] << 8 | respget[7]);
        printser(serbuf);

        if (respget[4] > 6) {
            sprintf(serbuf, "+%d/%d %5u -:%5u\r\n", respget[10] >> 4, respget[10] & 15, respget[13] << 8 | respget[14],
              respget[11] << 8 | respget[12]);
            printser(serbuf);
        }
        if (respget[4] > 11) {
            sprintf(serbuf, "+%d/%d %5u - %5u\r\n", respget[15] >> 4, respget[15] & 15, respget[18] << 8 | respget[19],
              respget[16] << 8 | respget[17]);
            printser(serbuf);
        }
    }
    // sweep definitions must stay within the sections above
    makecmd(cmdsend, slice, 0xa, REQMAXSWEEPINDEX, 0, NULL);
    sendcmd(cmdsend, RESPMAXSWEEPINDEX, respget);
    sprintf(serbuf, "MaxSweepIndex: %d (+1 for number of definitions)\r\n", respget[5]);
    printser(serbuf);
    maxswp = respget[5];
}

// This is the second part of the sweep probe which gets the actual sweep definitions
static void sweep2() {
    int ix, ret;
    // read sweep sections
    makecmd(cmdsend, slice, 0xa, REQALLSWEEPDEFINITIONS, 0, NULL);
    sendcmd(cmdsend, RESPSWEEPDEFINITION, respget);
        printser_P(PSTR("SweepDefinitions:\r\n"));
    sprintf(serbuf, "+%d/%d Low:%5u High:%5u\r\n", 1 + (respget[5] & 63), 1 + maxswp, respget[8] << 8 | respget[9],
      respget[6] << 8 | respget[7]);
    printser(serbuf);
    for (ix = 0; ix < maxswp;) {
        ret = readpkt(respget);
        if (ret < 5)
            continue;
        if (respget[3] != 0x17)
            continue;
        ix++;
        sprintf(serbuf, "+%d/%d Low:%5u High:%5u\r\n", 1 + (respget[5] & 63), 1 + maxswp, respget[8] << 8 | respget[9],
          respget[6] << 8 | respget[7]);
        printser(serbuf);
    }
}

// user interactive program to get a 16 bit unsigned word (frequency to write sweep)
static unsigned getword() {
    unsigned ret = 0;
    unsigned char c;
    for (;;) {
        while (inhead == intail)
            readpkt(respget);
        c = inbuf[intail++];
        if (c >= ' ' && c < 127) {
            while (!(UCSR0A & 0x20));
            UDR0 = c;
        }
        switch (c) {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            ret *= 10;
            ret += c - '0';
            break;
        case 127:
        case 8:
            ret /= 10;
            printser_P(PSTR("\b \b"));
            break;
        case '\r':
            return ret;
        case 0x15:
            ret = 0;
            printser_P(PSTR("\r            \r"));
            break;
        case 0x1b:
            return 0;
        default:
            break;
        }
    }
}

// Set a new set of sweep definitions
static void sweepset() {
    int ix, iy;
    unsigned low[20], high[20];

    syncresp();

    sweep1();
    sweep2();
    printser_P(PSTR("Enter Definition Ranges, low and high, 0 to end\r\n"));
    for (ix = 0; ix <= maxswp; ix++) {
        sprintf(serbuf, "Def %d Low:\r\n", ix + 1);
        printser(serbuf);
        low[ix] = getword();
        printser_P(PSTR("\r\n"));
        if (low[ix] == 0)
            break;
        sprintf(serbuf, "Def %d High:\r\n", ix + 1);
        printser(serbuf);
        high[ix] = getword();
        printser_P(PSTR("\r\n"));
        if (high[ix] == 0)
            break;
    }
    // print and ask to commit
    if (!ix)
        return;

    for (iy = 0; iy < ix; iy++) {
        sprintf(serbuf, "%2d: %5u - %5u\r\n", iy + 1, low[iy], high[iy]);
        printser(serbuf);
    }
    printser_P(PSTR("Write to V1? (Y/N):"));
    while (inhead == intail)
        readpkt(respget);
    if (inbuf[intail] != 'Y' && inbuf[intail] != 'y') {
        intail++;
        return;
    }
    intail++;
    for (iy = 0; iy < ix; iy++) {
        unsigned char wrsbuf[5];
        // write, commit on the last.
        wrsbuf[0] = 0x80 | iy;
        if (iy == ix - 1)
            wrsbuf[0] |= 0x40;
        wrsbuf[1] = high[iy] >> 8;
        wrsbuf[2] = high[iy];
        wrsbuf[3] = low[iy] >> 8;
        wrsbuf[4] = low[iy];
        makecmd(cmdsend, slice, 0xa, REQWRITESWEEPDEFINITIONS, 5, wrsbuf);
        sendcmd(cmdsend, NORESPONSE, respget);
        sprintf(serbuf, "Wrote %2d: %5u - %5u\r\n", iy + 1, low[iy], high[iy]);
        printser(serbuf);
    }
    // get write response and decode errors.
    for (;;) {
        iy = readpkt(respget);
        if (iy < 6)
            continue;
        if (respget[3] == RESPSWEEPWRITERESULT)
            break;
    }
    if (respget[5]) {
        sprintf(serbuf, "Write Error in index %d\r\n", respget[5] - 1);
        printser(serbuf);
    }
    else
        printser_P(PSTR("Write Successfup\r\n"));
    sweep2();
}

// set default sweeps
static void defaultsweeps() {
    syncresp();
    makecmd(cmdsend, slice, 0xa, REQDEFAULTSWEEPS, 0, NULL);
    sendcmd(cmdsend, NORESPONSE, respget);
    printser_P(PSTR("Default Sweeps Done, Please use infoscan to confirm\r\n"));
}

// show user bytes
static void usershow() {
    int ix;
    // User settings
    makecmd(cmdsend, slice, 0xa, REQUSERBYTES, 0, NULL);
    sendcmd(cmdsend, RESPUSERBYTES, respget);
    char userset[] = "12345678AbCdEFGHJuUtL   ";
    printser_P(PSTR("UserSet: (default) "));
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (respget[5] >> ix) & 1 ? userset[ix] : '_');
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (respget[6] >> ix) & 1 ? userset[ix + 8] : '_');
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (respget[7] >> ix) & 1 ? userset[ix + 16] : '_');
        printser(serbuf);
    }
    printser_P(PSTR("\r\nUserSet: (changed) "));
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (respget[5] >> ix) & 1 ? '_' : userset[ix]);
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (respget[6] >> ix) & 1 ? '_' : userset[ix + 8]);
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (respget[7] >> ix) & 1 ? '_' : userset[ix + 16]);
        printser(serbuf);
    }
    printser_P(PSTR("\r\n"));
}

// Scan for everything I could find in the spec for all devices
static void infoscan() {
    int ix;

    //    sprintf( serbuf, "VN: %d: %02x %02x %02x %02x %02x %02x %02x\r\n", ix, cmdsend[0], cmdsend[1], cmdsend[2], cmdsend[3], cmdsend[4], cmdsend[5], cmdsend[6] );
    printser_P(PSTR("=====INFOSCAN=====\r\n"));
    syncresp();

    // do commands
    makecmd(cmdsend, slice, 0xa, REQVERSION, 0, NULL);
    sendcmd(cmdsend, RESPVERSION, respget);
    printser_P(PSTR("V1 Version: "));

    for (ix = 0; ix < respget[4] - 1; ix++) {
        sprintf(serbuf, "%c", respget[5 + ix] < 127 && respget[5 + ix] > 31 ? respget[5 + ix] : '.');
        printser(serbuf);
    }
    printser_P(PSTR("\r\n"));

    makecmd(cmdsend, slice, 0xa, REQSERIALNUMBER, 0, NULL);
    sendcmd(cmdsend, RESPSERIALNUMBER, respget);
    printser_P(PSTR("V1 SerialNo: "));
    for (ix = 0; ix < respget[4] - 1; ix++) {
        sprintf(serbuf, "%c", respget[5 + ix] < 127 && respget[5 + ix] > 31 ? respget[5 + ix] : '.');
        printser(serbuf);
    }
    printser_P(PSTR("\r\n"));

    makecmd(cmdsend, slice, 0xa, REQBATTERYVOLTAGE, 0, NULL);
    sendcmd(cmdsend, RESPBATTERYVOLTAGE, respget);
    sprintf(serbuf, "BattVolt: %d.%02d\r\n", respget[5], respget[6]);
    printser(serbuf);

    usershow();

    sweep1();
    sweep2();

    // Concealed Display
    makecmd(cmdsend, slice, 0, REQVERSION, 0, NULL);
    sendcmd(cmdsend, RESPVERSION, respget);
    printser_P(PSTR("CD Version: "));
    for (ix = 0; ix < respget[4] - 1; ix++) {
        sprintf(serbuf, "%c", respget[5 + ix] < 127 && respget[5 + ix] > 31 ? respget[5 + ix] : '.');
        printser(serbuf);
    }
    printser_P(PSTR("\r\n"));

    // Remote Audio
    makecmd(cmdsend, slice, 1, REQVERSION, 0, NULL);
    sendcmd(cmdsend, RESPVERSION, respget);
    printser_P(PSTR("RA Version: "));
    for (ix = 0; ix < respget[4] - 1; ix++) {
        sprintf(serbuf, "%c", respget[5 + ix] < 127 && respget[5 + ix] > 31 ? respget[5 + ix] : '.');
        printser(serbuf);
    }
    printser_P(PSTR("\r\n"));

    // Savvy
    makecmd(cmdsend, slice, 2, REQVERSION, 0, NULL);
    sendcmd(cmdsend, RESPVERSION, respget);
    printser_P(PSTR("SV Version: "));
    for (ix = 0; ix < respget[4] - 1; ix++) {
        sprintf(serbuf, "%c", respget[5 + ix] < 127 && respget[5 + ix] > 31 ? respget[5 + ix] : '.');
        printser(serbuf);
    }
    printser_P(PSTR("\r\n"));

    makecmd(cmdsend, slice, 2, REQSERIALNUMBER, 0, NULL);
    sendcmd(cmdsend, RESPSERIALNUMBER, respget);
    printser_P(PSTR("SV SerialNo: "));
    for (ix = 0; ix < respget[4] - 1; ix++) {
        sprintf(serbuf, "%c", respget[5 + ix] < 127 && respget[5 + ix] > 31 ? respget[5 + ix] : '.');
        printser(serbuf);
    }
    printser_P(PSTR("\r\n"));

    makecmd(cmdsend, slice, 2, REQSAVVYSTATUS, 0, NULL);
    sendcmd(cmdsend, RESPSAVVYSTATUS, respget);
    sprintf(serbuf, "SavvyStat: ThreshKPH:%d (unmu ena: throvrd):%d\r\n", respget[5], respget[6]);
    printser(serbuf);
    makecmd(cmdsend, slice, 2, REQVEHICLESPEED, 0, NULL);
    sendcmd(cmdsend, RESPVEHICLESPEED, respget);
    sprintf(serbuf, "SavvyVehSpd: %d kph\r\n", respget[5]);
    printser(serbuf);
    printser_P(PSTR("=====END INFOSCAN=====\r\n"));
}

// Ask for and Display (decoded) alerts
static const unsigned char typ[] = "LAKXU^-v", t;
static void alerts() {
    int ix;
    syncresp();
    printser_P(PSTR("=====ALERTS=====\r\n"));
    makecmd(cmdsend, slice, 0xa, REQSTARTALERTDATA, 0, NULL);
    sendcmd(cmdsend, NORESPONSE, respget);
    for (;;) {
        if (inhead != intail) {
            intail = inhead;
            break;
        }
        printser_P(PSTR("===\r\n"));
        ix = v1alerts;
        while (ix == v1alerts)
            readpkt(respget);
        for (ix = 0; ix < (v1alertout[0][0] & 15); ix++) {
            unsigned char *b = v1alertout[ix], t;
            sprintf(serbuf, "%2d/%2d %5u %3d ^v %3d ", b[0] >> 4, b[0] & 15, b[1] << 8 | b[2], b[3], b[4]);
            printser(serbuf);
            for (t = 0; t < 8; t++)
                if ((b[5] >> t) & 1) {
                    sprintf(serbuf, "%c", typ[t]);
                    printser(serbuf);
                }
            if (b[6] & 0x80) {
                sprintf(serbuf, "!");
                printser(serbuf);
            }
            printser_P(PSTR("\r\n"));
        }
    }
    makecmd(cmdsend, slice, 0xa, REQSTOPALERTDATA, 0, NULL);
    sendcmd(cmdsend, NORESPONSE, respget);
    printser_P(PSTR("=====END ALERTS=====\r\n"));
}

const prog_char sevs2ascii[] PROGMEM = {
      " ~'...17_...j..]"
      "........l...uvJ."
      "`\".^............" 
      "|.......LC...GU0" 
      "-.......=#.....3" 
      "r./.....c..2o.d." 
      "....\\.4......5y9" 
      ".F.Ph.HAtE..b6.8"
    };

static void showinfdisp() {
    int ix;
    sprintf(serbuf, "%c%c %02x %02x ", pgm_read_byte(sevs2ascii + (respget[5] & 0x7f)), respget[5] & 0x80 ? 'o' : ' ', respget[5],
      respget[6] ^ respget[5]);
    printser(serbuf);
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (respget[7] >> ix) & 1 ? '*' : '.');
        printser(serbuf);
    }
    printser(" ");
        
    for (ix = 0; ix < 8; ix++)
        if ((respget[8] >> ix) & 1) {
            sprintf(serbuf, "%c", typ[ix]);
            printser(serbuf);
        }
        else
            printser("_");
    printser(" ");
                
    for (ix = 0; ix < 8; ix++)
        if (((respget[8] ^ respget[9]) >> ix) & 1) {
            sprintf(serbuf, "%c", typ[ix]);
            printser(serbuf);
        }
        else
            printser("_");
    printser(" ");
    const unsigned char inf2[] = "MHUDEC-=";
    //bit 0-7: Mute, TSHold, SysUp, DispOn, Euro, Custom, -, -
    for (ix = 0; ix < 8; ix++)
        if ((respget[10] >> ix) & 1) {
            sprintf(serbuf, "%c", inf2[ix]);
            printser(serbuf);
        }
        else
            printser("_");
    printser_P(PSTR("\r\n"));
}

// This setup can be used for Savvy override and unmute setting, by changing the command
// set scan mode
// 1=AllBogeys, 2=Logic, 3=AdvancedLogic
static void setmode(unsigned char mode) {
    syncresp();
    makecmd(cmdsend, slice, 0xa, REQCHANGEMODE, 1, &mode);
    sendcmd(cmdsend, NORESPONSE, respget);
}

// no param command without response
static void quickcommand(cmd) {
    syncresp();
    makecmd(cmdsend, slice, 0xa, cmd, 0, NULL);
    sendcmd(cmdsend, NORESPONSE, respget);
}

// prototype - need to add set/reset dialog and edit the buffer
static void userbytes() {
    syncresp();
    printser_P(PSTR("Old\r\n"));
    usershow();
    printser_P(PSTR("Updating\r\n"));

    // INCOMPLETE
    // Edit userbytes at respget[5-7]

    makecmd(cmdsend, slice, 0xa, REQWRITEUSERBYTES, 6, &respget[5]);
    sendcmd(cmdsend, NORESPONSE, respget);
    printser_P(PSTR("New\r\n"));
    usershow();
}

/*========================================================================*/
#ifdef STANDALONE
int main()
#else
void init()
#endif
{
    cli();
    v1state = inmsgstate = inmsglen = polarity = bitcnt = 0;

    // UART init
#include <util/setbaud.h>
    UBRR2H = UBRR1H = UBRR0H = UBRRH_VALUE;
    UBRR2L = UBRR1L = UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR2A = UCSR1A = UCSR0A = _BV(U2X1);
#endif
    UCSR2C = UCSR1C = UCSR0C = _BV(UCSZ10) | _BV(UCSZ11);       // 8N1
    UCSR2B = UCSR0B = _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0);    // Enable TX and RX
    // for UART1, only TX is enabled, and only when sending in the right timeslice

    // Timer init
    GTCCR = 0;                  //_BV(PSR10);         /* reset prescaler */

    // trigger on falling edge (default), noise cancel
    // lower 3 bits is div, off,1,8,64,256,1024,extfall,extris ; CS12,11,10
    TCCR4B = _BV(ICNC1) | _BV(CS11);

    // clear and enable Input Capture interrupt 
    TIFR4 |= _BV(ICF4) | _BV(TOV4) | _BV(OCF4A) | _BV(OCF4B);
    TIMSK4 = _BV(ICIE4);        // enable input capture only

    sei();                      // enable interrupts
    // and sleep between events
    set_sleep_mode(SLEEP_MODE_IDLE);


    int ret;
    unsigned char lastdisp[12];
        
    printser_P(PSTR("V1MegaTool\r\n"));
    for(;;) {
        for (;;) {                  // get at least one inf packet
            ret = readpkt(respget);
            if (ret < 5)
                continue;
            if (respget[3] == INFDISPLAYDATA)
                break;
        }
        // should give Not Ready message if timeslice holdoff.
        printser_P(PSTR("A-alerts, I-infoscan, D-DefaultSweep, S-SetSweeps. T-transparent, V-ViewDisplay\r\n"));
        while (inhead == intail)
            readpkt(respget);
    
        switch (inbuf[intail++]) {
/*
quickcommand(REQFACTORYDEFAULT);
quickcommand(REQTURNOFFMAINDISPLAY);
quickcommand(REQTURNONMAINDISPLAY);
quickcommand(REQMUTEON);
quickcommand(REQMUTEOFF);
setmode(1); // all bogeys
setmode(2); // logic
setmode(3); // advanced logic
*/

        case 'A':
        case 'a':
            alerts();
            break;
        case 'I':
        case 'i':
            infoscan();
            break;
        case 'S':
        case 's':
            sweepset();
            break;
        case 'D':
        case 'd':
            defaultsweeps();
            break;
        case 'T':
        case 't':
            transp = 1;             // act like bluetooth port 
            while (transp)
                sleep_mode();
            break;
        case 'V':
        case 'v':
            printser_P(PSTR("Mute (ESP)Hold systemUp mainDisp Euro Custom\r\n"));
            lastdisp[0] = 0;
            while( inhead == intail ) {
                ret = readpkt(respget);
                if (ret < 5)
                    continue;
                if (respget[3] == INFDISPLAYDATA && memcmp( lastdisp, respget, 12 )) {
                    showinfdisp();
                    memcpy( lastdisp, respget, 12 );
                }
            }
            break;
        default:
            break;
        }
    }
}


