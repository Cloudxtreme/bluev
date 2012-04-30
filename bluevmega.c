#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>

//#define F_CPU         18432000
#define F_CPU  	16000000
#define BAUD 	57600

#define PRESCALE1 8
#define BITTIME(x)		( ((x) * F_CPU) / PRESCALE1 / BAUD)

#include <util/delay.h>

/*-------------------------------------------------------------------------*/
#define INMSGSIZE 24
static unsigned char inmsgbuf[INMSGSIZE], inmsglen, inmsgcks;
static unsigned char slice = 4;
static unsigned char transp = 0;

static unsigned char nochecksum = 0;
#define ANYSLICE
// Received character from bluetooth
static unsigned char inmsgstate;
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

static unsigned char inbuf[256], inhead = 0, intail = 0;
static unsigned char v1buf[256], v1head = 0, v1tail = 0;
ISR(USART0_RX_vect)
{
    if (transp) {
        inchar(UDR0);
        return;
    }
    inbuf[inhead++] = UDR0;
}

/*-------------------------------------------------------------------------*/
static unsigned int bitcnt;
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
static unsigned char v1state, thislen;
unsigned char infDisp[] = "\xaa\xd8\xea\x31\x09";       // put in Flash?
void dostate(unsigned char val)
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

#define REQVERSION (1)
#define RESPVERSION (2)
#define REQSERIALNUMBER (3)
#define RESPSERIALNUMBER (4)

#define REQUSERBYTES (0x11)
#define RESPUSERBYTES (0x12)
#define REQWRITEUSERBYTES (0x11)
#define REQFACTORYDEFAULT (0x14)

#define REQWRITESWEEPDEFINITIONS (0x16)
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

unsigned char tstnocks = 0;

int makecmd(unsigned char *buf, unsigned char src, unsigned char dst, unsigned char pkt, unsigned char len, unsigned char *param)
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

unsigned char v1idd = 0, v1infdisplaydata[8];
unsigned char v1alerts = 0, v1alerttemp[16][7], v1alertout[16][7];
unsigned char cddr = 0;

static int readv1rx(void)
{
    while (v1head == v1tail)
        sleep_mode();
    return v1buf[v1tail++];
}

static char serbuf[256];
static void printser(char *str)
{
    while (*str) {
        while (!(UCSR0A & 0x20));
        UDR0 = *str++;
    }
}

int readpkt(unsigned char *buf)
{
    unsigned char len, ix;
    buf[0] = 0;
    for (;;) {
        while (buf[0] != 0xaa)  // SOF
            buf[0] = readv1rx();
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
            printser("\n");
#endif
        }
        return len;
    }
}

int sendcmd(unsigned char *thiscmd, unsigned char resp, unsigned char *buf)
{
    unsigned char ix;
    int ret;

    memcpy(inmsgbuf, thiscmd, thiscmd[4] + 6);
    inmsglen = thiscmd[4] + 6;
    inmsgstate = 4;

#define ECHOTIME 8
    for (ix = 0; ix < ECHOTIME; ix++) { // look for command on bus
        ret = readpkt(buf);
        //      sprintf( serbuf, "%d: %02x %02x %02x %02x %02x %02x %02x\n", ret, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6] );
        if (ret < thiscmd[4] + 6)
            continue;
        if (!memcmp(thiscmd, buf, ret))
            break;
    }
    if (ix == ECHOTIME)
        return -1;

    if (resp == NORESPONSE)
        return 0;

    // look for response
#define RESPTIME 20
    for (ix = 0; ix < RESPTIME; ix++) {
        ret = readpkt(buf);
        if (ret < 6)
            continue;
        switch (buf[3]) {
        case RESPUNSUPPORTEDPACKET:
            sprintf(serbuf, "Unsupported Packet\n");
            printser(serbuf);
            break;
        case RESPREQUESTNOTPROCESSED:
            sprintf(serbuf, "Request Not Processed %02x\n", buf[5]);
            printser(serbuf);
            // maybe abort, return -3?
            break;
        case INFV1BUSY:
#if 0
            printser("V1Busy:");
            for (ix = 0; ix < buf[4] - 1; ix++) {
                sprintf(serbuf, " %02x", buf[5 + ix]);
                printser(serbuf);
            }
            printser(serbuf, "\n");
#endif
            ix = 0;             // reset timer
            break;
        case RESPDATAERROR:
            sprintf(serbuf, "Data Error %02x\n", buf[5]);
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

/*========================================================================*/
void v1test(void)
{
    unsigned char buf[22], sendbuf[22], ix;
    int ret;

    while (inhead == intail)
        sleep_mode();

    printser("V1MegaTool\n");

    for (;;) {                  // get at least one inf packet
        ret = readpkt(buf);
        if (ret < 5)
            continue;
        if (buf[3] == INFDISPLAYDATA)
            break;
    }
    // should worry about timeslice holdoff.

    //    sprintf( serbuf, "VN: %d: %02x %02x %02x %02x %02x %02x %02x\n", ix, sendbuf[0], sendbuf[1], sendbuf[2], sendbuf[3], sendbuf[4], sendbuf[5], sendbuf[6] );
#if 0
    unsigned char cmparm[] = { 3 };
    ix = makecmd(sendbuf, slice, 0xa, REQCHANGEMODE, 1, cmparm);
    sendcmd(sendbuf, NORESPONSE, buf);
#endif
    // send one request version to clear out the incoming packet buffer
    makecmd(sendbuf, slice, 0xa, REQVERSION, 0, NULL);
    sendcmd(sendbuf, NORESPONSE, buf);
    for (;;) {
        ret = readpkt(buf);
        if (ret < 6)
            continue;
        if (buf[3] == RESPVERSION)
            break;
    }
    // do commands
    makecmd(sendbuf, slice, 0xa, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printser("V1 Version: ");

    for (ix = 0; ix < buf[4] - 1; ix++) {
        sprintf(serbuf, "%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
        printser(serbuf);
    }
    printser("\n");

    makecmd(sendbuf, slice, 0xa, REQSERIALNUMBER, 0, NULL);
    sendcmd(sendbuf, RESPSERIALNUMBER, buf);
    printser("V1 SerialNo: ");
    for (ix = 0; ix < buf[4] - 1; ix++) {
        sprintf(serbuf, "%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
        printser(serbuf);
    }
    printser("\n");

    makecmd(sendbuf, slice, 0xa, REQBATTERYVOLTAGE, 0, NULL);
    sendcmd(sendbuf, RESPBATTERYVOLTAGE, buf);
    sprintf(serbuf, "BattVolt: %d.%02d\n", buf[5], buf[6]);
    printser(serbuf);

    // User settings
    makecmd(sendbuf, slice, 0xa, REQUSERBYTES, 0, NULL);
    sendcmd(sendbuf, RESPUSERBYTES, buf);
    char userset[] = "12345678AbCdEFGHJuUtL   ";
    printser("UserSet: (default) ");
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (buf[5] >> ix) & 1 ? userset[ix] : '_');
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (buf[6] >> ix) & 1 ? userset[ix + 8] : '_');
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (buf[7] >> ix) & 1 ? userset[ix + 16] : '_');
        printser(serbuf);
    }
    printser("\nUserSet: (changed) ");
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (buf[5] >> ix) & 1 ? '_' : userset[ix]);
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (buf[6] >> ix) & 1 ? '_' : userset[ix + 8]);
        printser(serbuf);
    }
    for (ix = 0; ix < 8; ix++) {
        sprintf(serbuf, "%c", (buf[7] >> ix) & 1 ? '_' : userset[ix + 16]);
        printser(serbuf);
    }
    printser("\n");

    // Sweep Sections and Definitions
    makecmd(sendbuf, slice, 0xa, REQSWEEPSECTIONS, 0, NULL);
    sendcmd(sendbuf, RESPSWEEPSECTIONS, buf);
    printser("SweepSct:\n");
    sprintf(serbuf, "+%d/%d %5u - %5u\n", buf[5] >> 4, buf[5] & 15, buf[8] << 8 | buf[9], buf[6] << 8 | buf[7]);
    printser(serbuf);
    if (buf[4] > 6) {
        sprintf(serbuf, "+%d/%d %5u -:%5u\n", buf[10] >> 4, buf[10] & 15, buf[13] << 8 | buf[14], buf[11] << 8 | buf[12]);
        printser(serbuf);
    }
    if (buf[4] > 11) {
        sprintf(serbuf, "+%d/%d %5u - %5u\n", buf[15] >> 4, buf[15] & 15, buf[18] << 8 | buf[19], buf[16] << 8 | buf[17]);
        printser(serbuf);
    }
    unsigned int nswppkt = (buf[5] & 15);
    for (ix = 1; ix < nswppkt / 3;) {
        // read additional 0x23 packet, print it out
        ret = readpkt(buf);
        if (ret < 5)
            continue;
        if (buf[3] != 0x23)
            continue;
        ix++;
        sprintf(serbuf, "+%d/%d %5u - %5u\n", buf[5] >> 4, buf[5] & 15, buf[8] << 8 | buf[9], buf[6] << 8 | buf[7]);
        printser(serbuf);

        if (buf[4] > 6) {
            sprintf(serbuf, "+%d/%d %5u -:%5u\n", buf[10] >> 4, buf[10] & 15, buf[13] << 8 | buf[14], buf[11] << 8 | buf[12]);
            printser(serbuf);
        }
        if (buf[4] > 11) {
            sprintf(serbuf, "+%d/%d %5u - %5u\n", buf[15] >> 4, buf[15] & 15, buf[18] << 8 | buf[19], buf[16] << 8 | buf[17]);
            printser(serbuf);
        }
    }
    // sweep definitions must stay within the sections above
    makecmd(sendbuf, slice, 0xa, REQMAXSWEEPINDEX, 0, NULL);
    sendcmd(sendbuf, RESPMAXSWEEPINDEX, buf);
    sprintf(serbuf, "SweepMax: %d\n", buf[5]);
    printser(serbuf);
    unsigned int maxswp = buf[5];
    // read sweep sections
    makecmd(sendbuf, slice, 0xa, REQALLSWEEPDEFINITIONS, 0, NULL);
    sendcmd(sendbuf, RESPSWEEPDEFINITION, buf);
    sprintf(serbuf, "SweepDef: %d Top:%5u Bot:%5u\n", buf[5] & 63, buf[6] << 8 | buf[7], buf[8] << 8 | buf[9]);
    printser(serbuf);
    for (ix = 0; ix < maxswp;) {
        ret = readpkt(buf);
        if (ret < 5)
            continue;
        if (buf[3] != 0x17)
            continue;
        ix++;
        sprintf(serbuf, "SweepDef: %d Top:%5u Bot:%5u\n", buf[5] & 63, buf[6] << 8 | buf[7], buf[8] << 8 | buf[9]);
        printser(serbuf);
    }

    // Concealed Display
    makecmd(sendbuf, slice, 0, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printser("CD Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++) {
        sprintf(serbuf, "%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
        printser(serbuf);
    }
    printser("\n");

    // Remote Audio
    makecmd(sendbuf, slice, 1, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printser("RA Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++) {
        sprintf(serbuf, "%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
        printser(serbuf);
    }
    printser("\n");

    // Savvy
    makecmd(sendbuf, slice, 2, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printser("SV Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++) {
        sprintf(serbuf, "%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
        printser(serbuf);
    }
    printser("\n");

    makecmd(sendbuf, slice, 2, REQSERIALNUMBER, 0, NULL);
    sendcmd(sendbuf, RESPSERIALNUMBER, buf);
    printser("SV SerialNo: ");
    for (ix = 0; ix < buf[4] - 1; ix++) {
        sprintf(serbuf, "%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
        printser(serbuf);
    }
    printser("\n");

    makecmd(sendbuf, slice, 2, REQSAVVYSTATUS, 0, NULL);
    sendcmd(sendbuf, RESPSAVVYSTATUS, buf);
    sprintf(serbuf, "SavvyStat: ThreshKPH:%d (unmu ena: throvrd):%d\n", buf[5], buf[6]);
    printser(serbuf);
    makecmd(sendbuf, slice, 2, REQVEHICLESPEED, 0, NULL);
    sendcmd(sendbuf, RESPVEHICLESPEED, buf);
    sprintf(serbuf, "SavvyVehSpd: %d kph\n", buf[5]);
    printser(serbuf);
    makecmd(sendbuf, slice, 0xa, REQSTARTALERTDATA, 0, NULL);
    sendcmd(sendbuf, NORESPONSE, buf);
    for (;;) {
        printser("===\n");
        ix = v1alerts;
        while (ix == v1alerts)
            readpkt(buf);
        for (ix = 0; ix < (v1alertout[0][0] & 15); ix++) {
            unsigned char *b = v1alertout[ix];
            unsigned char typ[] = "LAKXU^-v", t;
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
            printser("\n");
        }
    }
    return;
    /*

       char sevs2ascii[] = {
       ' ', '~', '.', '.', '.', '.', '1', '7',
       '_', '.', '.', '.', 'j', '.', '.', ']',
       '.', '.', '.', '.', '.', '.', '.', '.',
       'l', '.', '.', '.', 'u', 'v', 'J', '.',

       '.', '.', '.', '^', '.', '.', '.', '.',
       '.', '.', '.', '.', '.', '.', '.', '.',
       '|', '.', '.', '.', '.', '.', '.', '.',
       'L', 'C', '.', '.', '.', 'G', 'U', '0',

       '-', '.', '.', '.', '.', '.', '.', '.',
       '=', '#', '.', '.', '.', '.', '.', '3',
       'r', '.', '/', '.', '.', '.', '.', '.',
       'c', '.', '.', '2', 'o', '.', 'd', '.',

       '.', '.', '.', '.', '\\', '.', '4', '.',
       '.', '.', '.', '.', '.', '5', 'y', '9',
       '.', 'F', '.', 'P', 'h', '.', 'H', 'A',
       't', 'E', '.', '.', 'b', '6', '.', '8'
       };
       sprintf( serbuf,"Disp: %c%c %02x %02x ", sevs2ascii[buf[5] & 0x7f], buf[5] & 0x80 ? 'o' : ' ', buf[5], buf[6] ^ buf[5]);
       for (ix = 0; ix < 8; ix++)
       sprintf( serbuf,"%c", (buf[7] >> ix) & 1 ? '*' : '.');

       //bit 0-7: Laser, Ka, K, X, -, Front, Side, Rear
       sprintf( serbuf," %02x %02x", buf[8], buf[9] ^ buf[8]);
       //bit 0-7: Mute, TSHold, SysUp, DispOn, Euro, Custom, -, -
       sprintf( serbuf," %02x\n", buf[10]);
       break;

     */
}

/*-------------------------------------------------------------------------*/
int main(void)
{

    /* for testing
       inmsgstate = 4;
       strcpy_P( inmsgbuf, PSTR("\xaa\xda\xe4\x41\x01\xaa\xab") );
       inmsglen = 7;
     */
    v1state = inmsgstate = inmsglen = 0;

    // UART init
#include <util/setbaud.h>
    UBRR1H = UBRRH_VALUE;
    UBRR1L = UBRRL_VALUE;
#if USE_2X
    UCSR1A |= (1 << U2X1);
#else
    UCSR1A &= ~(1 << U2X1);
#endif

    UCSR1C = _BV(UCSZ10) | _BV(UCSZ11); // 8N1
    //    UCSR1B = _BV(RXEN1) | _BV(RXCIE1);     // Enable Receive

    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    UCSR0C = _BV(UCSZ00) | _BV(UCSZ01); // 8N1
    UCSR0B = _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0);     // Enable TX and RX

    UBRR2H = UBRRH_VALUE;
    UBRR2L = UBRRL_VALUE;
#if USE_2X
    UCSR2A |= (1 << U2X2);
#else
    UCSR2A &= ~(1 << U2X2);
#endif
    UCSR2C = _BV(UCSZ20) | _BV(UCSZ21); // 8N1
    UCSR2B = _BV(TXEN2) | _BV(RXEN2) | _BV(RXCIE2);     // Enable TX and RX

    // Timer init
    GTCCR = 0;                  //_BV(PSR10);         /* reset prescaler */
    TCNT4 = 0;                  /* reset counter value */
    polarity = bitcnt = 0;

    // trigger on falling edge (default), noise cancel
    // lower 3 bits is div, off,1,8,64,256,1024,extfall,extris ; CS12,11,10
    TCCR4B = _BV(ICNC1) | _BV(CS11);

    // clear and enable Input Capture interrupt 
    TIFR4 |= _BV(ICF4) | _BV(TOV4) | _BV(OCF4A) | _BV(OCF4B);
    TIMSK4 |= _BV(ICIE4);       // enable input capture only

    sei();                      // enable interrupts
    // and sleep between events
    set_sleep_mode(SLEEP_MODE_IDLE);

    v1test();

    for (;;) {
        sleep_mode();
    }
}
