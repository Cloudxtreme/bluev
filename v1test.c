#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

FILE *fp = NULL;
unsigned char nochecksum = 0;

unsigned char slice = 4;
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
    if (nochecksum && dst == 0x0a)
        dst = 9;
    if (dst == 9 && !nochecksum)
        dst = 0x0a;
    *b++ = 0xd0 + dst;
    *b++ = 0xe0 + src;
    *b++ = pkt;
    *b++ = len + !nochecksum;
    while (l--)
        *b++ = *param++;
    if (!nochecksum) {
        unsigned char cks = 0, ix = 0;
        for (ix = 0; ix < len + 5; ix++)
            cks += buf[ix];
        *b++ = cks;
    }
    *b++ = 0xab;
    return b - buf;
};

int readpkt(unsigned char *buf)
{
    unsigned char len, ix;
    int ret;
    buf[0] = 0;
    for (;;) {
        while (buf[0] != 0xaa)  // SOF
            while (1 != (ret = fread(&buf[0], 1, 1, fp)))
                if (ret < 0)
                    return ret;
        while (1 != fread(&buf[1], 1, 1, fp));  // target
        if ((buf[1] & 0xf0) != 0xd0)
            continue;
        while (1 != fread(&buf[2], 1, 1, fp));  // source
        if ((buf[2] & 0xf0) != 0xe0)
            continue;
        if ((buf[2] & 15) == 0xa)
            nochecksum = 0;
        else if ((buf[2] & 15) == 9)
            nochecksum = 1;
        while (1 != fread(&buf[3], 1, 1, fp));  // packet id
        while (1 != fread(&buf[4], 1, 1, fp));  // length
        if (buf[4] > 16)
            continue;

        len = 5;
        for (ix = 0; ix < buf[4]; ix++) {
            while (1 != fread(&buf[len], 1, 1, fp));
            len++;
        }
        if (!nochecksum) {
            unsigned char cks = 0;
            for (ix = 0; ix < len - 1; ix++)
                cks += buf[ix];
            if (buf[len - 1] != (cks & 0xff))
                return -2;      // continue; ???
        }
        while (1 != fread(&buf[len], 1, 1, fp));
        if (buf[len++] != 0xab)
            continue;

        // save off current alert or inf packet separately
        // 0x31  0x43 0x61
        if (buf[3] == INFDISPLAYDATA) {
        }
        else if (buf[3] == RESPALERTDATA) {
        }
        else if (buf[3] == RESPDATARECEIVED) {
        }
        else
            printf("r: %d: %02x %02x %02x %02x %02x %02x %02x\n", len, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

        return len;
    }
}

int sendcmd(unsigned char *thiscmd, unsigned char resp, unsigned char *buf)
{
    unsigned char ix;
    int ret;

    fwrite(thiscmd, thiscmd[4] + 6, 1, fp);     // write command
#define ECHOTIME 8
    for (ix = 0; ix < ECHOTIME; ix++) { // look for command on bus
        ret = readpkt(buf);
        //      printf( "%d: %02x %02x %02x %02x %02x %02x %02x\n", ret, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6] );
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
#define RESPTIME 50
    for (ix = 0; ix < RESPTIME; ix++) {
        ret = readpkt(buf);
        if (ret < 6)
            continue;
        switch (buf[3]) {
        case RESPUNSUPPORTEDPACKET:
            printf("Unsupported Packet\n");
            break;
        case RESPREQUESTNOTPROCESSED:
            printf("Request Not Processed %02x\n", buf[5]);
            // maybe abort, return -3?
            break;
        case INFV1BUSY:
            printf("V1Busy:");
            for (ix = 0; ix < buf[4] - 1; ix++)
                printf(" %02x", buf[5 + ix]);
            ix = 0;             // reset timer
            printf("\n");
            break;
        case RESPDATAERROR:
            printf("Data Error %02x\n", buf[5]);
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
int main(int argc, char *argv[])
{
    unsigned char buf[22], sendbuf[22], ix;
    int ret;
    fp = fopen("/dev/ttyUSB0", "a+b");
    if (!fp)
        fp = fopen("/dev/rfcomm0", "a+b");
    if (!fp)
        return -1;

    // need to flush input
    fseek(fp, 0, SEEK_END);

    for (;;) {                  // get at least one inf packet
        ret = readpkt(buf);
        if (ret < 5)
            continue;
        if (buf[3] == INFDISPLAYDATA)
            break;
    }
    // should worry about timeslice holdoff.

    //    printf( "VN: %d: %02x %02x %02x %02x %02x %02x %02x\n", ix, sendbuf[0], sendbuf[1], sendbuf[2], sendbuf[3], sendbuf[4], sendbuf[5], sendbuf[6] );
#if 0
    unsigned char cmparm[] = { 3 };
    ix = makecmd(sendbuf, slice, 0xa, REQCHANGEMODE, 1, cmparm);
    sendcmd(sendbuf, NORESPONSE, buf);
#endif

    makecmd(sendbuf, slice, 0xa, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printf("Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
        printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    makecmd(sendbuf, slice, 0xa, REQSERIALNUMBER, 0, NULL);
    sendcmd(sendbuf, RESPSERIALNUMBER, buf);
    printf("SerialNo: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
        printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    makecmd(sendbuf, slice, 0xa, REQBATTERYVOLTAGE, 0, NULL);
    sendcmd(sendbuf, RESPBATTERYVOLTAGE, buf);
    printf("BattVolt: %d.%02d\n", buf[5], buf[6]);

    // User settings
    makecmd(sendbuf, slice, 0xa, REQUSERBYTES, 0, NULL);
    sendcmd(sendbuf, RESPUSERBYTES, buf);
    char userset[] = "12345678AbCdEFGHJuUtL   ";
    printf("UserSet: (default) ");
    for (ix = 0; ix < 8; ix++)
        printf("%c", (buf[5] >> ix) & 1 ? userset[ix] : '_');
    for (ix = 0; ix < 8; ix++)
        printf("%c", (buf[6] >> ix) & 1 ? userset[ix + 8] : '_');
    for (ix = 0; ix < 8; ix++)
        printf("%c", (buf[7] >> ix) & 1 ? userset[ix + 16] : '_');
    printf("\nUserSet: (changed) ");
    for (ix = 0; ix < 8; ix++)
        printf("%c", (buf[5] >> ix) & 1 ? '_' : userset[ix]);
    for (ix = 0; ix < 8; ix++)
        printf("%c", (buf[6] >> ix) & 1 ? '_' : userset[ix + 8]);
    for (ix = 0; ix < 8; ix++)
        printf("%c", (buf[7] >> ix) & 1 ? '_' : userset[ix + 16]);
    printf("\n");

    // Sweep Sections and Definitions
    makecmd(sendbuf, slice, 0xa, REQSWEEPSECTIONS, 0, NULL);
    sendcmd(sendbuf, RESPSWEEPSECTIONS, buf);
    printf("SweepSct:\n");
    printf("+%d/%d %5d - %5d\n", buf[5] >> 4, buf[5] & 15, buf[8] << 8 | buf[9], buf[6] << 8 | buf[7]);
    if (buf[4] > 6)
        printf("+%d/%d %5d -:%5d\n", buf[10] >> 4, buf[10] & 15, buf[13] << 8 | buf[14], buf[11] << 8 | buf[12]);
    if (buf[4] > 11)
        printf("+%d/%d %5d - %5d\n", buf[15] >> 4, buf[15] & 15, buf[18] << 8 | buf[19], buf[16] << 8 | buf[17]);
    unsigned int nswppkt = (buf[5] & 15);
    for (ix = 1; ix < nswppkt / 3;) {
        // read additional 0x23 packet, print it out
        ret = readpkt(buf);
        if (ret < 5)
            continue;
        if (buf[3] != 0x23)
            continue;
        ix++;
        printf("+%d/%d %5d - %5d\n", buf[5] >> 4, buf[5] & 15, buf[8] << 8 | buf[9], buf[6] << 8 | buf[7]);
        if (buf[4] > 6)
            printf("+%d/%d %5d -:%5d\n", buf[10] >> 4, buf[10] & 15, buf[13] << 8 | buf[14], buf[11] << 8 | buf[12]);
        if (buf[4] > 11)
            printf("+%d/%d %5d - %5d\n", buf[15] >> 4, buf[15] & 15, buf[18] << 8 | buf[19], buf[16] << 8 | buf[17]);
    }
    // sweep definitions must stay within the sections above
    makecmd(sendbuf, slice, 0xa, REQMAXSWEEPINDEX, 0, NULL);
    sendcmd(sendbuf, RESPMAXSWEEPINDEX, buf);
    printf("SweepMax: %d\n", buf[5]);
    unsigned int maxswp = buf[5];
    // read sweep sections
    makecmd(sendbuf, slice, 0xa, REQALLSWEEPDEFINITIONS, 0, NULL);
    sendcmd(sendbuf, RESPSWEEPDEFINITION, buf);
    printf("SweepDef: %d Top:%5d Bot:%5d\n", buf[5] & 63, buf[6] << 8 | buf[7], buf[8] << 8 | buf[9]);
    for (ix = 0; ix < maxswp;) {
        ret = readpkt(buf);
        if (ret < 5)
            continue;
        if (buf[3] != 0x17)
            continue;
        ix++;
        printf("SweepDef: %d Top:%5d Bot:%5d\n", buf[5] & 63, buf[6] << 8 | buf[7], buf[8] << 8 | buf[9]);
    }

    // Concealed Display
    makecmd(sendbuf, slice, 0, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printf("CD Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
        printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    // Remote Audio
    makecmd(sendbuf, slice, 1, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printf("RA Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
        printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    // Savvy
    makecmd(sendbuf, slice, 2, REQVERSION, 0, NULL);
    sendcmd(sendbuf, RESPVERSION, buf);
    printf("SV Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
        printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    makecmd(sendbuf, slice, 2, REQSERIALNUMBER, 0, NULL);
    sendcmd(sendbuf, RESPSERIALNUMBER, buf);
    printf("SV SerialNo: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
        printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    makecmd(sendbuf, slice, 2, REQSAVVYSTATUS, 0, NULL);
    sendcmd(sendbuf, RESPSAVVYSTATUS, buf);
    printf("SavvyStat: ThreshKPH:%d (unmu ena: throvrd):%d\n", buf[5], buf[6]);
    makecmd(sendbuf, slice, 2, REQVEHICLESPEED, 0, NULL);
    sendcmd(sendbuf, RESPVEHICLESPEED, buf);
    printf("SavvyVehSpd: %d kph\n", buf[5]);

    fclose(fp);
    return 0;
    /*

       switch (buf[3]) {
       case 0x21:
       printf("SweepWriteResult: %d\n", buf[5]);
       break;
       case 0x31:
       printf("Disp: %c%c %02x %02x ", sevs2ascii[buf[5] & 0x7f], buf[5] & 0x80 ? 'o' : ' ', buf[5], buf[6] ^ buf[5]);
       for (ix = 0; ix < 8; ix++)
       printf("%c", (buf[7] >> ix) & 1 ? '*' : '.');

       //bit 0-7: Laser, Ka, K, X, -, Front, Side, Rear
       printf(" %02x %02x", buf[8], buf[9] ^ buf[8]);
       //bit 0-7: Mute, TSHold, SysUp, DispOn, Euro, Custom, -, -
       printf(" %02x\n", buf[10]);
       break;
       case 0x43:
       printf("Alert: %d/%d %5d %3d^ %3dv %02x %02x\n", buf[5] >> 4, buf[5] & 15, buf[6] << 8 | buf[7], buf[8], buf[9],
       buf[10], buf[11]);
       break;

     */
}
