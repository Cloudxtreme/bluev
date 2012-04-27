#include <stdio.h>
#include <stdlib.h>
#include <string.h>

unsigned char cmds[][22] = {
{ 0xaa, 0xda, 0xe4, 0x01, 0x01, 0x6a, 0xab}, // 0 software version
{ 0xaa, 0xda, 0xe4, 0x03, 0x01, 0x6c, 0xab}, // 1 serial number
{ 0xaa, 0xda, 0xe4, 0x11, 0x01, 0x7a, 0xab}, // 2 user bytes
{ 0xaa, 0xda, 0xe4, 0x14, 0x01, 0x7d, 0xab}, // 3 factory defaults
{ 0xaa, 0xda, 0xe4, 0x16, 0x01, 0x7f, 0xab}, // 4 sweep definitions
{ 0xaa, 0xda, 0xe4, 0x18, 0x01, 0x81, 0xab}, // 5 default sweeps
{ 0xaa, 0xda, 0xe4, 0x19, 0x01, 0x82, 0xab}, // 6 max sweep index
{ 0xaa, 0xda, 0xe4, 0x22, 0x01, 0x8b, 0xab}, // 7 sweep sections
{ 0xaa, 0xda, 0xe4, 0x32, 0x01, 0x9b, 0xab}, // 8 main display off
{ 0xaa, 0xda, 0xe4, 0x33, 0x01, 0x9c, 0xab}, // 9 main display on
{ 0xaa, 0xda, 0xe4, 0x34, 0x01, 0x9d, 0xab}, // 10 mute on
{ 0xaa, 0xda, 0xe4, 0x35, 0x01, 0x9e, 0xab}, // 11 mute off
{ 0xaa, 0xda, 0xe4, 0x41, 0x01, 0xaa, 0xab}, // 12 alert data on
{ 0xaa, 0xda, 0xe4, 0x42, 0x01, 0xab, 0xab}, // 13 alert data off
{ 0xaa, 0xda, 0xe4, 0x62, 0x01, 0xcb, 0xab}, // 14 battery voltage
{ 0xaa, 0xda, 0xe4, 0x36, 0x02, 0x01, 0xa1, 0xab}, // 15 all bogeys mode
{ 0xaa, 0xda, 0xe4, 0x36, 0x02, 0x02, 0xa2, 0xab}, // 16 logic mode
{ 0xaa, 0xda, 0xe4, 0x36, 0x02, 0x03, 0xa3, 0xab}, // 17 advanced logic mode
{ 0xaa, 0xd0, 0xe4, 0x01, 0x01, 0x60, 0xab}, // 18 CD software version

{ 0xaa, 0xd1, 0xe4, 0x01, 0x01, 0x61, 0xab}, // 19 RA software version

{ 0xaa, 0xd2, 0xe4, 0x01, 0x01, 0x62, 0xab}, // 20 savvy software version
{ 0xaa, 0xd2, 0xe4, 0x03, 0x01, 0x64, 0xab}, // 21 savvy serial number
{ 0xaa, 0xd2, 0xe4, 0x71, 0x01, 0xd2, 0xab}, // 22 savvy status
{ 0xaa, 0xd2, 0xe4, 0x73, 0x01, 0xd4, 0xab}, // 23 savvy vehicle speed

// cks not verified
{ 0xaa, 0xd2, 0xe4, 0x76, 0x02, 0x00, 0xe0, 0xab}, // 24 savvy disable unmute
{ 0xaa, 0xd2, 0xe4, 0x76, 0x02, 0x01, 0xe1, 0xab}, // 25 savvy enable unmute

//  one byte param
{ 0xaa, 0xd2, 0xe4, 0x75, 0x02, 0x00, 0xdf, 0xab}, // 26 savvy thumbwheel override
//  mute threshold speed, KPH, 0=never 0xff=always, fix cks

//  complex param
//{ 0xaa, 0xda, 0xe4, 0x13, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x--, 0xab}, //  write user bytes
//{ 0xaa, 0xda, 0xe4, 0x15, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x--, 0xab}, //  write sweep definition
};
//  Note - an 0xaa 0xd4 0xe? packet for version or serial number might be responded to



FILE *fp = NULL;
unsigned char nochecksum = 0;
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
        return ++len;
    }
}

int sendcmd( unsigned char *thiscmd, unsigned char resp, unsigned char *buf ) {
    unsigned char ix;
    int ret;
    // checksum always included, fixme
     fwrite( thiscmd, thiscmd[4]+6, 1, fp ); // write command
#define ECHOTIME 8
     for (ix = 0; ix < ECHOTIME ; ix++) { // look for command on bus
        ret = readpkt(buf);
	//	printf( "%d: %02x %02x %02x %02x %02x %02x %02x\n", ret, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6] );
        if (ret < thiscmd[4]+6)
            continue;
	if( !memcmp(thiscmd,buf,ret) )
	    break;
    }
    if( ix == ECHOTIME )
	return -1;

    // look for response
#define RESPTIME 20
    for (ix = 0; ix < RESPTIME ; ix++) { 
        ret = readpkt(buf);
        if (ret < 6)
            continue;
	switch( buf[3] ) {
        case 0x64:
            printf("Unsupported Packet\n");
            break;
        case 0x65:
            printf("Request Not Processed %02x\n", buf[5]);
	    // maybe abort, return -3?
            break;
        case 0x66:
            printf("V1Busy:");
            for (ix = 0; ix < buf[4] - 1; ix++)
                printf(" %02x", buf[5 + ix]);
	    ix = 0; // reset timer
            printf("\n");
            break;
        case 0x67:
            printf("Data Error %02x\n", buf[5]);
            break;
	}

	if( buf[3] == resp )
	    break;
    }
    if( ix == RESPTIME )
	return -2;
    return 0;
}

int main(int argc, char *argv[])
{
    unsigned char buf[24], ix;
    int ret;
    fp = fopen("/dev/ttyUSB0", "a+b");
    if (!fp)
        fp = fopen("/dev/rfcomm0", "a+b");
    if (!fp)
        return -1;

    // need to flush input
    fseek( fp, 0, SEEK_END );

    for (;;) { // get at least one inf packet
        ret = readpkt(buf);
        if (ret < 5)
            continue;
	if( buf[3] == 0x31 )
	    break;
    }

    do {
	ret = sendcmd( cmds[0] , 2, buf );
    } while( ret < 0 );
    printf("Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
	printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    sendcmd( cmds[1] , 4, buf );
    printf("SerialNo: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
	printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    sendcmd( cmds[14] , 0x63, buf );
    printf("BattVolt: %d.%02d\n", buf[5], buf[6]);

    sendcmd( cmds[2] , 0x12, buf );
    char userset[] = "12345678AbCdEFGHJuUtL   ";
    printf("UserSet: (default)");
    for (ix = 0; ix < 8; ix++)
	printf("%c", (buf[5] >> ix) & 1 ? userset[ix] : '_');
    for (ix = 0; ix < 8; ix++)
	printf("%c", (buf[6] >> ix) & 1 ? userset[ix + 8] : '_');
    for (ix = 0; ix < 8; ix++)
	printf("%c", (buf[7] >> ix) & 1 ? userset[ix + 16] : '_');
    printf("\nUserSet: (changed)");
    for (ix = 0; ix < 8; ix++)
	printf("%c", (buf[5] >> ix) & 1 ? '_' : userset[ix]);
    for (ix = 0; ix < 8; ix++)
	printf("%c", (buf[6] >> ix) & 1 ? '_' : userset[ix + 8]);
    for (ix = 0; ix < 8; ix++)
	printf("%c", (buf[7] >> ix) & 1 ? '_' : userset[ix + 16]);
    printf("\n");


    sendcmd( cmds[18] , 2, buf );
    printf("CD Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
	printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");
    sendcmd( cmds[19] , 2, buf );
    printf("RA Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
	printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    sendcmd( cmds[20] , 2, buf );
    printf("SV Version: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
	printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    sendcmd( cmds[21] , 4, buf );
    printf("SV SerialNo: ");
    for (ix = 0; ix < buf[4] - 1; ix++)
	printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');
    printf("\n");

    sendcmd( cmds[22] , 0x72, buf );
    printf("SavvyStat: ThreshKPH:%d (unmu ena: throvrd):%d\n", buf[5], buf[6]);

    sendcmd( cmds[23] , 0x74, buf );
    printf("SavvyVehSpd: %d kph\n", buf[5]);

    fclose( fp );
    return 0;
	    /*

        printf("%x %x ", buf[1] & 15, buf[2] & 15);

        switch (buf[3]) {
        case 0x17:
            printf("SweepDef: %d Top:%5d Bot:%5d\n", buf[5], buf[6] << 8 | buf[7], buf[8] << 8 | buf[9]);
            break;
        case 0x20:
            printf("SweepMax: %d\n", buf[5]);
            break;
        case 0x21:
            printf("SweepWriteResult: %d\n", buf[5]);
            break;
        case 0x23:
            printf("SweepSct:\n");
            printf("+%d/%d %5d - %5d\n", buf[5] >> 4, buf[5] & 15, buf[8] << 8 | buf[9], buf[6] << 8 | buf[7]);
            if (buf[4] > 6)
                printf("+%d/%d %5d -:%5d\n", buf[10] >> 4, buf[10] & 15, buf[13] << 8 | buf[14], buf[11] << 8 | buf[12]);
            if (buf[4] > 11)
                printf("+%d/%d %5d - %5d\n", buf[15] >> 4, buf[15] & 15, buf[18] << 8 | buf[19], buf[16] << 8 | buf[17]);
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
        case 0x72:
            printf("SavvyStat: ThreshKPH:%d (unmu ena: throvrd):%d\n", buf[5], buf[6]);
            break;
        case 0x74:
            printf("SavvyVehSpd: %d kph\n", buf[5]);
            break;

        case 0x14:             //fac default
        case 0x16:             //allsweeps
        case 0x18:             //defsweeps
        case 0x19:             //maxsweepindex
        case 0x22:             //sweepsects
            break;

        case 0x13:             //WriteUserBytes p6
        case 0x15:             //WriteSweepDef p5
        case 0x36:             //changeMode p1
	    */
}
