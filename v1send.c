#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SLOT (4)

unsigned char cmdbuf[24] = "\xaa\xda\xe0\x00\x01\x00\xab";
unsigned char chmbuf[24] = "\xaa\xda\xe0\x36\x02\x00\x00\xab";

unsigned char optable[] = { 0x00,
			    0x01,0x03,0x11,0x16,0x18,0x19,0x22,0x32,0x33,0x34,0x35,0x41,0x42, 0x62, 0x71,0x73,0x36,0x36,0x36 };

int main(int argc, char *argv[]) {
    int op = -1;
    if( argc > 1 )
	op = atoi( argv[1] );
    if( op < 1 || op > sizeof(optable) ) {
	printf( 
"1 - Version\n"
"2 - SerNo\n"
"3 - UserBytes\n"
"4 - SweepDefs\n"
"5 - DefSweeps\n"
"6 - MaxSwpIdx\n"
"7 - SweepSects\n"
"8 - MainDispOFF\n"
"9 - MainDispON\n"
"10 - MuteON\n"
"11 - MuteOFF\n"
"12 - AlertDatON\n"
"13 - AlertDatOFF\n"
"14 - BattVolt\n"
"15 - SavvyStat\n"
"16 - SavvyVehSPd\n"

"17 - AllBogeys\n"
"18 - Logic\n"
"19 - AdvancedLogic\n"

		);
	return -1;
    }


    if( optable[op] != 0x36 )
	cmdbuf[3] = optable[op];
    else {
	memcpy(cmdbuf,chmbuf,10);
	cmdbuf[5] = op - 16;
    }
    cmdbuf[2] += SLOT;

    unsigned char ix, cks = 0;
    for( ix = 0; ix < cmdbuf[4]+4; ix++ )
	cks += cmdbuf[ix];
    cmdbuf[ix] = cks;

    FILE *fp = fopen( "/dev/ttyUSB0", "wb" );
    fwrite( cmdbuf,cmdbuf[4]+6,1,fp );
    fclose(fp);
    return 0;
}