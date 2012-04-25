#include <stdio.h>
#include <stdlib.h>


// 12345678AbCdEFGHIJuXtL-- ------------------------
// byte 0, bit 0 - 7, byte 1, etc.

int main(int argc, char *argv[])
{
    unsigned char buf[24], len;
    int ret;
    FILE *fp = fopen("/dev/ttyUSB0", "rb");
    if( !fp )
	fp = fopen("/dev/rfcomm0", "rb");
    if( !fp)
	return -1;
    for (;;) {
        printf("\n");
        len = 0;
        buf[0] = 0;
        while (buf[0] != 0xaa)
            while (1 != (ret = fread(&buf[0], 1, 1, fp)))
		if( ret < 0 )
		    return ret;

        printf("SOF-");

        while (1 != fread(&buf[1], 1, 1, fp));
        printf("%02x-", buf[1]);
        if ((buf[1] & 0xf0) != 0xd0)
            continue;

        while (1 != fread(&buf[2], 1, 1, fp));
        printf("%02x ", buf[2]);
        if ((buf[2] & 0xf0) != 0xe0)
            continue;

        while (1 != fread(&buf[3], 1, 1, fp));
        printf("%02x ", buf[3]);

        while (1 != fread(&buf[4], 1, 1, fp));
        printf("%02x ", buf[4]);

        if (buf[4] > 20)
            continue;
        unsigned char ix;
        len = 5;
        for (ix = 0; ix < buf[4]; ix++) {
            while (1 != fread(&buf[len], 1, 1, fp));
            if (ix == buf[4] - 1)
                printf(" ");
            printf("%02x", buf[len]);
            len++;
        }
        unsigned int cks = 0;
        for (ix = 0; ix < len - 1; ix++)
            cks += buf[ix];

        printf("=(%02x) ", cks & 0xff);
        len++;

        while (1 != fread(&buf[len], 1, 1, fp));
        printf("%02x ", buf[len]);

        for (ix = 0; ix < buf[4] - 1; ix++)
            printf("%c", buf[5 + ix] < 127 && buf[5 + ix] > 31 ? buf[5 + ix] : '.');

    }

}
