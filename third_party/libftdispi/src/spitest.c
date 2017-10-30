#include <stdio.h>
#include "ftdispi.h"

int main(int argc, char **argv)
{
	struct ftdi_context fc;
	struct ftdispi_context fsc;
	char rbuf[2];
	int i;

	if (ftdi_init(&fc) < 0) {
		fprintf(stderr, "ftdi_init failed\n");
		return 1;
	}

	ftdi_set_interface(&fc, INTERFACE_ANY);
	struct ftdi_device_list *devlist;
	int res;
	if ((res = ftdi_usb_find_all(&fc, &devlist, 0, 0)) < 0) {
		fprintf(stderr, "No FTDI with default VID/PID found\n");
		return res;
	}
	if (res == 1) {
		i = ftdi_usb_open_dev(&fc, devlist[0].dev);
		if (i < 0) {
			fprintf(stderr, "Unable to open device %d: (%s)\n", 0, ftdi_get_error_string(&fc));
			return i;
		}
	}

	if (res == 0) {
		fprintf(stderr, "No devices found\n");
		return 1;
	}

	rbuf[0] = 0x75 | 0x80;

	ftdispi_open(&fsc, &fc, INTERFACE_A);
	ftdispi_setmode(&fsc, 1, 0, 0, 0, 0, 0);
	ftdispi_setclock(&fsc, 200000);
	ftdispi_setloopback(&fsc, 0);
	ftdispi_write_read(&fsc, rbuf, 1, rbuf, 1, 0);
	ftdispi_close(&fsc, 1);
	
	printf("%.2X\n", rbuf[0]);
	return 0;
}
