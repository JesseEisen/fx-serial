#include <stdio.h>
#include "fx-serial.h"

int main(int argc, char *argv[]) 
{
	int data;
	struct fx_serial *ss = fx_serial_start("/dev/ttyUSB0", 9600, '7', 'N', '1');

	fx_register_set(ss, 120, 100);
	fx_register_get(ss, 120, &data);
	printf("D[%d] register data is :%d\n", 120, data);

	fx_serial_stop(ss);
	return 0;
}
