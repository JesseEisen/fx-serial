#include <stdio.h>
#include "fx-serial.h"

int main(int argc, char *argv[]) 
{
	struct fx_serial *ss = fx_serial_start();
#define OPEN  1
#define CLOSE 0

	while (1) {
	//	controller_set(ss, 120, CLOSE);

	//	int status;
	//	controller_get(ss, 120, &status);

	//	printf("controller %d status is %d\n", 120, status);

	//	sleep(3);
	
	//	controller_set(ss, 120, CLOSE);
	//	sleep(3);

	//	sensor_set(ss, 20, 100);
	//	int status;
	//	sensor_get(ss, 20, &status);

	//	printf("sensor %d status is %d\n", 20, status);
	//	
	//	sleep(3);

		struct sensorX sx;
		int id[] = {1, 10, 20, 40};
		int data[] = {10, 20, 30, 40};
		
		sensorX_init(&sx, sizeof(id)/sizeof(int), id, data);
		sensorX_set(ss, &sx);

		struct sensorX sx_test;
		int id2[] = {1, 10, 20, 40};
		sensorX_init(&sx_test, sizeof(id2)/sizeof(int), id2, NULL);
		sensorX_get(ss, &sx_test);
	
		printf("Sensor Data:\n");
		printf("=========================\n");
		int i;
		for (i = 0; i < sx_test.n; i++) {
			printf("sensor id[%d] = %d\n", sx_test.id[i], sx_test.data[i]);
		}
		printf("=========================\n");
	
		sleep(3);
	}

	while (1) getchar();
	fx_serial_stop(ss);

	return 0;
}
