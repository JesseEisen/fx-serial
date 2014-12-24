#ifndef FX_SERIAL_H_
#define FX_SERIAL_H_

struct fx_serial *fx_serial_start();
int fx_serial_stop(struct fx_serial *ss);

int controller_set(struct fx_serial *ss, int id, int status);
int controller_get(struct fx_serial *ss, int id, int *status);
int sensor_set(struct fx_serial *ss, int id, int data);
int sensor_get(struct fx_serial *ss, int id, int *data);

struct sensorX {
	int id[255];
	int data[255];
	int n;
};

int sensorX_init(struct sensorX *sx, int n_id, int *id, int *data);
int sensorX_get(struct fx_serial *ss, struct sensorX *sx);
int sensorX_set(struct fx_serial *ss, struct sensorX *sx);

#endif
