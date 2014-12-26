#ifndef FX_SERIAL_H_
#define FX_SERIAL_H_

// uncomment for print useful info
//#define DEBUG_PRINT

// for example: 
// struct fx_serial *ss = fx_serial_start("/dev/ttyUSB0", 9600, '7', 'N', '1');
struct fx_serial* fx_serial_start(char *device, int baude, char bits, char parity, char stop);
int fx_serial_stop(struct fx_serial *ss);

int fx_register_set(struct fx_serial *ss, int id, int data);
int fx_register_get(struct fx_serial *ss, int id, int *data);

#endif
