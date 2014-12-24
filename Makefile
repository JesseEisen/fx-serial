all:
	gcc fx-serial.c main.c -lpthread -o fx-serial_example

shared:
	gcc fx-serial.c -fPIC -shared -o libfx-serial.so -lpthread
clean:
	rm -rf fx-serial_example *.so
