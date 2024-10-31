#
# modbus master client to test a modbus server
#

CFLAGS= -I/usr/include/modbus
LIBS= -lmodbus

all: client-read client-write client-fb

client-read: libmodbus_read.c
	$(CC) $(CFLAGS) libmodbus_read.c -o libmodbus_read $(LIBS)
client-write: libmodbus_write.c
	$(CC) $(CFLAGS) libmodbus_write.c -o libmodbus_write $(LIBS)
client-fb: libmodbus_feedback.c
	$(CC) $(CFLAGS) libmodbus_feedback.c -o libmodbus_feedback $(LIBS)

run-read:
	make
	./libmodbus_read

run-write:
	make
	./libmodbus_write
run-fb:
	make
	./libmodbus_feedback

