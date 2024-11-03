#
# modbus master client to test a modbus server
#

CFLAGS= -I/usr/include/modbus
LIBS= -lmodbus


objects = libmodbus_read libmodbus_write libmodbus_feedback libmodbus_plot

all: $(objects)

$(objects): %: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

run-read:
	make
	./libmodbus_read
run-write:
	make
	./libmodbus_write
run-fb:
	make
	./libmodbus_feedback

