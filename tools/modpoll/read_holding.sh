# -m rtu        Modbus RTU protocol (default if SERIALPORT contains a /)
# -a #          Slave address (1-255 for serial, 0-255 for TCP, 1 is default)
# -r #          Start reference (1-65536, 100 is default)
# -c #          Number of values to read (1-125, 1 is default)
# 
# -t Discrete output (coil) data type
#     -t 1          Discrete input data type
#     -t 3          16-bit input register data type
#     -t 3:hex      16-bit input register data type with hex display
#     -t 3:int      32-bit integer data type in input register table
#     -t 3:mod      32-bit module 10000 data type in input register table
#     -t 3:float    32-bit float data type in input register table
#     -t 4          16-bit output (holding) register data type (default)
#     -t 4:hex      16-bit output (holding) register data type with hex display
#     -t 4:int      32-bit integer data type in output (holding) register table
#     -t 4:mod      32-bit module 10000 type in output (holding) register table
#     -t 4:float    32-bit float data type in output (holding) register table
# 
# -1            Poll only once only, otherwise every poll rate interval
# -l #          Poll rate in ms, (1000 is default)
# 
# Options for Modbus ASCII and Modbus RTU:
# -b #          Baudrate (e.g. 9600, 19200, ...) (19200 is default)
# -d #          Databits (7 or 8 for ASCII protocol, 8 for RTU)
# -s #          Stopbits (1 or 2, 1 is default)
# -p none       No parity
# -p even       Even parity (default)
# -p odd        Odd parity

# -t 0          Discrete output (coil) data type
# -t 1          Discrete input data type
# -t 3          16-bit input register data type
# -t 4          16-bit output (holding) register data type (default)

# -b 115200 baud rate
# -d 8  data bits
# -s 1  stop bits
# -p none parity

# -l 200 poll rate 200 ms
# -a 1 slave device id
# -t 4 type 4 = holding
# -r 1 register number (address)
# -c 4 number of values


./modpoll -m rtu /dev/ttyUSB0 -b 115200 -d 8 -s 1 -p none -a 1 \
-l 200 \
-t 4 -r 1 -c 6

