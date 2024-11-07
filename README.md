# ModBus IO
A ModBus Client running on a STM32 Nucleo Board featuring follwing IOs:

- Encoder interface
- 4 Analog Inputs
- 10 Digital Outputs
- 16 Digital inputs

## Default configuration

The addresses can be defined in `common/modbus.h`.

Device ID: 1
Baud rate:. 115200, 8N1

### Read holding/read input register
| Addr  | IO           |
| ------|------------- |
| 0     | Analog in 1  |
| 1     | Analog in 2  |
| 2     | Analog in 3  |
| 3     | Analog in 4  |
| 4     | Encoder high |
| 5     | Encoder low  |

### Configuration holding register
| Addr  | IO                         |
| ------|----------------------------|
| 10    | Number of averages for ADC |
| 11    | Button pulse length        |

### Coils
| Addr    |                                           |
|---------|-------------------------------------------|
| 0 - 9   | GPIO outputs                              |
| 10 - 25 | GPIO inputs (read only)                   |
| 26 - 41 | Button inputs with pull-ups (read only) * |

*Extended button logic:
- 1. A button is read as `1` when pressed.
- 2. The buttons are software debounced.
- 3. A button press events generates a pulse of 100 ms (can be changed with write holding 11) if the button is pressed shorter than that.

This is useful to reduce the ModBus poll rate.


## Testing

### Set single registers/coils via python

1. install https://github.com/favalex/modbus-cli 
    by: pip install modbus_cli
2.  Read holding regs and coils in combination with watch:
    `watch -n 0.1 modbus /dev/ttyUSB0 -b 115200 -s 1 h@2 3 4 5 c@0 c@1 c@2`
3. Write values:
    `modbus /dev/ttyUSB0 -b 115200 -s 1 c@0=1`
    `modbus /dev/ttyUSB0 -b 115200 -s 1 h@10=200

### Using modpoll
Run one of the examples scripts in `modpoll/` :
```
read_coils.ah
read_holding.sh
```

### Using a C program using libmodbus
See `libmodbus_client/`

## Development

### Generate Code

The STM32 drivers are included in this project to ensure version compatibility. But I didn't find a way in CubeMX to set the driver paths correctly.
So when generating code from CubeMX, the changes in following files need to be discarded:
```
stm32f3xx_it.C
.cxproject
.mxproject
.project
```
(Unless if you make bigger changes, then you need to check them manually)

