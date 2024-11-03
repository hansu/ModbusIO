/*
 * Test program to plot analog values.
 * Run with: ./libmodbus_plot > an.csv && ./plot.py
 * Simple ModBUs client using libmodus library
 * API: https://libmodbus.org/reference/modbus_read_registers/
 */

#include <stdio.h>
#include <unistd.h>
#include <modbus.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <asm/ioctls.h>

#define NB_REGS 2
#define UART_PATH "/dev/ttyUSB0"

/*
This function uses the Modbus function code 0x01 (read coil status):
modbus_read_bits(ctx, 10, 26, input_coils);

This function uses the Modbus function code 0x02 (read input status):
modbus_read_input_bits(ctx, addr, 1, input_coils);

This function uses the Modbus function code 0x03 (read holding registers):
ret = modbus_read_registers(ctx, addr, NB_REGS, holding_registers);

This function uses the Modbus function code 0x04 (read input registers):
modbus_read_input_registers(ctx, addr, NB_REGS, holding_registers);

This function uses the Modbus function code 0x05 (force single coil):
modbus_write_bit(ctx, addr, TRUE);

This function uses the Modbus function code 0x0F (force multiple coils):
modbus_write_bits(ctx, 0, 10, output_coils);
*/


int main()
{
    int ret;
    modbus_t *ctx;
    uint16_t holding_registers [0xFF];
    uint8_t read_coils [0xFF];
    uint8_t write_coils [0xFF];

    struct timeval response_timeout;
    response_timeout.tv_sec = 1;
    response_timeout.tv_usec = 0;

    ctx = modbus_new_rtu(UART_PATH, 115200, 'N', 8, 1);
    if (ctx == NULL) {
        perror("Unable to create the libmodbus context\n");
        return -1;
    }

    modbus_set_response_timeout(ctx, response_timeout.tv_sec, response_timeout.tv_usec);

    ret = modbus_set_slave(ctx, 1);
    if(ret < 0){
        perror("modbus_set_slave error\n");
        return -1;
    }

/*	ret = modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS232);
    if(ret < 0){
        perror("modbus_rtu_set_serial_mode error\n");
        return;
    }
*/
    ret = modbus_connect(ctx);
    if(ret < 0){
        perror("modbus_connect error\n");
        return -1;
    }

    fprintf(stderr, "Starting printing of analog channels to stdout. Stop by pressing first modbus input button...");
    int counter = 0;
    for(;;){
        int addr = 2;
        int count = 4;
        // Modbus function code 0x03 (read holding registers)
        ret = modbus_read_registers(ctx, addr, count, holding_registers);
        if(ret < 0){
            perror("modbus_read_regs error\n");
            return -1;
        }
        printf("%4d", counter);
        for(int i=0; i<count; i++){
            printf("; %-4d", holding_registers[i]);
        }
        printf("\n");
        addr = 26;
        count = 1;
        // Modbus function code 0x01 (read coil status)
        ret = modbus_read_bits(ctx, addr, count, read_coils);
        if(ret < 0){
            perror("modbus error\n");
            return -1;
        }
        if(read_coils[0] == 1){
            fprintf(stderr, "\n");
            return 0;
        }

        // if(counter++ > 100) return 0;
        // fprintf(stderr, "%d\n", counter);
        usleep(100);
    }

    modbus_close(ctx);
    modbus_free(ctx);

}
