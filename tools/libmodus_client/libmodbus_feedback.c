/*
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

    for(int i=0; i<10; i++){
        write_coils[i ] = 0;
    }
    
    // Modbus function code 0x0F (force multiple coils)
    ret = modbus_write_bits(ctx, 0, 10, write_coils);
    if(ret < 0){
        perror("modbus error\n");
        return -1;
    }
    sleep(2);

    for(;;){

        int addr = 0;
        int count = 26;
        // Modbus function code 0x01 (read coil status)
        ret = modbus_read_bits(ctx, addr, count, read_coils);
        if(ret < 0){
            perror("modbus error\n");
            return -1;
        }
        int offset = 0;
        for(int i=0; i<10; i++){
            if(i==1) offset = 6;
            write_coils[i] = !read_coils[i+10+offset];
            // printf("i: %d, copy %d to %d\n", i, read_coils[i+10], write_coils[i]);
        }
        
        // Modbus function code 0x0F (force multiple coils)
        ret = modbus_write_bits(ctx, 0, 10, write_coils);
        if(ret < 0){
            perror("modbus error\n");
            return -1;
        }
        printf("Coil outputs:\n");
        for(int i=0; i<10; i++)  printf("[%2d]  ", addr++ );
        printf("\n");
        for(int i=0; i<10; i++) printf(" %2d   ",read_coils[i]);
        printf("\nCoil inputs:\n");
        for(int i=10; i<26; i++)printf("[%2d]  ", addr++ );
        printf("\n");
        for(int i=10; i<26; i++)printf(" %2d   ",read_coils[i]);
        printf("\n\n");

        usleep(200000);
    }

    modbus_close(ctx);
    modbus_free(ctx);

}
