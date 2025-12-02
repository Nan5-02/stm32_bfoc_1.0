#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>

#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55

void modbus_init(void);
uint8_t modbus_progress(void);
void modbus_update(void);
void modbus_loop(void);

uint8_t modbus_get_motor_start_flag(void);

#endif

