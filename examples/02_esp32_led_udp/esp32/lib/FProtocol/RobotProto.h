#ifndef ROBOT_H
#define ROBOT_H

#include "fprotocol.h"

extern uint8_t led; /*Index: 0x0001 */
int16_t callback_led(uint16_t type, uint32_t from, uint16_t error_code);
void write_led(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_led(fprotocol_handler *handler,uint16_t node);



extern fprotocol_get_index_info_t robot_index_info;

#endif /* GENERATED_H */
