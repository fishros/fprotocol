#ifndef ROBOT_H
#define ROBOT_H

#include "fprotocol.h"

typedef struct {
    uint8_t status_charge;
    uint8_t volatge;
} __attribute__((packed))  data_t;

extern uint8_t led; /*Index: 0x0001 */
extern data_t data; /*Index: 0x02 */
int16_t callback_data(uint16_t type, uint32_t from, uint16_t error_code);
void write_led(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_led(fprotocol_handler *handler,uint16_t node);
void write_data(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_data(fprotocol_handler *handler,uint16_t node);



extern fprotocol_get_index_info_t robot_index_info;

#endif /* GENERATED_H */
