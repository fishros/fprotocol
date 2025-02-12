#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "fprotocol.h"

typedef struct {
    uint32_t arbitration_id;
    uint8_t dlc;
    uint8_t rtr;
    uint8_t data[8];
} __attribute__((packed))  canframe_t;

extern canframe_t can_recv_frame; /*Index: 0x0001 */
extern canframe_t can_send_frame; /*Index: 0x0002 */
int16_t callback_can_recv_frame(uint16_t type, uint32_t from, uint16_t error_code);
int16_t callback_can_send_frame(uint16_t type, uint32_t from, uint16_t error_code);
void write_can_recv_frame(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_can_recv_frame(fprotocol_handler *handler,uint16_t node);
void write_can_send_frame(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_can_send_frame(fprotocol_handler *handler,uint16_t node);



extern fprotocol_get_index_info_t example_index_info;

#endif /* GENERATED_H */
