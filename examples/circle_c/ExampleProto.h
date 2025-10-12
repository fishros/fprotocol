#ifndef EXAMPLE_H
#define EXAMPLE_H

#include "fprotocol.h"

typedef struct {
    uint16_t _data_size;
    uint8_t data[1024];
    uint16_t _data1_size;
    uint8_t data1[1024];
} __attribute__((packed))  rawuint8_t;

extern rawuint8_t write_485; /*Index: 0x0001 */
extern rawuint8_t read_485; /*Index: 0x0002 */
extern uint8_t test; /*Index: 0x0003 */
int16_t callback_write_485(uint16_t type, uint32_t from, uint16_t error_code);
int16_t callback_read_485(uint16_t type, uint32_t from, uint16_t error_code);
void write_write_485(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_write_485(fprotocol_handler *handler,uint16_t node);
void write_read_485(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_read_485(fprotocol_handler *handler,uint16_t node);
void write_test(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_test(fprotocol_handler *handler,uint16_t node);



extern fprotocol_get_index_info_t example_index_info;

#endif /* GENERATED_H */
