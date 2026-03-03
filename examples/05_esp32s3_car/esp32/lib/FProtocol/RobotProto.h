#ifndef ROBOT_H
#define ROBOT_H

#include "fprotocol.h"

typedef struct {
    float x;
    float y;
    float z;
} __attribute__((packed))  float3d_t;

typedef struct {
    uint16_t _data_size;
    char data[512];
} __attribute__((packed))  nav_opt_t;

extern float3d_t cmd_vel; /*Index: 0x0001 */
extern nav_opt_t nav_target; /*Index: 0x0002 */
int16_t callback_cmd_vel(uint16_t type, uint8_t from, uint16_t error_code);
void write_cmd_vel(fprotocol_handler *handler,uint8_t node,uint8_t response);
void read_cmd_vel(fprotocol_handler *handler,uint8_t node);
void write_nav_target(fprotocol_handler *handler,uint8_t node,uint8_t response);
void read_nav_target(fprotocol_handler *handler,uint8_t node);



extern fprotocol_get_index_info_t robot_index_info;

#endif /* GENERATED_H */
