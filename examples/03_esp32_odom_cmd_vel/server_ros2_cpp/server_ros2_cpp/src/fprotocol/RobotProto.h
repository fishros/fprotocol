#ifndef ROBOT_H
#define ROBOT_H

#include "fprotocol.h"

typedef struct {
    float x;
    float y;
    float z;
} __attribute__((packed))  float3d_t;

extern uint8_t led; /*Index: 0x0001 */
extern float3d_t odom; /*Index: 0x0002 */
extern float3d_t cmd_vel; /*Index: 0x0003 */
int16_t callback_led(uint16_t type, uint32_t from, uint16_t error_code);
int16_t callback_cmd_vel(uint16_t type, uint32_t from, uint16_t error_code);
void write_led(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_led(fprotocol_handler *handler,uint16_t node);
void write_odom(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_odom(fprotocol_handler *handler,uint16_t node);
void write_cmd_vel(fprotocol_handler *handler,uint16_t node,uint8_t response);
void read_cmd_vel(fprotocol_handler *handler,uint16_t node);



extern fprotocol_get_index_info_t robot_index_info;

#endif /* GENERATED_H */
