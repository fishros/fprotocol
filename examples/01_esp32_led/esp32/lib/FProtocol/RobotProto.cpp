#include "RobotProto.h"

uint8_t led;

static const FieldDescriptor uint8_t_fields[] = {
0, TYPE_UINT8, -1, 0
};

static const StructDescriptor uint8_t_desc = {
    .size = sizeof(uint8_t),
    .field_count = 1,
    .fields = uint8_t_fields,
};
    
fprotocol_data data_table[] = {
    {0x0001, sizeof(led), &led, callback_led,&uint8_t_desc},
};

fprotocol_data *robot_fprotocol_get_index_info(uint16_t index)
{
    switch (index)
    {
    case 0x0001:
        return &data_table[0];
        break;
    default:
        break;
    }
    return NULL;
}
void write_led(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0001, &led, sizeof(led),&uint8_t_desc);
}
void read_led(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0001, &led, sizeof(led),&uint8_t_desc);
}
fprotocol_get_index_info_t robot_index_info = robot_fprotocol_get_index_info;