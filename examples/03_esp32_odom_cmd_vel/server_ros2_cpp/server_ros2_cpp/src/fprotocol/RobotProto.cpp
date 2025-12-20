#include "RobotProto.h"

uint8_t led;
float3d_t odom;
float3d_t cmd_vel;

static const FieldDescriptor float3d_t_fields[] = {
    {offsetof(float3d_t, x), TYPE_FLOAT, -1, },
    {offsetof(float3d_t, y), TYPE_FLOAT, -1, },
    {offsetof(float3d_t, z), TYPE_FLOAT, -1, },
};

static const StructDescriptor float3d_t_desc = {
    .size = sizeof(float3d_t),
    .field_count = 3,
    .fields = float3d_t_fields,
};

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
    {0x0002, sizeof(odom), &odom, NULL,&float3d_t_desc},
    {0x0003, sizeof(cmd_vel), &cmd_vel, callback_cmd_vel,&float3d_t_desc},
};

fprotocol_data *robot_fprotocol_get_index_info(uint16_t index)
{
    switch (index)
    {
    case 0x0001:
        return &data_table[0];
        break;
    case 0x0002:
        return &data_table[1];
        break;
    case 0x0003:
        return &data_table[2];
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
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0001, &led, 0,&uint8_t_desc);
}
void write_odom(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0002, &odom, sizeof(odom),&float3d_t_desc);
}
void read_odom(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0002, &odom, 0,&float3d_t_desc);
}
void write_cmd_vel(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0003, &cmd_vel, sizeof(cmd_vel),&float3d_t_desc);
}
void read_cmd_vel(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0003, &cmd_vel, 0,&float3d_t_desc);
}
fprotocol_get_index_info_t robot_index_info = robot_fprotocol_get_index_info;