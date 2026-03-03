#include "RobotProto.h"

float3d_t cmd_vel;
nav_opt_t nav_target;

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

static const FieldDescriptor nav_opt_t_fields[] = {
    {offsetof(nav_opt_t, _data_size), TYPE_UINT16, -1, 0},
    {offsetof(nav_opt_t, data), TYPE_CHAR, 0, 512},
};

static const StructDescriptor nav_opt_t_desc = {
    .size = sizeof(nav_opt_t),
    .field_count = 2,
    .fields = nav_opt_t_fields,
};

fprotocol_data data_table[] = {
    {0x0001, sizeof(cmd_vel), &cmd_vel, callback_cmd_vel,&float3d_t_desc},
    {0x0002, sizeof(nav_target), &nav_target, NULL,&nav_opt_t_desc},
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
    default:
        break;
    }
    return NULL;
}
void write_cmd_vel(fprotocol_handler *handler,uint8_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0001, &cmd_vel, sizeof(cmd_vel),&float3d_t_desc);
}
void read_cmd_vel(fprotocol_handler *handler,uint8_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0001, &cmd_vel, 0,&float3d_t_desc);
}
void write_nav_target(fprotocol_handler *handler,uint8_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0002, &nav_target, sizeof(nav_target),&nav_opt_t_desc);
}
void read_nav_target(fprotocol_handler *handler,uint8_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0002, &nav_target, 0,&nav_opt_t_desc);
}
fprotocol_get_index_info_t robot_index_info = robot_fprotocol_get_index_info;